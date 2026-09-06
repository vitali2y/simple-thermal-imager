//! Thermal motion detector / blob tracker
//! Tested on TOOLTOP T7 (InfiRay-based) camera

use anyhow::{Context, Result, bail};
use serde::Serialize;
use std::{
    io::Write,
    path::Path,
    time::{SystemTime, UNIX_EPOCH},
};
use v4l::{
    buffer::Type,
    io::mmap::Stream,
    io::traits::CaptureStream,
    video::Capture,
    {Device, FourCC},
};

use cli::{Args, parse_cal, parse_roi};
use debug::{render_debug_frame, save_debug_png};
use detector::{dilate3x3, erode3x3, find_blobs, merge_nearby_detections};
use thermal::{
    box_blur3x3, extract_thermal, frame_stats, init_background, is_black_frame, run_diagnostics,
};
use tracker::Tracker;

mod cli;
mod debug;
mod detector;
mod thermal;
mod tracker;

const WIDTH: u32 = 256;
const HEIGHT: u32 = 384;

pub(crate) const THERMAL_WIDTH: u32 = 256;
pub(crate) const THERMAL_HEIGHT: u32 = 192;
pub(crate) const THERMAL_PIXELS: usize = (THERMAL_WIDTH * THERMAL_HEIGHT) as usize;

// JSON output types
#[derive(Serialize, Debug, Clone)]
pub(crate) struct Detection {
    id: usize,
    x: u32,
    y: u32,
    width: u32,
    height: u32,
    area: usize,
    center_x: u32,
    center_y: u32,
    max_temp_raw: u16,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_temp_c: Option<f32>,
}

#[derive(Serialize)]
struct Event {
    timestamp: f64,
    detections: Vec<Detection>,
}

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

// device / stream helpers
fn open_device(device: &str) -> Result<Device> {
    if let Ok(index) = device.parse::<usize>() {
        return Device::new(index).context("failed to open device by index");
    }

    Device::with_path(Path::new(device)).context("failed to open device by path")
}

fn main() -> Result<()> {
    let args: Args = argh::from_env();

    let dev = open_device(&args.device)?;

    let mut fmt = dev.format()?;
    fmt.width = WIDTH;
    fmt.height = HEIGHT;
    fmt.fourcc = FourCC::new(b"YUYV");
    dev.set_format(&fmt)?;

    let mut stream = Stream::new(&dev, Type::VideoCapture)?;

    if args.diagnose {
        run_diagnostics(&mut stream, args.warmup)?;
        return Ok(());
    }

    let roi = parse_roi(&args.roi)?;
    let is_top = args.half == "top";
    let is_high_uv = args.packing == "high_uv";

    let cal = if args.no_temp {
        None
    } else {
        let a = parse_cal(&args.cal0)?;
        let b = parse_cal(&args.cal1)?;
        if (a.0 - b.0).abs() < 1.0 {
            bail!("cal0 and cal1 raw values must differ");
        }
        Some((a, b))
    };

    if let Some(dir) = &args.png_dir {
        std::fs::create_dir_all(dir).with_context(|| format!("failed to create {}", dir))?;
        eprintln!("debug PNGs -> {}", dir);
    }

    let mut preview_in: Option<std::process::ChildStdin> = None;
    let mut _preview_child: Option<std::process::Child> = None;

    if args.preview {
        let mut cmd = std::process::Command::new("ffplay");
        cmd.args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
        ])
        .arg("-video_size")
        .arg(format!("{}x{}", THERMAL_WIDTH, THERMAL_HEIGHT))
        .args(["-framerate", "25", "-autoexit", "-i", "pipe:0"])
        .stdin(std::process::Stdio::piped());

        let mut child = cmd
            .spawn()
            .context("failed to spawn ffplay (is ffmpeg installed?)")?;

        preview_in = child.stdin.take();
        _preview_child = Some(child);
        eprintln!("live preview started with ffplay...");
    }

    // init persistent buffers
    let mut background = init_background(&mut stream, is_top, is_high_uv, args.warmup)?;
    let mut foreground_age = vec![0u16; THERMAL_PIXELS];
    let mut prev_thermal: Option<Vec<u16>> = None;

    let mut tracker = Tracker::new(args.track_dist, args.track_miss);
    let mut settle_left = args.settle;
    let mut frame_no: u64 = 0;
    let mut emitted_records: usize = 0;
    let mut ambient_est: u16 = 0;
    let mut ambient_init = false;
    let mut global_streak: u32 = 0;

    eprintln!("starting motion detection loop...");

    loop {
        let (frame, _meta) = stream.next()?;

        let stats = frame_stats(frame);
        if is_black_frame(&stats) {
            continue;
        }

        let thermal_raw = extract_thermal(frame, is_top, is_high_uv);
        if thermal_raw.len() != background.len() {
            continue;
        }

        frame_no += 1;

        let thermal = box_blur3x3(&thermal_raw, THERMAL_WIDTH, THERMAL_HEIGHT);

        if let Some(prev) = &prev_thermal {
            let mut sum_abs = 0u64;
            let mut sum_signed = 0i64;
            let mut changed_100 = 0usize;
            let mut changed_200 = 0usize;

            for i in 0..thermal.len() {
                let d = thermal[i] as i32 - prev[i] as i32;

                sum_abs += d.unsigned_abs() as u64;
                sum_signed += d as i64;

                if d.unsigned_abs() >= 100 {
                    changed_100 += 1;
                }

                if d.unsigned_abs() >= 200 {
                    changed_200 += 1;
                }
            }

            let mean_abs = sum_abs as f64 / thermal.len() as f64;
            let mean_signed = sum_signed as f64 / thermal.len() as f64;
            let sensor_jump = changed_100 > (thermal.len() * 9) / 10 && mean_abs >= 80.0;

            if mean_abs >= 20.0 || changed_100 > thermal.len() / 20 {
                eprintln!(
                    "frame jump {:>6} | mean_abs {:>7.2} | mean_signed {:+8.2} | >=100 {:>6} | >=200 {:>6}",
                    frame_no, mean_abs, mean_signed, changed_100, changed_200,
                );
            }
            if sensor_jump {
                let offset = mean_signed.round() as i32;

                for value in &mut background {
                    *value = ((*value as i32 + offset).clamp(0, u16::MAX as i32)) as u16;
                }

                eprintln!(
                    "sensor-wide jump at frame {}, shifting background by {:+}",
                    frame_no, offset
                );
            }
        }

        prev_thermal = Some(thermal.clone());

        if settle_left > 0 {
            background.copy_from_slice(&thermal);
            foreground_age.fill(0);
            settle_left -= 1;

            if settle_left == 0 {
                eprintln!("sensor settled; detection armed");
            }

            if let Some(stdin) = preview_in.as_mut() {
                let rgb = render_debug_frame(&thermal_raw, &[], &[]);
                if stdin.write_all(&rgb).is_err() {
                    eprintln!("preview pipe broken, disabling preview");
                    preview_in = None;
                }
            }

            continue;
        }

        let mut raw_min = u16::MAX;
        let mut raw_max = 0u16;
        for &v in &thermal_raw {
            raw_min = raw_min.min(v);
            raw_max = raw_max.max(v);
        }

        let mut mask = vec![false; thermal.len()];
        let mut active = 0usize;
        let mut active_hot = 0usize;
        let mut active_cold = 0usize;

        // motion/background loop
        for i in 0..thermal.len() {
            let signed_diff = thermal[i] as i32 - background[i] as i32;
            let diff = signed_diff.unsigned_abs() as u16;

            if diff > args.threshold {
                mask[i] = true;
                active += 1;

                if signed_diff > 0 {
                    active_hot += 1;
                } else {
                    active_cold += 1;
                }

                foreground_age[i] = foreground_age[i].saturating_add(1);

                if foreground_age[i] >= 25 {
                    background[i] = ((background[i] as u32 * 99 + thermal[i] as u32) / 100) as u16;
                }
            } else {
                foreground_age[i] = 0;

                background[i] = ((background[i] as u32 * 19 + thermal[i] as u32) / 20) as u16;
            }
        }

        if active > (thermal.len() * 3) / 4 {
            global_streak += 1;
        } else {
            global_streak = 0;
        }

        if global_streak >= 3 {
            eprintln!(
                "global change ({} px for {} frames), re-learning motion background",
                active, global_streak
            );

            background.copy_from_slice(&thermal);
            foreground_age.fill(0);
            global_streak = 0;

            // do not turn a camera-wide thermal shift into object detections
            continue;
        }

        let dilated = dilate3x3(&mask, THERMAL_WIDTH, THERMAL_HEIGHT);
        let stable = erode3x3(&dilated, THERMAL_WIDTH, THERMAL_HEIGHT);

        let mut warm_px = 0usize;
        if args.warm_delta > 0 {
            let mut sorted = thermal_raw.clone();
            sorted.sort_unstable();
            let p10 = sorted[sorted.len() / 10];

            if !ambient_init {
                ambient_est = p10;
                ambient_init = true;
            } else {
                ambient_est = ((ambient_est as u32 * 99 + p10 as u32) / 100) as u16;
            }

            for &value in &thermal_raw {
                if value > ambient_est + args.warm_delta {
                    warm_px += 1;
                }
            }
        }

        let mut detections = find_blobs(
            &stable,
            &thermal_raw,
            THERMAL_WIDTH,
            THERMAL_HEIGHT,
            args.min_area,
            &roi,
        );

        detections = merge_nearby_detections(detections, 8);

        tracker.update(&mut detections, frame_no);
        detections.retain(|d| tracker.is_confirmed(d.id));

        if let Some(((r0, c0), (r1, c1))) = cal {
            for d in detections.iter_mut() {
                d.max_temp_c = Some(c0 + (d.max_temp_raw as f32 - r0) * (c1 - c0) / (r1 - r0));
            }
        }

        if let Some(stdin) = preview_in.as_mut() {
            let rgb = render_debug_frame(&thermal_raw, &stable, &detections);
            if stdin.write_all(&rgb).is_err() {
                eprintln!("preview pipe broken, disabling preview");
                preview_in = None;
            }
        }

        // periodic stats block
        if args.stats && frame_no % 25 == 0 {
            eprintln!(
                "frame {:>6} | active {:>6} (+{:>6}/-{:>6}) | streak {} | ambient {:>5} | warm {:>6} | raw {:>5}/{:>5} | blobs {}",
                frame_no,
                active,
                active_hot,
                active_cold,
                global_streak,
                ambient_est,
                warm_px,
                raw_min,
                raw_max,
                detections.len()
            );
        }

        // throttled event trace: anything triggering at all
        if args.stats && (active > 0 || warm_px > 0) && frame_no % 5 == 0 {
            eprintln!(
                "event frame {:>6} | active {:>6} (+{:>6}/-{:>6}) | warm {:>6} | rawmax {:>5} | blobs {}",
                frame_no,
                active,
                active_hot,
                active_cold,
                warm_px,
                raw_max,
                detections.len()
            );
        }

        if !detections.is_empty() {
            emitted_records += 1;

            if let Some(dir) = &args.png_dir {
                if args.png_every <= 1 || emitted_records % args.png_every == 0 {
                    if let Err(e) =
                        save_debug_png(dir, frame_no, &thermal_raw, &stable, &detections)
                    {
                        eprintln!("warning: failed to save debug PNG: {}", e);
                    }
                }
            }

            let event = Event {
                timestamp: now_secs(),
                detections,
            };

            println!("{}", serde_json::to_string(&event)?);
        }
    }
}
