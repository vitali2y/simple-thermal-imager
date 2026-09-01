//! Captures thermal crops for each detection,
//! saves normalized PNG per detection to disk,
//! organized into directories by track ID
//!
//! Directory structure:
//!   crops/
//!     track_0001/
//!       frame_000042_19812.png     (frame number + max_temp_raw)
//!     track_0002/
//!       ...
//!
//! After collecting enough data, manually sort into class directories:
//!   dataset/
//!     person/
//!     hand/
//!     dog/
//!     car/
//!     ...
//!
//! Then train a classifier (e.g. YOLOv8-cls or MobileNet) and export to ONNX.

use anyhow::{Context, Result, bail};
use argh::FromArgs;
use image::{ImageBuffer, Luma};
use std::path::Path;

use v4l::buffer::Type;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture;
use v4l::{Device, FourCC};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 384;
const THERMAL_WIDTH: u32 = 256;
const THERMAL_HEIGHT: u32 = 192;
const THERMAL_PIXELS: usize = (THERMAL_WIDTH * THERMAL_HEIGHT) as usize;

#[derive(FromArgs)]
/// Save thermal crops per tracked blob for classifier training
struct Args {
    /// video device path or index
    #[argh(option, default = "String::from(\"/dev/video0\")")]
    device: String,

    /// thermal half: top or bottom
    #[argh(option, default = "String::from(\"bottom\")")]
    half: String,

    /// packing: high_uv or high_y
    #[argh(option, default = "String::from(\"high_uv\")")]
    packing: String,

    /// max frames to wait for camera warmup
    #[argh(option, default = "500")]
    warmup: usize,

    /// frames to let the sensor settle
    #[argh(option, default = "125")]
    settle: usize,

    /// raw-value difference threshold for movement
    #[argh(option, default = "600")]
    threshold: u16,

    /// minimum blob area in pixels
    #[argh(option, default = "100")]
    min_area: usize,

    /// max tracking distance (px)
    #[argh(option, default = "60.0")]
    track_dist: f32,

    /// max missed frames per track
    #[argh(option, default = "12")]
    track_miss: u64,

    /// output directory
    #[argh(option, default = "String::from(\"crops\")")]
    out_dir: String,

    /// margin (px) to add around each crop
    #[argh(option, default = "8")]
    margin: u32,

    /// max crops to save (0 = unlimited)
    #[argh(option, default = "0")]
    max_crops: usize,
}

// reused types from motion_detector_tracker
#[derive(Debug, Clone, Copy)]
struct FrameStats {
    avg_y: u8,
    avg_u: u8,
    avg_v: u8,
}

fn frame_stats(frame: &[u8]) -> FrameStats {
    let mut sum_y = 0u64;
    let mut count_y = 0u64;
    let mut sum_u = 0u64;
    let mut count_u = 0u64;
    let mut sum_v = 0u64;
    let mut count_v = 0u64;

    for (i, &b) in frame.iter().enumerate() {
        match i % 4 {
            0 | 2 => {
                sum_y += b as u64;
                count_y += 1;
            }
            1 => {
                sum_u += b as u64;
                count_u += 1;
            }
            _ => {
                sum_v += b as u64;
                count_v += 1;
            }
        }
    }

    FrameStats {
        avg_y: if count_y > 0 {
            (sum_y / count_y) as u8
        } else {
            0
        },
        avg_u: if count_u > 0 {
            (sum_u / count_u) as u8
        } else {
            0
        },
        avg_v: if count_v > 0 {
            (sum_v / count_v) as u8
        } else {
            0
        },
    }
}

fn is_black_frame(s: &FrameStats) -> bool {
    s.avg_y <= 2 && s.avg_u >= 124 && s.avg_u <= 132 && s.avg_v >= 124 && s.avg_v <= 132
}

fn extract_thermal(frame: &[u8], is_top: bool, is_high_uv: bool) -> Vec<u16> {
    let half_bytes = THERMAL_PIXELS * 2;
    let data = if is_top {
        frame.get(..half_bytes).unwrap_or(&[])
    } else {
        frame.get(half_bytes..).unwrap_or(&[])
    };

    let mut pixels = Vec::with_capacity(THERMAL_PIXELS);
    for i in (0..data.len()).step_by(4) {
        if i + 3 >= data.len() {
            break;
        }
        let y0 = data[i] as u16;
        let u = data[i + 1] as u16;
        let y1 = data[i + 2] as u16;
        let v = data[i + 3] as u16;

        let (p0, p1) = if is_high_uv {
            ((u << 8) | y0, (v << 8) | y1)
        } else {
            ((y0 << 8) | u, (y1 << 8) | v)
        };

        pixels.push(p0);
        pixels.push(p1);
    }
    pixels
}

fn box_blur3x3(src: &[u16], w: u32, h: u32) -> Vec<u16> {
    let (w, h) = (w as usize, h as usize);
    let mut out = vec![0u16; src.len()];
    for y in 0..h {
        for x in 0..w {
            let mut sum = 0u32;
            let mut n = 0u32;
            for dy in 0..3isize {
                let yy = y as isize + dy - 1;
                if yy < 0 || yy >= h as isize {
                    continue;
                }
                for dx in 0..3isize {
                    let xx = x as isize + dx - 1;
                    if xx < 0 || xx >= w as isize {
                        continue;
                    }
                    sum += src[yy as usize * w + xx as usize] as u32;
                    n += 1;
                }
            }
            out[y * w + x] = (sum / n) as u16;
        }
    }
    out
}

// blob detection (same as motion_detector_tracker)
#[derive(Default)]
struct Blob {
    min_x: u32,
    max_x: u32,
    min_y: u32,
    max_y: u32,
    area: usize,
    sum_x: u64,
    sum_y: u64,
    max_val: u16,
}

impl Blob {
    fn add(&mut self, x: u32, y: u32, val: u16) {
        if self.area == 0 {
            self.min_x = x;
            self.max_x = x;
            self.min_y = y;
            self.max_y = y;
            self.area = 1;
            self.sum_x = x as u64;
            self.sum_y = y as u64;
            self.max_val = val;
            return;
        }
        self.min_x = self.min_x.min(x);
        self.max_x = self.max_x.max(x);
        self.min_y = self.min_y.min(y);
        self.max_y = self.max_y.max(y);
        self.area += 1;
        self.sum_x += x as u64;
        self.sum_y += y as u64;
        self.max_val = self.max_val.max(val);
    }
}

struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new() -> Self {
        Self {
            parent: vec![0],
            rank: vec![0],
        }
    }
    fn add(&mut self) -> usize {
        let id = self.parent.len();
        self.parent.push(id);
        self.rank.push(0);
        id
    }
    fn find(&mut self, item: usize) -> usize {
        if self.parent[item] != item {
            self.parent[item] = self.find(self.parent[item]);
        }
        self.parent[item]
    }
    fn union(&mut self, a: usize, b: usize) -> usize {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return ra;
        }
        if self.rank[ra] < self.rank[rb] {
            self.parent[ra] = rb;
            rb
        } else if self.rank[ra] > self.rank[rb] {
            self.parent[rb] = ra;
            ra
        } else {
            self.parent[rb] = ra;
            self.rank[ra] += 1;
            ra
        }
    }
}

struct Detection {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
    area: usize,
    center_x: u32,
    center_y: u32,
    max_val: u16,
    id: usize,
}

fn find_blobs(mask: &[bool], values: &[u16], w: u32, h: u32, min_area: usize) -> Vec<Detection> {
    let (ww, hh) = (w as usize, h as usize);
    let mut labels = vec![0usize; mask.len()];
    let mut uf = UnionFind::new();

    for y in 0..hh {
        for x in 0..ww {
            let idx = y * ww + x;
            if !mask[idx] {
                continue;
            }
            let left = if x > 0 { labels[idx - 1] } else { 0 };
            let up = if y > 0 { labels[idx - ww] } else { 0 };
            labels[idx] = match (left, up) {
                (0, 0) => uf.add(),
                (l, 0) => uf.find(l),
                (0, u) => uf.find(u),
                (l, u) => uf.union(l, u),
            };
        }
    }

    let mut blobs = std::collections::HashMap::new();
    for y in 0..hh {
        for x in 0..ww {
            let idx = y * ww + x;
            if !mask[idx] {
                continue;
            }
            let label = uf.find(labels[idx]);
            blobs
                .entry(label)
                .or_insert_with(Blob::default)
                .add(x as u32, y as u32, values[idx]);
        }
    }

    blobs
        .into_iter()
        .filter(|(_, b)| b.area >= min_area)
        .map(|(id, b)| Detection {
            id,
            x: b.min_x,
            y: b.min_y,
            width: b.max_x - b.min_x + 1,
            height: b.max_y - b.min_y + 1,
            area: b.area,
            center_x: (b.sum_x / b.area as u64) as u32,
            center_y: (b.sum_y / b.area as u64) as u32,
            max_val: b.max_val,
        })
        .collect()
}

// tracker
#[derive(Clone)]
struct Track {
    id: u64,
    cx: f32,
    cy: f32,
    last_seen: u64,
}

struct Tracker {
    tracks: Vec<Track>,
    next_id: u64,
    max_dist_sq: f32,
    max_missing: u64,
}

impl Tracker {
    fn new(max_dist: f32, max_missing: u64) -> Self {
        Self {
            tracks: Vec::new(),
            next_id: 1,
            max_dist_sq: max_dist * max_dist,
            max_missing,
        }
    }

    fn update(&mut self, dets: &mut [Detection], frame_no: u64) {
        for det in dets.iter_mut() {
            let (dx, dy) = (det.center_x as f32, det.center_y as f32);
            let mut best: Option<(usize, f32)> = None;

            for (i, t) in self.tracks.iter().enumerate() {
                if t.last_seen == frame_no {
                    continue;
                }
                let d = (t.cx - dx).powi(2) + (t.cy - dy).powi(2);
                if d < self.max_dist_sq && best.map_or(true, |(_, bd)| d < bd) {
                    best = Some((i, d));
                }
            }

            match best {
                Some((i, _)) => {
                    let t = &mut self.tracks[i];
                    t.cx = t.cx * 0.5 + dx * 0.5;
                    t.cy = t.cy * 0.5 + dy * 0.5;
                    t.last_seen = frame_no;
                    det.id = t.id as usize;
                }
                None => {
                    det.id = self.next_id as usize;
                    self.tracks.push(Track {
                        id: self.next_id,
                        cx: dx,
                        cy: dy,
                        last_seen: frame_no,
                    });
                    self.next_id += 1;
                }
            }
        }

        self.tracks
            .retain(|t| frame_no - t.last_seen <= self.max_missing);
    }
}

// crop saving
fn save_crop(
    thermal_raw: &[u16],
    det: &Detection,
    frame_no: u64,
    out_dir: &str,
    margin: u32,
) -> Result<()> {
    let x0 = det.x.saturating_sub(margin);
    let y0 = det.y.saturating_sub(margin);
    let x1 = (det.x + det.width + margin).min(THERMAL_WIDTH);
    let y1 = (det.y + det.height + margin).min(THERMAL_HEIGHT);

    let cw = x1 - x0;
    let ch = y1 - y0;

    if cw == 0 || ch == 0 {
        return Ok(());
    }

    // Extract crop
    let mut crop = Vec::with_capacity((cw * ch) as usize);
    for y in y0..y1 {
        for x in x0..x1 {
            crop.push(thermal_raw[(y * THERMAL_WIDTH + x) as usize]);
        }
    }

    // Normalize to 8-bit for visual inspection / training
    let min = *crop.iter().min().unwrap();
    let max = *crop.iter().max().unwrap();
    let range = (max - min).max(1) as f32;

    let pixels: Vec<u8> = crop
        .iter()
        .map(|&v| (((v - min) as f32 / range) * 255.0).clamp(0.0, 255.0) as u8)
        .collect();

    let dir = format!("{}/track_{:04}", out_dir, det.id);
    std::fs::create_dir_all(&dir)?;

    let filename = format!("{}/frame_{:06}_{}.png", dir, frame_no, det.max_val);

    let img =
        ImageBuffer::<Luma<u8>, Vec<u8>>::from_vec(cw, ch, pixels).context("crop image buffer")?;

    img.save(&filename)?;

    Ok(())
}

fn main() -> Result<()> {
    let args: Args = argh::from_env();

    let dev = if let Ok(idx) = args.device.parse::<usize>() {
        Device::new(idx)?
    } else {
        Device::with_path(Path::new(&args.device))?
    };

    let mut fmt = dev.format()?;
    fmt.width = WIDTH;
    fmt.height = HEIGHT;
    fmt.fourcc = FourCC::new(b"YUYV");
    dev.set_format(&fmt)?;

    let mut stream = Stream::new(&dev, Type::VideoCapture)?;

    let is_top = args.half == "top";
    let is_high_uv = args.packing == "high_uv";

    // warmup
    eprintln!("Waiting for camera warmup...");
    let mut background = vec![0u16; THERMAL_PIXELS];
    let mut bg_count = 0u64;

    for i in 0..args.warmup {
        let (frame, _) = stream.next()?;
        let stats = frame_stats(frame);
        if is_black_frame(&stats) {
            if i % 25 == 0 {
                eprintln!("frame {:>4}: warming up...", i);
            }
            continue;
        }

        let th = box_blur3x3(
            &extract_thermal(frame, is_top, is_high_uv),
            THERMAL_WIDTH,
            THERMAL_HEIGHT,
        );

        if th.len() == THERMAL_PIXELS {
            for j in 0..THERMAL_PIXELS {
                background[j] =
                    ((background[j] as u64 * bg_count + th[j] as u64) / (bg_count + 1)) as u16;
            }
            bg_count += 1;
        }

        if bg_count >= 10 {
            break;
        }
    }

    if bg_count == 0 {
        bail!("No valid frames during warmup. Try --warmup 1000.");
    }

    eprintln!("Background from {} frames", bg_count);

    // settle
    eprintln!("Settling sensor ({} frames)...", args.settle);
    for _ in 0..args.settle {
        let (frame, _) = stream.next()?;
        let stats = frame_stats(frame);
        if is_black_frame(&stats) {
            continue;
        }
        let th = box_blur3x3(
            &extract_thermal(frame, is_top, is_high_uv),
            THERMAL_WIDTH,
            THERMAL_HEIGHT,
        );
        if th.len() == THERMAL_PIXELS {
            background.copy_from_slice(&th);
        }
    }

    eprintln!("Sensor settled. Collecting crops → {}/", args.out_dir);
    std::fs::create_dir_all(&args.out_dir)?;

    let mut tracker = Tracker::new(args.track_dist, args.track_miss);
    let mut prev_mask = vec![false; THERMAL_PIXELS];
    let mut frame_no: u64 = 0;
    let mut total_crops: usize = 0;

    loop {
        let (frame, _) = stream.next()?;

        let stats = frame_stats(frame);
        if is_black_frame(&stats) {
            continue;
        }

        let thermal_raw = extract_thermal(frame, is_top, is_high_uv);
        if thermal_raw.len() != THERMAL_PIXELS {
            continue;
        }

        frame_no += 1;

        let thermal = box_blur3x3(&thermal_raw, THERMAL_WIDTH, THERMAL_HEIGHT);

        // update background
        let mut mask = vec![false; THERMAL_PIXELS];
        let mut active = 0usize;

        for i in 0..THERMAL_PIXELS {
            background[i] = ((background[i] as u32 * 19 + thermal[i] as u32) / 20) as u16;

            let diff = if thermal[i] > background[i] {
                thermal[i] - background[i]
            } else {
                background[i] - thermal[i]
            };

            if diff > args.threshold {
                mask[i] = true;
                active += 1;
            }
        }

        // global drift guard
        if active > THERMAL_PIXELS / 3 {
            eprintln!("Global change, re-learning");
            background.copy_from_slice(&thermal);
            prev_mask.fill(false);
            continue;
        }

        // temporal confirmation
        let mut stable = vec![false; THERMAL_PIXELS];
        for i in 0..THERMAL_PIXELS {
            stable[i] = mask[i] && prev_mask[i];
        }
        prev_mask.copy_from_slice(&mask);

        let mut detections = find_blobs(
            &stable,
            &thermal_raw,
            THERMAL_WIDTH,
            THERMAL_HEIGHT,
            args.min_area,
        );

        if detections.is_empty() {
            continue;
        }

        tracker.update(&mut detections, frame_no);

        for det in &detections {
            if let Err(e) = save_crop(&thermal_raw, det, frame_no, &args.out_dir, args.margin) {
                eprintln!("Warning: failed to save crop: {}", e);
            }

            total_crops += 1;

            eprintln!(
                "crop #{:>5} | track {:>4} | frame {:>6} | {}x{} @ ({},{}) | area {:>5} | raw {:>5}",
                total_crops,
                det.id,
                frame_no,
                det.width,
                det.height,
                det.x,
                det.y,
                det.area,
                det.max_val
            );
        }

        if args.max_crops > 0 && total_crops >= args.max_crops {
            eprintln!("Reached --max-crops {}. Done.", args.max_crops);
            break;
        }
    }

    Ok(())
}
