use anyhow::{Context, Result, bail};
use image::{ImageBuffer, Luma};
use v4l::{io::mmap::Stream, io::traits::CaptureStream};

use crate::{THERMAL_HEIGHT, THERMAL_PIXELS, THERMAL_WIDTH};

// frame statistics / black-frame detection
#[derive(Debug, Clone, Copy)]
pub struct FrameStats {
    min_y: u8,
    max_y: u8,
    avg_y: u8,
    min_u: u8,
    max_u: u8,
    avg_u: u8,
    min_v: u8,
    max_v: u8,
    avg_v: u8,
}

pub fn frame_stats(frame: &[u8]) -> FrameStats {
    let mut min_y = 255u8;
    let mut max_y = 0u8;
    let mut sum_y = 0u64;
    let mut count_y = 0u64;

    let mut min_u = 255u8;
    let mut max_u = 0u8;
    let mut sum_u = 0u64;
    let mut count_u = 0u64;

    let mut min_v = 255u8;
    let mut max_v = 0u8;
    let mut sum_v = 0u64;
    let mut count_v = 0u64;

    for (i, b) in frame.iter().enumerate() {
        let b = *b;
        match i % 4 {
            0 | 2 => {
                min_y = min_y.min(b);
                max_y = max_y.max(b);
                sum_y += b as u64;
                count_y += 1;
            }
            1 => {
                min_u = min_u.min(b);
                max_u = max_u.max(b);
                sum_u += b as u64;
                count_u += 1;
            }
            _ => {
                min_v = min_v.min(b);
                max_v = max_v.max(b);
                sum_v += b as u64;
                count_v += 1;
            }
        }
    }

    FrameStats {
        min_y,
        max_y,
        avg_y: if count_y > 0 {
            (sum_y / count_y) as u8
        } else {
            0
        },
        min_u,
        max_u,
        avg_u: if count_u > 0 {
            (sum_u / count_u) as u8
        } else {
            0
        },
        min_v,
        max_v,
        avg_v: if count_v > 0 {
            (sum_v / count_v) as u8
        } else {
            0
        },
    }
}

/// neutral/black YUYV frame is Y ~= 0, U ~= 128, V ~= 128
pub fn is_black_frame(stats: &FrameStats) -> bool {
    stats.avg_y <= 2
        && stats.avg_u >= 124
        && stats.avg_u <= 132
        && stats.avg_v >= 124
        && stats.avg_v <= 132
}

fn range_score(stats: &FrameStats) -> u32 {
    let y = (stats.max_y as u32).saturating_sub(stats.min_y as u32);
    let u = (stats.max_u as u32).saturating_sub(stats.min_u as u32);
    let v = (stats.max_v as u32).saturating_sub(stats.min_v as u32);
    y + u + v
}

// thermal extraction / processing
pub fn extract_thermal(frame: &[u8], is_top: bool, is_high_uv: bool) -> Vec<u16> {
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

pub fn box_blur3x3(src: &[u16], width: u32, height: u32) -> Vec<u16> {
    let w = width as usize;
    let h = height as usize;
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

fn u16_stats(data: &[u16]) -> (u16, u16, f64) {
    if data.is_empty() {
        return (0, 0, 0.0);
    }

    let mut min = u16::MAX;
    let mut max = 0u16;
    let mut sum = 0u64;

    for &v in data {
        min = min.min(v);
        max = max.max(v);
        sum += v as u64;
    }

    (min, max, sum as f64 / data.len() as f64)
}

fn save_normalized(path: &str, data: &[u16], width: u32, height: u32) -> Result<()> {
    if data.is_empty() {
        bail!("empty thermal data");
    }

    let expected = (width * height) as usize;
    if data.len() != expected {
        bail!(
            "thermal data length {} does not match expected size {}",
            data.len(),
            expected
        );
    }

    let min = *data.iter().min().unwrap();
    let max = *data.iter().max().unwrap();
    let range = (max - min).max(1) as f32;

    let buf: Vec<u8> = data
        .iter()
        .map(|&v| {
            let normalized = ((v - min) as f32 / range) * 255.0;
            normalized.clamp(0.0, 255.0) as u8
        })
        .collect();

    let img = ImageBuffer::<Luma<u8>, Vec<u8>>::from_vec(width, height, buf)
        .with_context(|| format!("failed to create image buffer for {}", path))?;

    img.save(path)?;

    Ok(())
}

pub fn init_background(
    stream: &mut Stream<'_>,
    is_top: bool,
    is_high_uv: bool,
    warmup_frames: usize,
) -> Result<Vec<u16>> {
    eprintln!(
        "initializing background model (waiting up to {} frames, about {} seconds)...",
        warmup_frames,
        warmup_frames / 25
    );

    let mut acc = vec![0u64; THERMAL_PIXELS];
    let mut count = 0u64;

    for frame_index in 0..warmup_frames {
        let (frame, _meta) = stream.next()?;
        let stats = frame_stats(frame);

        if is_black_frame(&stats) {
            if frame_index % 25 == 0 {
                eprintln!("frame {:>4}: waiting for camera warmup...", frame_index);
            }
            continue;
        }

        let thermal = box_blur3x3(
            &extract_thermal(frame, is_top, is_high_uv),
            THERMAL_WIDTH,
            THERMAL_HEIGHT,
        );

        if thermal.len() != acc.len() {
            continue;
        }

        for i in 0..acc.len() {
            acc[i] += thermal[i] as u64;
        }

        count += 1;

        if count >= 10 {
            break;
        }
    }

    if count == 0 {
        bail!(
            "could not initialize background: no valid non-black frames after {} frames; try --warmup 1000",
            warmup_frames
        );
    }

    eprintln!("background initialized from {} frames", count);

    Ok(acc.iter().map(|v| (v / count) as u16).collect())
}

pub fn run_diagnostics(stream: &mut Stream<'_>, warmup_frames: usize) -> Result<()> {
    eprintln!(
        "capturing frames for diagnostics (up to {} frames, about {} seconds)...",
        warmup_frames,
        warmup_frames / 25
    );

    let mut best_frame: Option<Vec<u8>> = None;
    let mut best_score = 0u32;
    let mut non_black_frames = 0usize;

    for frame_index in 0..warmup_frames {
        let (frame, _meta) = stream.next()?;
        let stats = frame_stats(frame);

        if is_black_frame(&stats) {
            if frame_index % 25 == 0 {
                eprintln!("frame {:>4}: still black/uninitialized", frame_index);
            }
            continue;
        }

        non_black_frames += 1;

        let score = range_score(&stats);

        if best_frame.is_none() || score > best_score {
            best_score = score;
            best_frame = Some(frame.to_vec());
        }

        if non_black_frames >= 10 {
            eprintln!("got {} non-black frames", non_black_frames);
            break;
        }
    }

    let Some(frame) = best_frame else {
        bail!(
            "no non-black frames after {} frames; try a larger value, e.g. --warmup 1000",
            warmup_frames
        );
    };

    let stats = frame_stats(&frame);

    eprintln!();
    eprintln!("selected diagnostic frame:");
    eprintln!(
        "  Y min/max/avg: {}/{}/{}",
        stats.min_y, stats.max_y, stats.avg_y
    );
    eprintln!(
        "  U min/max/avg: {}/{}/{}",
        stats.min_u, stats.max_u, stats.avg_u
    );
    eprintln!(
        "  V min/max/avg: {}/{}/{}",
        stats.min_v, stats.max_v, stats.avg_v
    );

    let end = 16.min(frame.len());
    eprintln!("  first {} bytes: {:02x?}", end, &frame[..end]);
    eprintln!();

    let configs = [
        ("top_high_uv", true, true),
        ("top_high_y", true, false),
        ("bottom_high_uv", false, true),
        ("bottom_high_y", false, false),
    ];

    for (name, is_top, is_high_uv) in configs {
        let thermal = extract_thermal(&frame, is_top, is_high_uv);
        let (min, max, mean) = u16_stats(&thermal);

        eprintln!(
            "config: {:<16} | Pixels: {:>6} | Min: {:>5} | Max: {:>5} | Range: {:>5} | Mean: {:>8.2}",
            name,
            thermal.len(),
            min,
            max,
            max.saturating_sub(min),
            mean
        );

        let filename = format!("diag_{}.png", name);

        save_normalized(&filename, &thermal, THERMAL_WIDTH, THERMAL_HEIGHT)
            .with_context(|| format!("failed to save {}", filename))?;
    }

    eprintln!("saved diagnostic PNGs");
    eprintln!("for this camera the radiometric data is: --half bottom --packing high_uv");

    Ok(())
}
