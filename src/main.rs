//! Thermal motion detector / blob tracker
//! Tested on TOOLTOP T7 (InfiRay-based) camera
//!
//! Pipeline:
//!   UVC YUYV 256x384 frame
//!     -> bottom half = raw 16-bit radiometric data, packing (U<<8)|Y / (V<<8)|Y
//!     -> 3x3 blur -> adaptive background (IIR) -> diff threshold   (motion)
//!     -> slow ambient percentile + warm delta                      (presence)
//!     -> temporal confirmation (2 consecutive frames)
//!     -> connected components (union-find) -> blobs
//!     -> nearest-neighbor tracker -> stable IDs
//!     -> optional raw -> Celsius calibration
//!     -> JSON events on stdout; logs on stderr
//!     -> optional live preview (ffplay) / debug PNGs

use anyhow::{Context, Result, bail};
use argh::FromArgs;
use image::{ImageBuffer, Luma, Rgb};
use serde::Serialize;
use std::{
    collections::HashMap,
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

const WIDTH: u32 = 256;
const HEIGHT: u32 = 384;

const THERMAL_WIDTH: u32 = 256;
const THERMAL_HEIGHT: u32 = 192;
const THERMAL_PIXELS: usize = (THERMAL_WIDTH * THERMAL_HEIGHT) as usize;

// CLI
#[derive(FromArgs)]
/// Thermal motion detector / blob tracker
struct Args {
    /// video device path (default: "/dev/video0") or index, e.g. /dev/video0 or 0
    #[argh(option, default = "String::from(\"/dev/video0\")")]
    device: String,

    /// region of interest: x, y, w, h
    #[argh(option)]
    roi: Option<String>,

    /// minimum blob area in pixels (default: 100)
    #[argh(option, default = "100")]
    min_area: usize,

    /// raw-value difference threshold for movement (default: 600)
    #[argh(option, default = "600")]
    threshold: u16,

    /// run diagnostics and save PNGs
    #[argh(switch)]
    diagnose: bool,

    /// thermal half: top or bottom (default: "bottom")
    #[argh(option, default = "String::from(\"bottom\")")]
    half: String,

    /// packing: high_uv or high_y (default: "high_uv")
    #[argh(option, default = "String::from(\"high_uv\")")]
    packing: String,

    /// max frames to wait for camera warmup (default: 500)
    #[argh(option, default = "500")]
    warmup: usize,

    /// frames to let the sensor settle after warmup before arming (default: 125)
    #[argh(option, default = "125")]
    settle: usize,

    /// max distance (px) to associate a detection with an existing track (default: "60.0")
    #[argh(option, default = "60.0")]
    track_dist: f32,

    /// frames a track may be missing before it is dropped (default: 12)
    #[argh(option, default = "12")]
    track_miss: u64,

    /// report objects warmer than ambient by this raw delta (0 = motion only; default: 800)
    #[argh(option, default = "800")]
    warm_delta: u16,

    /// calibration point 0 as raw:celsius (approximate T7 values; default: "19050:23.0")
    #[argh(option, default = "String::from(\"19050:23.0\")")]
    cal0: String,

    /// calibration point 1 as raw:celsius (default: "22700:34.5")
    #[argh(option, default = "String::from(\"22700:34.5\")")]
    cal1: String,

    /// do not add max_temp_c to JSON output
    #[argh(switch)]
    no_temp: bool,

    /// directory to save a visualization PNG for each emitted JSON record
    #[argh(option)]
    png_dir: Option<String>,

    /// save only every Nth record's PNG (1 = every record; default: 1)
    #[argh(option, default = "1")]
    png_every: usize,

    /// open a live preview ffplay window with the annotated thermal view
    #[argh(switch)]
    preview: bool,

    /// print detection statistics to stderr
    #[argh(switch)]
    stats: bool,
}

// JSON output types
#[derive(Serialize, Debug, Clone)]
struct Detection {
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

// frame statistics / black-frame detection
#[derive(Debug, Clone, Copy)]
struct FrameStats {
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

fn frame_stats(frame: &[u8]) -> FrameStats {
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

/// neutral/black YUYV frame is Y ~= 0, U ~= 128, V ~= 128.
fn is_black_frame(stats: &FrameStats) -> bool {
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

fn box_blur3x3(src: &[u16], width: u32, height: u32) -> Vec<u16> {
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

// parsing helpers
fn parse_roi(input: &Option<String>) -> Result<Option<(u32, u32, u32, u32)>> {
    let Some(input) = input else {
        return Ok(None);
    };

    let parts: Result<Vec<u32>, _> = input
        .split(',')
        .map(|part| part.trim().parse::<u32>())
        .collect();

    let parts = parts.context("failed to parse ROI")?;

    if parts.len() != 4 {
        bail!("ROI must have format x,y,w,h");
    }

    Ok(Some((parts[0], parts[1], parts[2], parts[3])))
}

fn parse_cal(s: &str) -> Result<(f32, f32)> {
    let mut it = s.split(':');
    let raw: f32 = it
        .next()
        .context("calibration must be raw:celsius")?
        .trim()
        .parse()
        .context("calibration raw value")?;
    let c: f32 = it
        .next()
        .context("calibration must be raw:celsius")?
        .trim()
        .parse()
        .context("calibration celsius value")?;

    Ok((raw, c))
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

fn init_background(
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

fn run_diagnostics(stream: &mut Stream<'_>, warmup_frames: usize) -> Result<()> {
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

    eprintln!();
    eprintln!("saved diagnostic PNGs.");
    eprintln!("for this camera the radiometric data is: --half bottom --packing high_uv");
    eprintln!();

    Ok(())
}

// blob detection (connected components)
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
        let root_a = self.find(a);
        let root_b = self.find(b);

        if root_a == root_b {
            return root_a;
        }

        if self.rank[root_a] < self.rank[root_b] {
            self.parent[root_a] = root_b;
            root_b
        } else if self.rank[root_a] > self.rank[root_b] {
            self.parent[root_b] = root_a;
            root_a
        } else {
            self.parent[root_b] = root_a;
            self.rank[root_a] += 1;
            root_a
        }
    }
}

fn find_blobs(
    mask: &[bool],
    values: &[u16],
    width: u32,
    height: u32,
    min_area: usize,
    roi: &Option<(u32, u32, u32, u32)>,
) -> Vec<Detection> {
    debug_assert_eq!(mask.len(), values.len());

    let w = width as usize;
    let h = height as usize;

    let mut labels = vec![0usize; mask.len()];
    let mut uf = UnionFind::new();

    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;

            if !mask[idx] {
                continue;
            }

            let left = if x > 0 { labels[idx - 1] } else { 0 };
            let up = if y > 0 { labels[idx - w] } else { 0 };

            labels[idx] = match (left, up) {
                (0, 0) => uf.add(),
                (l, 0) => uf.find(l),
                (0, u) => uf.find(u),
                (l, u) => uf.union(l, u),
            };
        }
    }

    let mut blobs: HashMap<usize, Blob> = HashMap::new();

    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;

            if !mask[idx] {
                continue;
            }

            let label = uf.find(labels[idx]);
            blobs
                .entry(label)
                .or_default()
                .add(x as u32, y as u32, values[idx]);
        }
    }

    let mut detections: Vec<Detection> = blobs
        .into_iter()
        .filter(|(_, blob)| blob.area >= min_area)
        .filter(|(_, blob)| {
            let Some((rx, ry, rw, rh)) = roi else {
                return true;
            };

            blob.max_x >= *rx
                && blob.min_x < *rx + *rw
                && blob.max_y >= *ry
                && blob.min_y < *ry + *rh
        })
        .map(|(id, blob)| Detection {
            id,
            x: blob.min_x,
            y: blob.min_y,
            width: blob.max_x - blob.min_x + 1,
            height: blob.max_y - blob.min_y + 1,
            area: blob.area,
            center_x: (blob.sum_x / blob.area as u64) as u32,
            center_y: (blob.sum_y / blob.area as u64) as u32,
            max_temp_raw: blob.max_val,
            max_temp_c: None,
        })
        .collect();

    detections.sort_by(|a, b| b.area.cmp(&a.area));

    detections
}

// tracker (stable IDs across frames)
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

    fn update(&mut self, detections: &mut [Detection], frame_no: u64) {
        for det in detections.iter_mut() {
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

// debug rendering (preview window / PNG dumps)
const DIGIT_FONT: [[u8; 5]; 10] = [
    [7, 5, 5, 5, 7], // 0
    [2, 6, 2, 2, 7], // 1
    [7, 1, 7, 4, 7], // 2
    [7, 1, 7, 1, 7], // 3
    [5, 5, 7, 1, 1], // 4
    [7, 4, 7, 1, 7], // 5
    [7, 4, 7, 5, 7], // 6
    [7, 1, 1, 1, 1], // 7
    [7, 5, 7, 5, 7], // 8
    [7, 5, 7, 1, 7], // 9
];

fn draw_glyph(
    img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>,
    gx: u32,
    gy: u32,
    rows: &[u8; 5],
    scale: u32,
    c: Rgb<u8>,
) {
    let (iw, ih) = img.dimensions();

    for (ry, row) in rows.iter().enumerate() {
        let ry = ry as u32;

        for cx in 0..3u32 {
            if row & (1 << (2 - cx)) != 0 {
                for sy in 0..scale {
                    for sx in 0..scale {
                        let px = gx + cx * scale + sx;
                        let py = gy + ry * scale + sy;

                        if px < iw && py < ih {
                            img.put_pixel(px, py, c);
                        }
                    }
                }
            }
        }
    }
}

fn draw_number(
    img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>,
    n: usize,
    x: u32,
    y: u32,
    scale: u32,
    c: Rgb<u8>,
) {
    let mut cx = x;

    for ch in n.to_string().chars() {
        if let Some(d) = ch.to_digit(10) {
            draw_glyph(img, cx, y, &DIGIT_FONT[d as usize], scale, c);
        }
        cx += 4 * scale;
    }
}

fn draw_rect(img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>, x: u32, y: u32, w: u32, h: u32, c: Rgb<u8>) {
    let (iw, ih) = img.dimensions();
    let x1 = (x + w).min(iw);
    let y1 = (y + h).min(ih);

    for px in x..x1 {
        if y < ih {
            img.put_pixel(px, y, c);
        }
        if y1 > 0 {
            img.put_pixel(px, y1 - 1, c);
        }
    }

    for py in y..y1 {
        if x < iw {
            img.put_pixel(x, py, c);
        }
        if x1 > 0 {
            img.put_pixel(x1 - 1, py, c);
        }
    }
}

/// render normalized thermal image + red mask tint + boxes + track IDs
/// returns RGB24 bytes, `stable` may be empty (no tint)
fn render_debug_frame(thermal_raw: &[u16], stable: &[bool], detections: &[Detection]) -> Vec<u8> {
    let w = THERMAL_WIDTH as usize;
    let h = THERMAL_HEIGHT as usize;

    let mut min = u16::MAX;
    let mut max = 0u16;
    for &v in thermal_raw {
        min = min.min(v);
        max = max.max(v);
    }
    let range = (max - min).max(1) as f32;

    let mut img = ImageBuffer::<Rgb<u8>, Vec<u8>>::new(THERMAL_WIDTH, THERMAL_HEIGHT);

    for y in 0..h {
        for x in 0..w {
            let i = y * w + x;
            let g = (((thermal_raw[i] - min) as f32 / range) * 255.0) as u8;

            let (r, gg, b) = if stable.get(i) == Some(&true) {
                (g.max(180), g / 2, g / 2)
            } else {
                (g, g, g)
            };

            img.put_pixel(x as u32, y as u32, Rgb([r, gg, b]));
        }
    }

    for det in detections {
        draw_rect(
            &mut img,
            det.x,
            det.y,
            det.width,
            det.height,
            Rgb([0, 255, 0]),
        );
        draw_number(
            &mut img,
            det.id,
            det.x + 1,
            det.y + 1,
            2,
            Rgb([255, 255, 0]),
        );
    }

    img.into_raw()
}

fn save_debug_png(
    dir: &str,
    frame_no: u64,
    thermal_raw: &[u16],
    stable: &[bool],
    detections: &[Detection],
) -> Result<()> {
    let rgb = render_debug_frame(thermal_raw, stable, detections);

    let img = ImageBuffer::<Rgb<u8>, Vec<u8>>::from_vec(THERMAL_WIDTH, THERMAL_HEIGHT, rgb)
        .expect("rgb buffer size mismatch");

    img.save(format!("{}/frame_{:06}.png", dir, frame_no))?;

    Ok(())
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
        eprintln!("Debug PNGs -> {}", dir);
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
        eprintln!("Live preview started (ffplay).");
    }

    let mut background = init_background(&mut stream, is_top, is_high_uv, args.warmup)?;

    let mut tracker = Tracker::new(args.track_dist, args.track_miss);
    let mut prev_mask = vec![false; THERMAL_PIXELS];
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

        if settle_left > 0 {
            background.copy_from_slice(&thermal);
            prev_mask.fill(false);
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

        for i in 0..thermal.len() {
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
            prev_mask.fill(false);
            global_streak = 0;
        }

        let mut stable = vec![false; mask.len()];
        for i in 0..mask.len() {
            stable[i] = mask[i] && prev_mask[i];
        }
        prev_mask.copy_from_slice(&mask);

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

            for i in 0..stable.len() {
                if thermal_raw[i] > ambient_est + args.warm_delta {
                    stable[i] = true;
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

        tracker.update(&mut detections, frame_no);

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

        if args.stats && frame_no % 25 == 0 {
            eprintln!(
                "frame {:>6} | active {:>6} | streak {} | ambient {:>5} | warm {:>6} | raw {:>5}/{:>5} | blobs {}",
                frame_no,
                active,
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
                "event frame {:>6} | active {:>6} | warm {:>6} | rawmax {:>5} | blobs {}",
                frame_no,
                active,
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
