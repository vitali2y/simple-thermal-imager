use anyhow::{Context, Result, bail};
use argh::FromArgs;

// CLI
#[derive(FromArgs)]
/// Thermal motion detector / blob tracker
pub struct Args {
    /// video device path (default: "/dev/video0") or index, e.g. /dev/video0 or 0
    #[argh(option, default = "String::from(\"/dev/video0\")")]
    pub device: String,

    /// region of interest: x, y, w, h
    #[argh(option)]
    pub roi: Option<String>,

    /// minimum blob area in pixels (default: 100)
    #[argh(option, default = "100")]
    pub min_area: usize,

    /// raw-value difference threshold for movement (default: 300)
    #[argh(option, default = "300")]
    pub threshold: u16,

    /// run diagnostics and save PNGs
    #[argh(switch)]
    pub diagnose: bool,

    /// thermal half: top or bottom (default: "bottom")
    #[argh(option, default = "String::from(\"bottom\")")]
    pub half: String,

    /// packing: high_uv or high_y (default: "high_uv")
    #[argh(option, default = "String::from(\"high_uv\")")]
    pub packing: String,

    /// max frames to wait for camera warmup (default: 500)
    #[argh(option, default = "500")]
    pub warmup: usize,

    /// frames to let the sensor settle after warmup before arming (default: 125)
    #[argh(option, default = "125")]
    pub settle: usize,

    /// max distance (px) to associate a detection with an existing track (default: "60.0")
    #[argh(option, default = "60.0")]
    pub track_dist: f32,

    /// frames a track may be missing before it is dropped (default: 12)
    #[argh(option, default = "12")]
    pub track_miss: u64,

    /// report objects warmer than ambient by this raw delta (0 = motion only; default: 800)
    #[argh(option, default = "800")]
    pub warm_delta: u16,

    /// calibration point 0 as raw:celsius (approximate T7 values; default: "19050:23.0")
    #[argh(option, default = "String::from(\"19050:23.0\")")]
    pub cal0: String,

    /// calibration point 1 as raw:celsius (default: "22700:34.5")
    #[argh(option, default = "String::from(\"22700:34.5\")")]
    pub cal1: String,

    /// do not add max_temp_c to JSON output
    #[argh(switch)]
    pub no_temp: bool,

    /// directory to save a visualization PNG for each emitted JSON record
    #[argh(option)]
    pub png_dir: Option<String>,

    /// save only every Nth record's PNG (1 = every record; default: 1)
    #[argh(option, default = "1")]
    pub png_every: usize,

    /// open a live preview ffplay window with the annotated thermal view
    #[argh(switch)]
    pub preview: bool,

    /// print detection statistics to stderr
    #[argh(switch)]
    pub stats: bool,
}

// parsing helpers
pub fn parse_roi(input: &Option<String>) -> Result<Option<(u32, u32, u32, u32)>> {
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

pub fn parse_cal(s: &str) -> Result<(f32, f32)> {
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
