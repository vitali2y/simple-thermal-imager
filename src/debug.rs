use anyhow::Result;
use image::{ImageBuffer, Rgb};

use crate::{Detection, THERMAL_HEIGHT, THERMAL_WIDTH};

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

/// rendering normalized thermal image + red mask tint + boxes + track IDs
/// returns RGB24 bytes, `stable` may be empty (no tint)
pub fn render_debug_frame(
    thermal_raw: &[u16],
    stable: &[bool],
    detections: &[Detection],
) -> Vec<u8> {
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

pub fn save_debug_png(
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
