// Capture the 256x384 frame, split it, decode the bottom half into raw 16-bit thermal values

// ➜  simple-thermal-imager git:(master) ✗ cargo r --example save_raw_grayscale
//     Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.97s
//      Running `target/debug/examples/save_raw_grayscale`
// captured 196608 bytes
// saved thermal_raw_16bit.png
// captured 10240 bytes of metadata
// ➜  simple-thermal-imager git:(master) ✗

use anyhow::Result;
use image::{ImageBuffer, Luma};
use v4l::{
    buffer::Type,
    io::{mmap::Stream, traits::CaptureStream},
    video::Capture,
    {Device, FourCC},
};

fn main() -> Result<()> {
    let width: u32 = 256;
    let height: u32 = 384; // the stacked height

    // open /dev/video0
    let dev = Device::new(0)?;

    // force YUYV format at 256x384
    let mut fmt = dev.format()?;
    fmt.width = width;
    fmt.height = height;
    fmt.fourcc = FourCC::new(b"YUYV");
    dev.set_format(&fmt)?;

    // create stream with VideoCapture buffer type
    let mut stream = Stream::new(&dev, Type::VideoCapture)?;

    // get the next frame using CaptureStream::next()
    let (frame, _meta) = stream.next()?;

    println!("captured {} bytes", frame.len());

    // calculate the split point (bottom half starts at 256 * 192 * 2 bytes)
    let top_half_size = (width as usize) * ((height as usize) / 2) * 2;
    let raw_thermal_data = &frame[top_half_size..];

    // decode the YUYV packed 16-bit raw data
    // YUYV format: [Y0, U, Y1, V] -> 4 bytes for 2 pixels
    let pixel_count = (width as usize) * ((height as usize) / 2);
    let mut raw_pixels: Vec<u16> = Vec::with_capacity(pixel_count);

    for i in (0..raw_thermal_data.len()).step_by(4) {
        let y0 = raw_thermal_data[i] as u16;
        let u = raw_thermal_data[i + 1] as u16;
        let y1 = raw_thermal_data[i + 2] as u16;
        let v = raw_thermal_data[i + 3] as u16;

        // standard InfiRay packing: High byte in U/V, Low byte in Y
        let pixel0 = (u << 8) | y0;
        let pixel1 = (v << 8) | y1;

        raw_pixels.push(pixel0);
        raw_pixels.push(pixel1);
    }

    // save the raw thermal data as a 16-bit grayscale PNG
    let thermal_img: ImageBuffer<Luma<u16>, Vec<u16>> =
        ImageBuffer::from_vec(width, height / 2, raw_pixels).expect("image buffer size mismatch");

    thermal_img.save("thermal_raw_16bit.png")?;
    println!("saved thermal_raw_16bit.png");

    // reading the metadata (like spot temperatures)
    // open the metadata node /dev/video1
    let dev = Device::new(1)?;

    let mut stream = Stream::new(&dev, Type::MetaCapture)?;

    // getting raw UVC metadata payload
    let (buf, _meta) = stream.next()?;
    println!("captured {} bytes of metadata", buf.len());
    // TODO: to parse metadata according to InfiRay UVC metadata spec (contains a header followed by an array of spot temperatures)

    Ok(())
}
