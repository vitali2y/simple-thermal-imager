// Dump metadata from /dev/video1

use anyhow::Result;
use v4l::Device;
use v4l::buffer::Type;
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;

fn main() -> Result<()> {
    let dev = Device::new(1)?;

    let mut stream = Stream::new(&dev, Type::MetaCapture)?;

    for i in 0..20 {
        let (buf, _meta) = stream.next()?;

        let filename = format!("meta_{:02}.bin", i);
        std::fs::write(&filename, buf)?;

        eprintln!("Saved {} ({} bytes)", filename, buf.len());
    }

    Ok(())
}
