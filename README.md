# simple-thermal-imager

Thermal motion detector / blob tracker for TOOLTOP T7 (InfiRay-based) USB-C thermal imager:
radiometric frame decoding, motion detection, warm-object presence,
multi-object blob tracking and JSON event stream for downstream tools.


## Details

The TOOLTOP T7 (InfiRay-based, USB ID `0bda:5840`) enumerates on Linux as a
standard UVC camera exposing `YUYV 256x384 @ 25 fps`. That frame is actually
two stacked 256x192 halves:

- **top half** – processed 8-bit grayscale image (Y channel, U/V neutral);
- **bottom half** – raw 16-bit radiometric data packed as
  `pixel = (U << 8) | Y` (and `(V << 8) | Y1`).

This project decodes the radiometric half and runs the following pipeline
entirely in userspace:

```
YUYV 256x384
  -> decode bottom half to u16 raw thermal matrix
  -> 3x3 blur (denoise)
  -> adaptive background (IIR) + diff threshold      (motion)
  -> ambient percentile + warm delta                 (presence, optional)
  -> temporal confirmation (2 consecutive frames)
  -> connected components (union-find) -> blobs
  -> nearest-neighbor tracker -> stable track IDs
  -> optional raw -> Celsius calibration
  -> JSON events on stdout (and, logs on stderr)
```

The camera needs ~2–4 secs after stream start before it emits real frames
(black `00 80` frames before that); the tool handles this automatically
(`--warmup` + `--settle`).


### Output format

```json
{"timestamp":1788125629.21,"detections":[
  {"id":10,"x":225,"y":111,"width":31,"height":19,"area":391,
   "center_x":240,"center_y":119,"max_temp_raw":19828,"max_temp_c":25.6}
]}
```

`id` is a stable track ID while the object lives.


## Build

```bash
cargo build --release --examples
```

Run (requires read access to `/dev/video0`, i.e. membership in the `video` group):

```bash
sudo usermod -aG video $USER
```


## Usage

```bash
➜  simple-thermal-imager git:(master) ✗ cargo r --example motion_detector_tracker -- \
  --half bottom --packing high_uv --threshold 600 --min-area 100 --settle 125 --warm-delta 800 \
  --cal0 19050:23.0 --cal1 22700:34.5 | jq -c '.detections[] | {id, x, y, area, max_temp_c}'
   Compiling simple-thermal-imager v0.1.0 (/home/vit/prjs/simple-thermal-imager)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 1.62s
     Running `target/debug/examples/motion_detector_tracker --half bottom --packing high_uv --threshold 600
    --min-area 100 --settle 125 --warm-delta 800 --cal0 '19050:23.0' --cal1 '22700:34.5'`
initializing background model (waiting up to 500 frames, about 20 seconds)...
frame    0: waiting for camera warmup...
frame   25: waiting for camera warmup...
frame   50: waiting for camera warmup...
background initialized from 10 frames
starting motion detection loop...
Sensor settled. Detection armed.
{"id":1,"x":33,"y":16,"area":107,"max_temp_c":27.026575}
{"id":1,"x":33,"y":16,"area":108,"max_temp_c":27.013973}
{"id":1,"x":33,"y":16,"area":113,"max_temp_c":26.988768}
{"id":1,"x":32,"y":15,"area":109,"max_temp_c":26.950958}
{"id":1,"x":32,"y":15,"area":112,"max_temp_c":26.950958}
{"id":1,"x":32,"y":15,"area":114,"max_temp_c":26.950958}
{"id":1,"x":32,"y":15,"area":114,"max_temp_c":26.925753}
{"id":2,"x":153,"y":115,"area":136,"max_temp_c":22.603014}
{"id":2,"x":156,"y":112,"area":115,"max_temp_c":22.577808}
{"id":3,"x":167,"y":154,"area":1129,"max_temp_c":25.047945}
{"id":3,"x":169,"y":156,"area":960,"max_temp_c":25.047945}
{"id":3,"x":175,"y":160,"area":521,"max_temp_c":25.02274}
{"id":4,"x":85,"y":175,"area":208,"max_temp_c":25.07315}
{"id":5,"x":33,"y":16,"area":105,"max_temp_c":26.976164}
{"id":5,"x":33,"y":16,"area":111,"max_temp_c":26.976164}
^C
➜  simple-thermal-imager git:(master) ✗ ./target/debug/examples/motion_detector_tracker --help
Usage: motion_detector_tracker [--device <device>] [--roi <roi>] [--min-area <min-area>]
  [--threshold <threshold>] [--diagnose] [--half <half>] [--packing <packing>] [--warmup <warmup>]
  [--settle <settle>] [--track-dist <track-dist>] [--track-miss <track-miss>] [--warm-delta <warm-delta>]
  [--cal0 <cal0>] [--cal1 <cal1>]

Thermal motion detector / blob tracker for Tooltop T7

Options:
  --device          video device path or index, e.g. /dev/video0 or 0
  --roi             region of interest: x,y,w,h
  --min-area        minimum blob area in pixels
  --threshold       raw-value difference threshold for movement
  --diagnose        run diagnostics and save PNGs
  --half            thermal half: top or bottom
  --packing         packing: high_uv or high_y
  --warmup          max frames to wait for camera warmup
  --settle          frames to let the sensor settle after warmup before arming
  --track-dist      max distance (px) to associate a detection with an existing
                    track
  --track-miss      frames a track may be missing before it is dropped
  --warm-delta      report objects warmer than ambient by this raw delta (0 =
                    off)
  --cal0            calibration point 0 as raw:celsius, e.g. 19050:23.0
  --cal1            calibration point 1 as raw:celsius, e.g. 22700:34.5
  --help, help      display usage information

➜  simple-thermal-imager git:(master) ✗
```

Pipe to another tool:

```bash
./target/release/examples/motion_detector_tracker \
  | jq -c '.detections[] | {id, center_x, center_y, area, max_temp_c}'
```

Common options:

```bash
# pure motion mode (no presence, no temperatures)
... -- --warm-delta 0 --no-temp

# watch only a region (x,y,w,h)
... -- --roi 128,0,128,192

# tune sensitivity
... -- --threshold 600 --min-area 100

# real calibration (two reference objects measured with a contact thermometer)
... -- --cal0 19050:23.0 --cal1 22700:34.5

# tracking behaviour
... -- --track-dist 60 --track-miss 12
```

Collect training crops for a future classifier (see `examples/crop_dumper.rs`):

```bash
cargo run --release --example crop_dumper -- --max-crops 200
```


## Debug

If something looks wrong, use the diagnostic mode:

```bash
./target/release/examples/motion_detector_tracker --diagnose --warmup 1000
```

It waits out the black startup frames, picks the best frame, prints Y/U/V
statistics and saves four contrast-normalized PNGs (`diag_top_high_uv.png`,
`diag_top_high_y.png`, `diag_bottom_high_uv.png`, `diag_bottom_high_y.png`).

Interpretation for a healthy T7:

- `bottom_high_uv` shows the thermal image and has a range of ~1000–3000
  with mean ~19000–21000 → **this is the radiometric config**
  (`--half bottom --packing high_uv`);
- `top_high_*` show the processed visual image;
- `bottom_high_y` is byte-swapped garbage.

Troubleshooting:

| Symptom | Cause / fix |
|---|---|
| `xxd` shows only `0080 0080 ...` | camera still warming up; increase `--warmup`, or unplug/replug and wait |
| all diag PNGs black | no valid frames yet; `--warmup 2000`; check lens cover; verify `v4l2-ctl --list-devices` |
| one giant full-frame blob right after start | sensor still drifting; increase `--settle` |
| many tiny speckle blobs | raise `--min-area` / `--threshold` |
| one giant persistent blob with `--warm-delta` | delta too low for this scene; raise it (e.g. 800–1200) |
| `Permission denied` on `/dev/video0` | add user to `video` group |

Raw frame inspection without the tool:

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=256,height=384,pixelformat=YUYV \
  --stream-mmap=3 --stream-count=500 --stream-to=long_384.yuyv
tail -c 196608 long_384.yuyv | xxd -l 64          # top half
tail -c 196608 long_384.yuyv | xxd -s 98304 -l 64 # bottom half
```


## TODO

- [ ] generic source modes: `--mode stacked16|luma`, `--width/--height`,
      luma-appropriate thresholds (support analog/webcam sources)
- [ ] object classification: ONNX (YOLO-cls/MobileNet) via `tract` or
      `candle`, fed by `crop_dumper` datasets; add `class`/`confidence`
      to JSON
- [ ] tracking upgrades: velocity-aware matching (Kalman), enter/exit events,
      object counting per session
- [ ] calibration helper: interactive two-point calibration mode
      (`--calibrate`) that samples raw values from known references
- [ ] replace `sort_unstable` ambient estimate with a histogram percentile
      (cheaper on SBCs)
- [ ] explore `/dev/video1` UVC metadata for spot temperatures / shutter info
- [ ] per-track temperature history and simple analytics in the JSON stream
- [ ] config file support instead of long CLI lines


## License                                                                                                                                                                             

MIT license ([LICENSE](https://github.com/vitali2y/simple-thermal-imager/blob/main/LICENSE) or <http://opensource.org/licenses/MIT>)
