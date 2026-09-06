# simple-thermal-imager

  - [Details](#details)
  - [Build](#build)
  - [Usage](#usage)
    - [Output format](#output-format)
  - [Debug](#debug)
    - [Troubleshooting](#troubleshooting)
    - [Sensor-wide thermal recalibration](#sensor-wide-thermal-recalibration)
  - [Known limitations](#known-limitations)
  - [TODO](#todo)
  - [License](#license)

Thermal motion detector / blob tracker for TOOLTOP T7 (InfiRay-based) USB-C thermal imager:
radiometric frame decoding, motion detection, multi-object blob tracking,
optional thermal diagnostics, and a JSON event stream for downstream tools.


## Details

The TOOLTOP T7 device (InfiRay-based, USB ID `0bda:5840`) enumerates on Linux as a
standard UVC camera exposing YUYV 256x384 @ 25 fps.

In system logs it's reported as below:
```
==> /var/log/syslog <==
2026-09-01T18:17:45.935408+03:00 greenall kernel: usb 3-1: New USB device found, idVendor=0bda, idProduct=5840, bcdDevice=13.03
2026-09-01T18:17:45.935468+03:00 greenall kernel: usb 3-1: New USB device strings: Mfr=3, Product=1, SerialNumber=2
2026-09-01T18:17:45.935478+03:00 greenall kernel: usb 3-1: Product: USB Camera
2026-09-01T18:17:45.935484+03:00 greenall kernel: usb 3-1: Manufacturer: Generic
2026-09-01T18:17:45.935487+03:00 greenall kernel: usb 3-1: SerialNumber: 200901010000
2026-09-01T18:17:45.969834+03:00 greenall mtp-probe: checking bus 3, device 4: "/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1"
2026-09-01T18:17:45.970028+03:00 greenall mtp-probe: bus: 3, device: 4 was not an MTP device
2026-09-01T18:17:45.991186+03:00 greenall kernel: usb 3-1: Found UVC 1.00 device USB Camera (0bda:5840)
2026-09-01T18:17:46.007149+03:00 greenall kernel: usbcore: registered new interface driver uvcvideo
2026-09-01T18:17:46.009824+03:00 greenall mtp-probe: checking bus 3, device 4: "/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1"
2026-09-01T18:17:46.009892+03:00 greenall mtp-probe: bus: 3, device: 4 was not an MTP device
2026-09-01T18:17:46.030324+03:00 greenall wireplumber[1411]: <WpPortalPermissionStorePlugin:0x587d06638970> Failed to call Lookup: GDBus.Error:org.freedesktop.portal.Error.NotFound: No entry for camera
``` 

That frame is actually two stacked 256x192 halves:

- *top half* – processed 8-bit grayscale image (Y channel, U/V neutral);
- *bottom half* – raw 16-bit radiometric data packed as
  `pixel = (U << 8) | Y` (and `(V << 8) | Y1`).

This project decodes the radiometric half and runs the following pipeline
entirely in userspace:

```
YUYV 256x384
  -> decode bottom half to u16 raw thermal matrix
  -> 3x3 blur (denoise)
  -> detect sensor-wide offset jumps and compensate background
  -> adaptive background (IIR) + signed diff threshold
  -> 3x3 morphological closing
  -> connected components (8-connectivity) -> blobs
  -> merge nearby blobs
  -> nearest-neighbor / bbox-aware tracker
  -> track confirmation (2 hits)
  -> optional raw -> Celsius calibration
  -> JSON events on stdout (logs/diagnostics on stderr)

Warm-pixel counting (`--warm-delta`) is diagnostic metadata only; it is not OR'ed
into the motion mask.
```

The camera needs ~2–4 secs after stream start before it emits real frames
(black `00 80` frames before that); the tool handles this automatically
(`--warmup` + `--settle`).


## Build

```bash
cargo build --release
```

Run (requires read access to `/dev/video0`, i.e. membership in the `video` group):

```bash
sudo usermod -aG video $USER
```


## Usage

Real testing on Linux Mint box (with additional ffplay window view) when fingers were moved near the camera, then removed, so total 3 times back and forth in such way:
```bash
➜  simple-thermal-imager git:(main) ✗ date && cargo r --release -- --preview --stats
Sun Sep  6 11:48:54 PM EEST 2026
   Compiling simple-thermal-imager v0.3.0 (/home/vit/prjs/simple-thermal-imager)
    Finished `release` profile [optimized] target(s) in 7.39s
     Running `target/release/simple-thermal-imager --preview --stats`
live preview started with ffplay...
initializing background model (waiting up to 500 frames, about 20 seconds)...
frame    0: waiting for camera warmup...
frame   25: waiting for camera warmup...
frame   50: waiting for camera warmup...
background initialized from 10 frames
starting motion detection loop...
frame jump      3 | mean_abs   21.15 | mean_signed   -21.15 | >=100      0 | >=200      0
frame jump     60 | mean_abs   22.86 | mean_signed   -22.86 | >=100      0 | >=200      0
frame jump     85 | mean_abs   20.23 | mean_signed   -20.23 | >=100      0 | >=200      0
frame jump    114 | mean_abs  537.75 | mean_signed  -537.75 | >=100  49152 | >=200  49152
sensor-wide jump at frame 114, shifting background by -538
sensor settled; detection armed
frame    150 | active      0 (+     0/-     0) | streak 0 | ambient 19072 | warm      0 | raw 19000/19140 | blobs 0
event frame    165 | active    191 (+   191/-     0) | warm      0 | rawmax 19624 | blobs 1
{"timestamp":1788727751.7884555,"detections":[{"id":1,"x":1,"y":125,"width":5,"height":42,"area":146,"center_x":2,"center_y":147,"max_temp_raw":19604,"max_temp_c":24.74548}]}
{"timestamp":1788727751.8286526,"detections":[{"id":1,"x":1,"y":116,"width":9,"height":51,"area":302,"center_x":3,"center_y":143,"max_temp_raw":19632,"max_temp_c":24.833698}]}
{"timestamp":1788727751.8707054,"detections":[{"id":1,"x":1,"y":107,"width":13,"height":61,"area":528,"center_x":5,"center_y":139,"max_temp_raw":19652,"max_temp_c":24.896713}]}
{"timestamp":1788727751.9101188,"detections":[{"id":1,"x":1,"y":102,"width":15,"height":67,"area":673,"center_x":6,"center_y":139,"max_temp_raw":19652,"max_temp_c":24.896713}]}
{"timestamp":1788727751.9508727,"detections":[{"id":1,"x":1,"y":99,"width":17,"height":72,"area":822,"center_x":7,"center_y":140,"max_temp_raw":19656,"max_temp_c":24.909315}]}
event frame    170 | active   1094 (+  1094/-     0) | warm      0 | rawmax 19668 | blobs 1
{"timestamp":1788727751.9890962,"detections":[{"id":1,"x":1,"y":96,"width":20,"height":77,"area":1016,"center_x":8,"center_y":140,"max_temp_raw":19668,"max_temp_c":24.947123}]}
{"timestamp":1788727752.032092,"detections":[{"id":1,"x":1,"y":98,"width":19,"height":79,"area":976,"center_x":7,"center_y":143,"max_temp_raw":19668,"max_temp_c":24.947123}]}
{"timestamp":1788727752.0709574,"detections":[{"id":1,"x":1,"y":102,"width":18,"height":77,"area":946,"center_x":7,"center_y":146,"max_temp_raw":19668,"max_temp_c":24.947123}]}
{"timestamp":1788727752.1112995,"detections":[{"id":1,"x":1,"y":101,"width":20,"height":77,"area":1039,"center_x":8,"center_y":146,"max_temp_raw":19664,"max_temp_c":24.93452}]}
{"timestamp":1788727752.1504982,"detections":[{"id":1,"x":1,"y":95,"width":24,"height":81,"area":1274,"center_x":10,"center_y":142,"max_temp_raw":19664,"max_temp_c":24.93452}]}
frame    175 | active   1698 (+  1698/-     0) | streak 0 | ambient 19047 | warm      0 | raw 18960/19672 | blobs 1
event frame    175 | active   1698 (+  1698/-     0) | warm      0 | rawmax 19672 | blobs 1
{"timestamp":1788727752.191082,"detections":[{"id":1,"x":1,"y":87,"width":29,"height":84,"area":1616,"center_x":12,"center_y":136,"max_temp_raw":19668,"max_temp_c":24.947123}]}
{"timestamp":1788727752.2291548,"detections":[{"id":1,"x":1,"y":82,"width":30,"height":88,"area":1757,"center_x":12,"center_y":134,"max_temp_raw":19680,"max_temp_c":24.984932}]}
{"timestamp":1788727752.2694113,"detections":[{"id":1,"x":1,"y":81,"width":30,"height":88,"area":1786,"center_x":12,"center_y":133,"max_temp_raw":19680,"max_temp_c":24.984932}]}
{"timestamp":1788727752.3098612,"detections":[{"id":1,"x":1,"y":80,"width":30,"height":89,"area":1814,"center_x":12,"center_y":132,"max_temp_raw":19688,"max_temp_c":25.010138}]}
{"timestamp":1788727752.3520129,"detections":[{"id":1,"x":1,"y":79,"width":32,"height":89,"area":1868,"center_x":13,"center_y":132,"max_temp_raw":19684,"max_temp_c":24.997534}]}
frame jump    180 | mean_abs   24.00 | mean_signed   -19.89 | >=100    556 | >=200    131
event frame    180 | active   2148 (+  2148/-     0) | warm      0 | rawmax 19676 | blobs 1
{"timestamp":1788727752.391659,"detections":[{"id":1,"x":1,"y":77,"width":34,"height":90,"area":2062,"center_x":14,"center_y":130,"max_temp_raw":19676,"max_temp_c":24.972328}]}
{"timestamp":1788727752.429777,"detections":[{"id":1,"x":1,"y":70,"width":39,"height":96,"area":2376,"center_x":16,"center_y":128,"max_temp_raw":19672,"max_temp_c":24.959726}]}
{"timestamp":1788727752.472261,"detections":[{"id":1,"x":1,"y":61,"width":43,"height":103,"area":2703,"center_x":17,"center_y":124,"max_temp_raw":19672,"max_temp_c":24.959726}]}
{"timestamp":1788727752.511247,"detections":[{"id":1,"x":1,"y":56,"width":46,"height":107,"area":2960,"center_x":19,"center_y":121,"max_temp_raw":19672,"max_temp_c":24.959726}]}
{"timestamp":1788727752.55115,"detections":[{"id":1,"x":1,"y":52,"width":48,"height":109,"area":3096,"center_x":20,"center_y":119,"max_temp_raw":19668,"max_temp_c":24.947123}]}
event frame    185 | active   3395 (+  3395/-     0) | warm      0 | rawmax 19672 | blobs 1
{"timestamp":1788727752.5879943,"detections":[{"id":1,"x":1,"y":48,"width":50,"height":112,"area":3303,"center_x":20,"center_y":117,"max_temp_raw":19664,"max_temp_c":24.93452}]}
{"timestamp":1788727752.6304204,"detections":[{"id":1,"x":1,"y":46,"width":51,"height":114,"area":3361,"center_x":21,"center_y":115,"max_temp_raw":19660,"max_temp_c":24.921917}]}
{"timestamp":1788727752.6706698,"detections":[{"id":1,"x":1,"y":45,"width":51,"height":113,"area":3394,"center_x":21,"center_y":114,"max_temp_raw":19660,"max_temp_c":24.921917}]}
{"timestamp":1788727752.7125478,"detections":[{"id":1,"x":1,"y":44,"width":50,"height":113,"area":3358,"center_x":21,"center_y":113,"max_temp_raw":19656,"max_temp_c":24.909315}]}
{"timestamp":1788727752.7526944,"detections":[{"id":1,"x":1,"y":45,"width":48,"height":112,"area":3184,"center_x":20,"center_y":113,"max_temp_raw":19644,"max_temp_c":24.871508}]}
event frame    190 | active   3008 (+  3008/-     0) | warm      0 | rawmax 19640 | blobs 1
{"timestamp":1788727752.791441,"detections":[{"id":1,"x":1,"y":47,"width":44,"height":109,"area":2914,"center_x":18,"center_y":113,"max_temp_raw":19636,"max_temp_c":24.846302}]}
{"timestamp":1788727752.8313913,"detections":[{"id":1,"x":1,"y":51,"width":40,"height":105,"area":2582,"center_x":16,"center_y":115,"max_temp_raw":19628,"max_temp_c":24.821096}]}
{"timestamp":1788727752.8707132,"detections":[{"id":1,"x":1,"y":60,"width":33,"height":94,"area":2046,"center_x":13,"center_y":117,"max_temp_raw":19596,"max_temp_c":24.720274}]}
{"timestamp":1788727752.9100254,"detections":[{"id":1,"x":1,"y":72,"width":24,"height":83,"area":1299,"center_x":9,"center_y":120,"max_temp_raw":19564,"max_temp_c":24.619452}]}
{"timestamp":1788727752.952982,"detections":[{"id":1,"x":1,"y":95,"width":12,"height":60,"area":489,"center_x":5,"center_y":126,"max_temp_raw":19540,"max_temp_c":24.543835}]}
event frame    195 | active    323 (+   323/-     0) | warm      0 | rawmax 19524 | blobs 1
{"timestamp":1788727752.9903858,"detections":[{"id":2,"x":238,"y":117,"width":17,"height":22,"area":296,"center_x":247,"center_y":128,"max_temp_raw":19524,"max_temp_c":24.493425}]}
{"timestamp":1788727753.03181,"detections":[{"id":2,"x":231,"y":115,"width":24,"height":23,"area":455,"center_x":243,"center_y":126,"max_temp_raw":19520,"max_temp_c":24.480822}]}
{"timestamp":1788727753.0666308,"detections":[{"id":2,"x":223,"y":113,"width":32,"height":23,"area":628,"center_x":239,"center_y":124,"max_temp_raw":19528,"max_temp_c":24.506027}]}
{"timestamp":1788727753.1105392,"detections":[{"id":2,"x":216,"y":111,"width":39,"height":23,"area":799,"center_x":236,"center_y":122,"max_temp_raw":19516,"max_temp_c":24.46822}]}
{"timestamp":1788727753.150071,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
frame    200 | active    986 (+   986/-     0) | streak 0 | ambient 19008 | warm      0 | raw 18804/19508 | blobs 1
event frame    200 | active    986 (+   986/-     0) | warm      0 | rawmax 19508 | blobs 1
{"timestamp":1788727753.1881075,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.2278745,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.2700324,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.3105972,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.3500006,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
event frame    205 | active    986 (+   986/-     0) | warm      0 | rawmax 19508 | blobs 1
{"timestamp":1788727753.388889,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.4292364,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.4699447,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.509325,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.5526814,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
event frame    210 | active    986 (+   986/-     0) | warm      0 | rawmax 19508 | blobs 1
{"timestamp":1788727753.5947378,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
{"timestamp":1788727753.6306996,"detections":[{"id":2,"x":223,"y":113,"width":32,"height":23,"area":630,"center_x":239,"center_y":124,"max_temp_raw":19528,"max_temp_c":24.506027}]}
{"timestamp":1788727753.667487,"detections":[{"id":2,"x":216,"y":111,"width":39,"height":23,"area":772,"center_x":236,"center_y":122,"max_temp_raw":19516,"max_temp_c":24.46822}]}
{"timestamp":1788727753.7107015,"detections":[{"id":2,"x":209,"y":110,"width":46,"height":24,"area":963,"center_x":233,"center_y":121,"max_temp_raw":19508,"max_temp_c":24.443014}]}
frame jump    214 | mean_abs  234.82 | mean_signed  +234.51 | >=100  49080 | >=200  18804
sensor-wide jump at frame 214, shifting background by +235
{"timestamp":1788727753.7495065,"detections":[{"id":2,"x":145,"y":106,"width":110,"height":85,"area":4395,"center_x":216,"center_y":137,"max_temp_raw":19748,"max_temp_c":25.199179}]}
event frame    215 | active   4773 (+  4773/-     0) | warm      0 | rawmax 19760 | blobs 1
{"timestamp":1788727753.7904153,"detections":[{"id":2,"x":141,"y":106,"width":114,"height":85,"area":4673,"center_x":214,"center_y":138,"max_temp_raw":19760,"max_temp_c":25.236986}]}
{"timestamp":1788727753.8313754,"detections":[{"id":2,"x":137,"y":107,"width":118,"height":84,"area":4923,"center_x":213,"center_y":139,"max_temp_raw":19756,"max_temp_c":25.224384}]}
{"timestamp":1788727753.8689673,"detections":[{"id":2,"x":134,"y":107,"width":121,"height":84,"area":5159,"center_x":212,"center_y":140,"max_temp_raw":19748,"max_temp_c":25.199179}]}
{"timestamp":1788727753.9104326,"detections":[{"id":2,"x":132,"y":107,"width":123,"height":84,"area":5309,"center_x":211,"center_y":140,"max_temp_raw":19748,"max_temp_c":25.199179}]}
{"timestamp":1788727753.9493124,"detections":[{"id":2,"x":130,"y":108,"width":125,"height":83,"area":5371,"center_x":210,"center_y":141,"max_temp_raw":19740,"max_temp_c":25.173973}]}
event frame    220 | active   5540 (+  5540/-     0) | warm      0 | rawmax 19740 | blobs 1
{"timestamp":1788727753.9878953,"detections":[{"id":2,"x":129,"y":108,"width":126,"height":83,"area":5436,"center_x":210,"center_y":141,"max_temp_raw":19740,"max_temp_c":25.173973}]}
{"timestamp":1788727754.029728,"detections":[{"id":2,"x":128,"y":108,"width":127,"height":83,"area":5478,"center_x":209,"center_y":141,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788727754.0701637,"detections":[{"id":2,"x":127,"y":109,"width":128,"height":82,"area":5520,"center_x":209,"center_y":142,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788727754.1087673,"detections":[{"id":2,"x":127,"y":110,"width":128,"height":81,"area":5502,"center_x":208,"center_y":142,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788727754.1499944,"detections":[{"id":2,"x":127,"y":110,"width":128,"height":81,"area":5444,"center_x":209,"center_y":142,"max_temp_raw":19728,"max_temp_c":25.136164}]}
frame    225 | active   5465 (+  5465/-     0) | streak 0 | ambient 18982 | warm      0 | raw 18972/19724 | blobs 1
event frame    225 | active   5465 (+  5465/-     0) | warm      0 | rawmax 19724 | blobs 1
{"timestamp":1788727754.1912198,"detections":[{"id":2,"x":128,"y":110,"width":127,"height":81,"area":5370,"center_x":209,"center_y":142,"max_temp_raw":19724,"max_temp_c":25.123562}]}
{"timestamp":1788727754.2314124,"detections":[{"id":2,"x":129,"y":110,"width":126,"height":81,"area":5255,"center_x":210,"center_y":142,"max_temp_raw":19724,"max_temp_c":25.123562}]}
{"timestamp":1788727754.270628,"detections":[{"id":2,"x":132,"y":110,"width":123,"height":81,"area":5003,"center_x":211,"center_y":142,"max_temp_raw":19716,"max_temp_c":25.098356}]}
{"timestamp":1788727754.309223,"detections":[{"id":2,"x":138,"y":110,"width":117,"height":81,"area":4554,"center_x":213,"center_y":141,"max_temp_raw":19708,"max_temp_c":25.07315}]}
{"timestamp":1788727754.3495748,"detections":[{"id":2,"x":147,"y":109,"width":108,"height":82,"area":3915,"center_x":217,"center_y":140,"max_temp_raw":19704,"max_temp_c":25.060547}]}
event frame    230 | active   3233 (+  3233/-     0) | warm      0 | rawmax 19688 | blobs 1
{"timestamp":1788727754.3916562,"detections":[{"id":2,"x":157,"y":107,"width":98,"height":84,"area":3292,"center_x":221,"center_y":136,"max_temp_raw":19688,"max_temp_c":25.010138}]}
frame jump    231 | mean_abs   21.13 | mean_signed   -17.29 | >=100   2054 | >=200   1459
{"timestamp":1788727754.4297,"detections":[{"id":2,"x":169,"y":105,"width":86,"height":86,"area":2488,"center_x":223,"center_y":129,"max_temp_raw":19684,"max_temp_c":24.997534}]}
{"timestamp":1788727754.4712603,"detections":[{"id":2,"x":183,"y":103,"width":72,"height":31,"area":1428,"center_x":221,"center_y":114,"max_temp_raw":19652,"max_temp_c":24.896713}]}
{"timestamp":1788727754.512028,"detections":[{"id":2,"x":196,"y":102,"width":59,"height":27,"area":1061,"center_x":226,"center_y":112,"max_temp_raw":19628,"max_temp_c":24.821096}]}
{"timestamp":1788727754.5467906,"detections":[{"id":2,"x":207,"y":101,"width":48,"height":22,"area":747,"center_x":232,"center_y":109,"max_temp_raw":19628,"max_temp_c":24.821096}]}
event frame    235 | active    564 (+   501/-    63) | warm      0 | rawmax 19616 | blobs 1
{"timestamp":1788727754.5894527,"detections":[{"id":2,"x":218,"y":100,"width":37,"height":18,"area":519,"center_x":237,"center_y":107,"max_temp_raw":19616,"max_temp_c":24.783287}]}
{"timestamp":1788727754.630684,"detections":[{"id":2,"x":229,"y":100,"width":26,"height":16,"area":347,"center_x":242,"center_y":106,"max_temp_raw":19612,"max_temp_c":24.770685}]}
{"timestamp":1788727754.6717787,"detections":[{"id":2,"x":201,"y":99,"width":54,"height":25,"area":303,"center_x":238,"center_y":108,"max_temp_raw":19596,"max_temp_c":24.720274}]}
{"timestamp":1788727754.7090774,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":137,"center_x":227,"center_y":114,"max_temp_raw":19060,"max_temp_c":23.031507}]}
{"timestamp":1788727754.7453597,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":19012,"max_temp_c":22.880274}]}
event frame    240 | active    173 (+     0/-   173) | warm      0 | rawmax 19084 | blobs 1
{"timestamp":1788727754.7892385,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":19004,"max_temp_c":22.855068}]}
{"timestamp":1788727754.8295672,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":19000,"max_temp_c":22.842466}]}
{"timestamp":1788727754.8693578,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":19000,"max_temp_c":22.842466}]}
{"timestamp":1788727754.9096732,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":19000,"max_temp_c":22.842466}]}
{"timestamp":1788727754.9516866,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18992,"max_temp_c":22.81726}]}
event frame    245 | active    173 (+     0/-   173) | warm      0 | rawmax 19068 | blobs 1
{"timestamp":1788727754.990737,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18984,"max_temp_c":22.792055}]}
{"timestamp":1788727755.032572,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18976,"max_temp_c":22.76685}]}
frame jump    247 | mean_abs   21.44 | mean_signed   -21.44 | >=100      0 | >=200      0
{"timestamp":1788727755.0714953,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18968,"max_temp_c":22.741644}]}
{"timestamp":1788727755.1107602,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18960,"max_temp_c":22.716438}]}
{"timestamp":1788727755.1494813,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18960,"max_temp_c":22.716438}]}
frame    250 | active    173 (+     0/-   173) | streak 0 | ambient 18970 | warm      0 | raw 18872/19036 | blobs 1
event frame    250 | active    173 (+     0/-   173) | warm      0 | rawmax 19036 | blobs 1
{"timestamp":1788727755.1890488,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18960,"max_temp_c":22.716438}]}
{"timestamp":1788727755.2282553,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18960,"max_temp_c":22.716438}]}
{"timestamp":1788727755.271593,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18956,"max_temp_c":22.703836}]}
{"timestamp":1788727755.3110075,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18948,"max_temp_c":22.67863}]}
{"timestamp":1788727755.348306,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18944,"max_temp_c":22.666027}]}
event frame    255 | active    173 (+     0/-   173) | warm      0 | rawmax 19020 | blobs 1
{"timestamp":1788727755.3888738,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18944,"max_temp_c":22.666027}]}
{"timestamp":1788727755.4307904,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
{"timestamp":1788727755.471366,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
{"timestamp":1788727755.509193,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
{"timestamp":1788727755.548804,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
event frame    260 | active    173 (+     0/-   173) | warm      0 | rawmax 19008 | blobs 1
{"timestamp":1788727755.586184,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
{"timestamp":1788727755.630022,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18940,"max_temp_c":22.653425}]}
{"timestamp":1788727755.6706364,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18916,"max_temp_c":22.577808}]}
{"timestamp":1788727755.709809,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18916,"max_temp_c":22.577808}]}
{"timestamp":1788727755.7506144,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18908,"max_temp_c":22.552603}]}
event frame    265 | active    173 (+     0/-   173) | warm      0 | rawmax 18984 | blobs 1
{"timestamp":1788727755.7876396,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18908,"max_temp_c":22.552603}]}
{"timestamp":1788727755.8286567,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18904,"max_temp_c":22.54}]}
{"timestamp":1788727755.8698337,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18904,"max_temp_c":22.54}]}
{"timestamp":1788727755.911486,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18904,"max_temp_c":22.54}]}
{"timestamp":1788727755.950431,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18904,"max_temp_c":22.54}]}
event frame    270 | active    173 (+     0/-   173) | warm      0 | rawmax 18972 | blobs 1
{"timestamp":1788727755.9907057,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18900,"max_temp_c":22.527397}]}
{"timestamp":1788727756.030157,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18900,"max_temp_c":22.527397}]}
{"timestamp":1788727756.0697055,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18896,"max_temp_c":22.514795}]}
{"timestamp":1788727756.1089923,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.1492715,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
frame    275 | active    173 (+     0/-   173) | streak 0 | ambient 18945 | warm      0 | raw 18792/18960 | blobs 1
event frame    275 | active    173 (+     0/-   173) | warm      0 | rawmax 18960 | blobs 1
{"timestamp":1788727756.1890922,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.2301545,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.2711682,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.3132856,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.3518803,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
event frame    280 | active    173 (+     0/-   173) | warm      0 | rawmax 18956 | blobs 1
{"timestamp":1788727756.3895802,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18892,"max_temp_c":22.502192}]}
{"timestamp":1788727756.4288266,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18884,"max_temp_c":22.476986}]}
{"timestamp":1788727756.4679267,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18884,"max_temp_c":22.476986}]}
{"timestamp":1788727756.509759,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18884,"max_temp_c":22.476986}]}
{"timestamp":1788727756.5500054,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18880,"max_temp_c":22.464384}]}
event frame    285 | active    173 (+     0/-   173) | warm      0 | rawmax 18944 | blobs 1
{"timestamp":1788727756.5908356,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18872,"max_temp_c":22.439178}]}
{"timestamp":1788727756.630041,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18872,"max_temp_c":22.439178}]}
{"timestamp":1788727756.6725633,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18876,"max_temp_c":22.45178}]}
{"timestamp":1788727756.7107553,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18876,"max_temp_c":22.45178}]}
{"timestamp":1788727756.749551,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18872,"max_temp_c":22.439178}]}
event frame    290 | active    173 (+     0/-   173) | warm      0 | rawmax 18936 | blobs 1
{"timestamp":1788727756.789983,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18872,"max_temp_c":22.439178}]}
{"timestamp":1788727756.8287475,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18872,"max_temp_c":22.439178}]}
{"timestamp":1788727756.86988,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18868,"max_temp_c":22.426575}]}
{"timestamp":1788727756.9105732,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18864,"max_temp_c":22.413973}]}
{"timestamp":1788727756.9490385,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18860,"max_temp_c":22.40137}]}
event frame    295 | active    173 (+     0/-   173) | warm      0 | rawmax 18928 | blobs 1
{"timestamp":1788727756.988015,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18860,"max_temp_c":22.40137}]}
{"timestamp":1788727757.0281563,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18852,"max_temp_c":22.376163}]}
{"timestamp":1788727757.0685718,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18852,"max_temp_c":22.376163}]}
{"timestamp":1788727757.1086748,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18848,"max_temp_c":22.363562}]}
{"timestamp":1788727757.1499033,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18844,"max_temp_c":22.35096}]}
frame    300 | active    173 (+     0/-   173) | streak 0 | ambient 18915 | warm      0 | raw 18748/18908 | blobs 1
event frame    300 | active    173 (+     0/-   173) | warm      0 | rawmax 18908 | blobs 1
{"timestamp":1788727757.1906984,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18844,"max_temp_c":22.35096}]}
{"timestamp":1788727757.227903,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.268074,"detections":[{"id":2,"x":201,"y":111,"width":54,"height":13,"area":140,"center_x":228,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.3092172,"detections":[{"id":2,"x":202,"y":111,"width":53,"height":13,"area":139,"center_x":228,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.35212,"detections":[{"id":2,"x":203,"y":111,"width":52,"height":13,"area":138,"center_x":228,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
event frame    305 | active    170 (+     0/-   170) | warm      0 | rawmax 18900 | blobs 1
{"timestamp":1788727757.388745,"detections":[{"id":2,"x":204,"y":111,"width":51,"height":13,"area":137,"center_x":228,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.4271307,"detections":[{"id":2,"x":208,"y":111,"width":47,"height":13,"area":133,"center_x":229,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.4698474,"detections":[{"id":2,"x":208,"y":111,"width":47,"height":13,"area":133,"center_x":229,"center_y":114,"max_temp_raw":18840,"max_temp_c":22.338356}]}
{"timestamp":1788727757.510463,"detections":[{"id":2,"x":208,"y":111,"width":47,"height":13,"area":130,"center_x":229,"center_y":114,"max_temp_raw":18832,"max_temp_c":22.31315}]}
{"timestamp":1788727757.5495384,"detections":[{"id":2,"x":208,"y":111,"width":47,"height":12,"area":129,"center_x":230,"center_y":114,"max_temp_raw":18832,"max_temp_c":22.31315}]}
event frame    310 | active    160 (+     0/-   160) | warm      0 | rawmax 18896 | blobs 1
{"timestamp":1788727757.5877757,"detections":[{"id":2,"x":209,"y":111,"width":46,"height":9,"area":125,"center_x":230,"center_y":114,"max_temp_raw":18832,"max_temp_c":22.31315}]}
{"timestamp":1788727757.6301427,"detections":[{"id":2,"x":209,"y":111,"width":46,"height":9,"area":123,"center_x":231,"center_y":114,"max_temp_raw":18832,"max_temp_c":22.31315}]}
{"timestamp":1788727757.6692922,"detections":[{"id":2,"x":209,"y":111,"width":46,"height":9,"area":122,"center_x":231,"center_y":114,"max_temp_raw":18832,"max_temp_c":22.31315}]}
{"timestamp":1788727757.7092319,"detections":[{"id":2,"x":209,"y":111,"width":46,"height":9,"area":120,"center_x":231,"center_y":114,"max_temp_raw":18828,"max_temp_c":22.300549}]}
{"timestamp":1788727757.7492478,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":114,"center_x":232,"center_y":114,"max_temp_raw":18828,"max_temp_c":22.300549}]}
event frame    315 | active    149 (+     0/-   149) | warm      0 | rawmax 18892 | blobs 1
{"timestamp":1788727757.789137,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":113,"center_x":232,"center_y":114,"max_temp_raw":18828,"max_temp_c":22.300549}]}
{"timestamp":1788727757.8282247,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":112,"center_x":232,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
{"timestamp":1788727757.8679225,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":111,"center_x":233,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
{"timestamp":1788727757.9094195,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":109,"center_x":233,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
{"timestamp":1788727757.9508,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":108,"center_x":233,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
event frame    320 | active    132 (+     0/-   132) | warm      0 | rawmax 18884 | blobs 1
{"timestamp":1788727757.989184,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":108,"center_x":233,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
{"timestamp":1788727758.0287333,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":107,"center_x":233,"center_y":114,"max_temp_raw":18820,"max_temp_c":22.275343}]}
{"timestamp":1788727758.0688329,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":106,"center_x":233,"center_y":114,"max_temp_raw":18812,"max_temp_c":22.250137}]}
{"timestamp":1788727758.108732,"detections":[{"id":2,"x":210,"y":111,"width":45,"height":8,"area":100,"center_x":234,"center_y":114,"max_temp_raw":18812,"max_temp_c":22.250137}]}
frame    325 | active    104 (+     0/-   104) | streak 0 | ambient 18872 | warm      0 | raw 18716/18876 | blobs 0
event frame    325 | active    104 (+     0/-   104) | warm      0 | rawmax 18876 | blobs 0
event frame    330 | active     25 (+     0/-    25) | warm      0 | rawmax 18884 | blobs 0
event frame    335 | active     13 (+     0/-    13) | warm      0 | rawmax 18880 | blobs 0
event frame    340 | active      9 (+     0/-     9) | warm      0 | rawmax 18872 | blobs 0
event frame    345 | active      3 (+     0/-     3) | warm      0 | rawmax 18872 | blobs 0
frame    350 | active      0 (+     0/-     0) | streak 0 | ambient 18847 | warm      0 | raw 18696/18872 | blobs 0
frame jump    357 | mean_abs  262.02 | mean_signed  +262.02 | >=100  49152 | >=200  49152
sensor-wide jump at frame 357, shifting background by +262
frame    375 | active      0 (+     0/-     0) | streak 0 | ambient 18860 | warm      0 | raw 18976/19156 | blobs 0
frame    400 | active      0 (+     0/-     0) | streak 0 | ambient 18885 | warm      0 | raw 18972/19128 | blobs 0
frame    425 | active      0 (+     0/-     0) | streak 0 | ambient 18908 | warm      0 | raw 18940/19116 | blobs 0
frame    450 | active      0 (+     0/-     0) | streak 0 | ambient 18909 | warm      0 | raw 18956/19116 | blobs 0
frame    475 | active      0 (+     0/-     0) | streak 0 | ambient 18909 | warm      0 | raw 18944/19104 | blobs 0
frame    500 | active      0 (+     0/-     0) | streak 0 | ambient 18909 | warm      0 | raw 18932/19140 | blobs 0
frame    525 | active      0 (+     0/-     0) | streak 0 | ambient 18932 | warm      0 | raw 18972/19152 | blobs 0
frame    550 | active      0 (+     0/-     0) | streak 0 | ambient 18933 | warm      0 | raw 18968/19140 | blobs 0
^C
➜  simple-thermal-imager git:(main) ✗
```

Help usage information:
```
➜  simple-thermal-imager git:(main) ✗ ./target/release/simple-thermal-imager --help
Usage: simple-thermal-imager [--device <device>] [--roi <roi>] [--min-area <min-area>]
  [--threshold <threshold>] [--diagnose] [--half <half>] [--packing <packing>]
  [--warmup <warmup>] [--settle <settle>] [--track-dist <track-dist>]
  [--track-miss <track-miss>] [--warm-delta <warm-delta>] [--cal0 <cal0>] [--cal1 <cal1>]
  [--no-temp] [--png-dir <png-dir>] [--png-every <png-every>] [--preview] [--stats]

Thermal motion detector / blob tracker

Options:
  --device          video device path (default: "/dev/video0") or index, e.g.
                    /dev/video0 or 0
  --roi             region of interest: x, y, w, h
  --min-area        minimum blob area in pixels (default: 100)
  --threshold       raw-value difference threshold for movement (default: 300)
  --diagnose        run diagnostics and save PNGs
  --half            thermal half: top or bottom (default: "bottom")
  --packing         packing: high_uv or high_y (default: "high_uv")
  --warmup          max frames to wait for camera warmup (default: 500)
  --settle          frames to let the sensor settle after warmup before arming
                    (default: 125)
  --track-dist      max distance (px) to associate a detection with an existing
                    track (default: "60.0")
  --track-miss      frames a track may be missing before it is dropped (default:
                    12)
  --warm-delta      report objects warmer than ambient by this raw delta (0 =
                    motion only; default: 800)
  --cal0            calibration point 0 as raw:celsius (approximate T7 values;
                    default: "19050:23.0")
  --cal1            calibration point 1 as raw:celsius (default: "22700:34.5")
  --no-temp         do not add max_temp_c to JSON output
  --png-dir         directory to save a visualization PNG for each emitted JSON
                    record
  --png-every       save only every Nth record's PNG (1 = every record; default:
                    1)
  --preview         open a live preview ffplay window with the annotated thermal
                    view
  --stats           print detection statistics to stderr
  --help, help      display usage information

➜  simple-thermal-imager git:(main) ✗
```

Pipe to another tool:

```bash
./target/release/simple-thermal-imager | \
  jq -c '.detections[] | {id, center_x, center_y, area, max_temp_c}'
```

Common options:

```bash
# pure motion mode (no presence, no temperatures)
... -- --warm-delta 0 --no-temp

# watch only a region (x, y, w, h)
... -- --roi 128,0,128,192

# tune sensitivity
... -- --threshold 300 --min-area 100

# real calibration (two reference objects measured with a contact thermometer)
... -- --cal0 19050:23.0 --cal1 22700:34.5

# tracking behaviour
... -- --track-dist 60 --track-miss 12
```

Collect training crops for a future classifier (see `examples/crop_dumper.rs`):

```bash
... -- --example crop_dumper -- --max-crops 200
```

Running for weak signals, motion-only, sensitive, with ffplay preview visibility and stats output:
```bash
... -- --preview --stats --threshold 300 --min-area 60
```

### Output format

```json
{"timestamp":1788125629.21,"detections":[
  {"id":10,"x":225,"y":111,"width":31,"height":19,"area":391,
   "center_x":240,"center_y":119,"max_temp_raw":19828,"max_temp_c":25.6}
]}
```

`id` is a stable track ID while the object lives.


## Debug

General status after connected camera:
```
greenall# ls -l /dev/video*
crw-rw----+ 1 root video 81, 0 Aug 29 22:21 /dev/video0
crw-rw----+ 1 root video 81, 1 Aug 29 22:21 /dev/video1
greenall# v4l2-ctl --list-devices
USB Camera: USB Camera (usb-0000:00:14.0-1):
	/dev/video0
	/dev/video1
	/dev/media0

greenall# v4l2-ctl -d /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture

	[0]: 'YUYV' (YUYV 4:2:2)
		Size: Discrete 256x192
			Interval: Discrete 0.040s (25.000 fps)
		Size: Discrete 256x384
			Interval: Discrete 0.040s (25.000 fps)
greenall# v4l2-ctl -d /dev/video1 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture
greenall#
```

The simplest raw visibility test:
```
ffplay /dev/video0
```

If something looks wrong, use the diagnostic mode:

```bash
./target/release/simple-thermal-imager --diagnose --warmup 1000
```

It waits out the black startup frames, picks the best frame, prints Y/U/V
statistics and saves four contrast-normalized PNGs (`diag_top_high_uv.png`,
`diag_top_high_y.png`, `diag_bottom_high_uv.png`, `diag_bottom_high_y.png`).

Interpretation for a healthy T7 camera:

- `bottom_high_uv` shows the thermal image and has a range of ~1000–3000
  with mean ~19000–21000 -> *this is the radiometric config* (`--half bottom --packing high_uv`);
- `top_high_*` show the processed visual image;
- `bottom_high_y` is byte-swapped garbage.

Also, both `--preview` and `--stats` CLI args are useful for debugging.


### Troubleshooting

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


### Sensor-wide thermal recalibration

The TOOLTOP T7 periodically produces a sensor-wide radiometric offset step, consistent
with an internal recalibration / correction event.

During this event, nearly all 256×192 thermal pixels can shift by roughly the same
raw-value offset in a single frame. This is not real scene motion.

`simple-thermal-imager` detects these frame-wide jumps and shifts the existing
background model by the measured global offset. It deliberately does **not** replace
the background with the current frame, because a real object may be visible during
the correction.

This avoids turning the correction into false motion while preserving real objects
that are already being tracked.

In repeated bench tests, strong events were often observed around frames 114, 214 and
357 after startup. Treat those frame numbers as observations only, **not** as a fixed
camera schedule.


## Known limitations

- The motion model is intentionally simple and optimized for a stationary camera.
  A warm object that remains nearly stationary for long enough can be partially
  learned into the background and may leave a temporary cold residual after it
  disappears. The current bounded foreground-recovery logic eventually clears it,
  but short-lived afterimages are still possible.
- A moving/flying camera will require ego-motion compensation / frame registration
  before background subtraction is reliable.
- The raw-to-Celsius conversion is currently an approximate two-point calibration,
  not a replacement for the camera vendor's full thermometry model.


## TODO

- [ ] generic source modes: `--mode stacked16|luma`, `--width/--height`,
      luma-appropriate thresholds (support analog/webcam sources)
- [ ] object classification: ONNX (YOLO-cls/MobileNet) via `tract` or
      `candle`, fed by `crop_dumper` datasets; add `class`/`confidence` to JSON
- [ ] tracking upgrades: velocity-aware matching (Kalman), enter/exit events,
      object counting per session
- [ ] calibration helper: interactive two-point calibration mode
      (`--calibrate`) that samples raw values from known references
- [ ] replace `sort_unstable` ambient estimate with a histogram percentile
      (cheaper on SBCs)
- [ ] explore `/dev/video1` UVC metadata for spot temperatures / shutter info
- [ ] per-track temperature history and simple analytics in the JSON stream
- [ ] config file support instead of long CLI lines
- [ ] cross-compilation (`Orange Pi Zero 2W`)
- [ ] to add `--quiet` CLI switch for silent machine-to-machine mode


## License

MIT license ([LICENSE](https://github.com/vitali2y/simple-thermal-imager/blob/main/LICENSE) or <http://opensource.org/licenses/MIT>)
