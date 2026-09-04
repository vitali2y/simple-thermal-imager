# simple-thermal-imager

Thermal motion detector / blob tracker for TOOLTOP T7 (InfiRay-based) USB-C thermal imager:
radiometric frame decoding, motion detection, warm-object presence,
multi-object blob tracking and JSON event stream for downstream tools.


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
cargo build --release
```

Run (requires read access to `/dev/video0`, i.e. membership in the `video` group):

```bash
sudo usermod -aG video $USER
```


## Usage

Real testing on Linux Mint box (with additional ffplay window view) when fingers were moved near the camera, then removed, so total 3 times back and forth in such way:
```bash
➜  simple-thermal-imager git:(main) ✗ cargo run --release -- --preview --stats                                         
   Compiling simple-thermal-imager v0.1.0 (/home/vit/prjs/simple-thermal-imager)
    Finished `release` profile [optimized] target(s) in 7.49s
     Running `target/release/simple-thermal-imager --preview --stats`
Live preview started (ffplay).
initializing background model (waiting up to 500 frames, about 20 seconds)...
frame    0: waiting for camera warmup...
frame   25: waiting for camera warmup...
frame   50: waiting for camera warmup...
background initialized from 10 frames
starting motion detection loop...
sensor settled; detection armed
{"timestamp":1788505353.334667,"detections":[{"id":1,"x":115,"y":138,"width":53,"height":44,"area":1528,"center_x":139,"center_y":159,"max_temp_raw":19828,"max_temp_c":25.451233}]}
{"timestamp":1788505353.3744507,"detections":[{"id":1,"x":115,"y":137,"width":54,"height":46,"area":1647,"center_x":140,"center_y":159,"max_temp_raw":19824,"max_temp_c":25.43863}]}
{"timestamp":1788505353.412539,"detections":[{"id":1,"x":115,"y":142,"width":43,"height":35,"area":825,"center_x":135,"center_y":157,"max_temp_raw":19808,"max_temp_c":25.38822}]}
{"timestamp":1788505353.453247,"detections":[{"id":1,"x":117,"y":142,"width":37,"height":35,"area":661,"center_x":133,"center_y":156,"max_temp_raw":19800,"max_temp_c":25.363014}]}
event frame    130 | active      0 | warm    671 | rawmax 19796 | blobs 1
{"timestamp":1788505353.4929779,"detections":[{"id":1,"x":117,"y":142,"width":36,"height":33,"area":600,"center_x":133,"center_y":156,"max_temp_raw":19796,"max_temp_c":25.35041}]}
{"timestamp":1788505353.5336435,"detections":[{"id":1,"x":116,"y":142,"width":38,"height":30,"area":641,"center_x":134,"center_y":156,"max_temp_raw":19796,"max_temp_c":25.35041}]}
{"timestamp":1788505353.574214,"detections":[{"id":1,"x":116,"y":142,"width":38,"height":28,"area":498,"center_x":132,"center_y":154,"max_temp_raw":19796,"max_temp_c":25.35041}]}
{"timestamp":1788505353.6142597,"detections":[{"id":1,"x":118,"y":143,"width":29,"height":16,"area":222,"center_x":131,"center_y":150,"max_temp_raw":19788,"max_temp_c":25.325205}]}
event frame    135 | active      0 | warm     24 | rawmax 19780 | blobs 0
event frame    145 | active    329 | warm      0 | rawmax 19756 | blobs 0
{"timestamp":1788505354.1344686,"detections":[{"id":2,"x":120,"y":0,"width":46,"height":29,"area":184,"center_x":143,"center_y":13,"max_temp_raw":19660,"max_temp_c":24.921917}]}
frame    150 | active      0 | streak 0 | ambient 18972 | warm      0 | raw 18928/19748 | blobs 0
event frame    155 | active   1276 | warm      0 | rawmax 19744 | blobs 0
{"timestamp":1788505354.5344434,"detections":[{"id":3,"x":118,"y":140,"width":35,"height":40,"area":922,"center_x":135,"center_y":159,"max_temp_raw":19044,"max_temp_c":22.981096},{"id":4,"x":114,"y":71,"width":17,"height":14,"area":157,"center_x":122,"center_y":77,"max_temp_raw":19016,"max_temp_c":22.892878}]}
{"timestamp":1788505354.5746214,"detections":[{"id":3,"x":120,"y":137,"width":50,"height":41,"area":1152,"center_x":144,"center_y":159,"max_temp_raw":19040,"max_temp_c":22.968493},{"id":4,"x":125,"y":64,"width":32,"height":49,"area":769,"center_x":138,"center_y":92,"max_temp_raw":19020,"max_temp_c":22.90548},{"id":2,"x":128,"y":23,"width":40,"height":35,"area":623,"center_x":148,"center_y":41,"max_temp_raw":19000,"max_temp_c":22.842466}]}
{"timestamp":1788505354.6142423,"detections":[{"id":2,"x":136,"y":22,"width":76,"height":88,"area":2490,"center_x":165,"center_y":62,"max_temp_raw":19004,"max_temp_c":22.855068},{"id":3,"x":132,"y":126,"width":46,"height":47,"area":1194,"center_x":155,"center_y":150,"max_temp_raw":19024,"max_temp_c":22.918081}]}
{"timestamp":1788505354.65305,"detections":[{"id":2,"x":146,"y":0,"width":110,"height":106,"area":3731,"center_x":195,"center_y":63,"max_temp_raw":19000,"max_temp_c":22.842466},{"id":3,"x":147,"y":122,"width":45,"height":49,"area":1233,"center_x":171,"center_y":146,"max_temp_raw":19020,"max_temp_c":22.90548}]}
event frame    160 | active   4833 | warm      0 | rawmax 19684 | blobs 3
{"timestamp":1788505354.694312,"detections":[{"id":3,"x":168,"y":101,"width":67,"height":68,"area":1935,"center_x":195,"center_y":135,"max_temp_raw":19008,"max_temp_c":22.867672},{"id":5,"x":226,"y":15,"width":30,"height":77,"area":907,"center_x":246,"center_y":57,"max_temp_raw":18980,"max_temp_c":22.779451},{"id":2,"x":171,"y":91,"width":16,"height":11,"area":115,"center_x":177,"center_y":95,"max_temp_raw":18996,"max_temp_c":22.829863}]}
{"timestamp":1788505354.7326953,"detections":[{"id":3,"x":175,"y":92,"width":81,"height":73,"area":3009,"center_x":216,"center_y":130,"max_temp_raw":19000,"max_temp_c":22.842466}]}
{"timestamp":1788505354.7688851,"detections":[{"id":5,"x":231,"y":96,"width":25,"height":35,"area":377,"center_x":247,"center_y":109,"max_temp_raw":18968,"max_temp_c":22.741644},{"id":3,"x":222,"y":154,"width":34,"height":21,"area":299,"center_x":243,"center_y":166,"max_temp_raw":19016,"max_temp_c":22.892878}]}
{"timestamp":1788505354.8132896,"detections":[{"id":3,"x":237,"y":157,"width":19,"height":17,"area":167,"center_x":248,"center_y":165,"max_temp_raw":18984,"max_temp_c":22.792055}]}
frame    175 | active      0 | streak 0 | ambient 18953 | warm      0 | raw 18904/19068 | blobs 0
frame    200 | active      0 | streak 0 | ambient 18924 | warm      0 | raw 18780/18904 | blobs 0
frame    225 | active      0 | streak 0 | ambient 18909 | warm      0 | raw 18916/19040 | blobs 0
frame    250 | active      0 | streak 0 | ambient 18903 | warm      0 | raw 18864/18968 | blobs 0
event frame    265 | active     27 | warm      0 | rawmax 19612 | blobs 0
frame    275 | active      0 | streak 0 | ambient 18878 | warm      0 | raw 18792/19608 | blobs 0
frame    300 | active      0 | streak 0 | ambient 18853 | warm      0 | raw 18756/19536 | blobs 0
frame    325 | active      0 | streak 0 | ambient 18828 | warm      0 | raw 18736/19448 | blobs 0
frame    350 | active      0 | streak 0 | ambient 18803 | warm      0 | raw 18716/19420 | blobs 0
{"timestamp":1788505362.573807,"detections":[{"id":6,"x":196,"y":8,"width":60,"height":182,"area":7567,"center_x":232,"center_y":100,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788505362.614778,"detections":[{"id":6,"x":181,"y":0,"width":75,"height":192,"area":9572,"center_x":226,"center_y":96,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788505362.6542428,"detections":[{"id":6,"x":179,"y":0,"width":77,"height":189,"area":9465,"center_x":226,"center_y":94,"max_temp_raw":19740,"max_temp_c":25.173973}]}
event frame    360 | active   3391 | warm   8327 | rawmax 19740 | blobs 1
{"timestamp":1788505362.695271,"detections":[{"id":6,"x":178,"y":0,"width":78,"height":184,"area":9640,"center_x":225,"center_y":92,"max_temp_raw":19740,"max_temp_c":25.173973}]}
{"timestamp":1788505362.7348287,"detections":[{"id":6,"x":176,"y":0,"width":80,"height":182,"area":10116,"center_x":224,"center_y":91,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505362.7711186,"detections":[{"id":6,"x":174,"y":0,"width":82,"height":180,"area":10040,"center_x":224,"center_y":91,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505362.8148558,"detections":[{"id":6,"x":171,"y":0,"width":85,"height":179,"area":10065,"center_x":224,"center_y":91,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505362.8536263,"detections":[{"id":6,"x":168,"y":0,"width":88,"height":179,"area":10461,"center_x":223,"center_y":91,"max_temp_raw":19752,"max_temp_c":25.21178}]}
event frame    365 | active   1470 | warm  10624 | rawmax 19752 | blobs 1
{"timestamp":1788505362.8948734,"detections":[{"id":6,"x":166,"y":0,"width":90,"height":179,"area":10867,"center_x":222,"center_y":91,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505362.9341586,"detections":[{"id":6,"x":170,"y":0,"width":86,"height":178,"area":10929,"center_x":222,"center_y":92,"max_temp_raw":19756,"max_temp_c":25.224384}]}
{"timestamp":1788505362.9744442,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":10828,"center_x":222,"center_y":92,"max_temp_raw":19768,"max_temp_c":25.262192}]}
{"timestamp":1788505363.0142305,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":11008,"center_x":222,"center_y":92,"max_temp_raw":19768,"max_temp_c":25.262192}]}
{"timestamp":1788505363.0547428,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":11113,"center_x":221,"center_y":91,"max_temp_raw":19768,"max_temp_c":25.262192}]}
event frame    370 | active     15 | warm  11268 | rawmax 19768 | blobs 1
{"timestamp":1788505363.0932562,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":11172,"center_x":221,"center_y":91,"max_temp_raw":19768,"max_temp_c":25.262192}]}
{"timestamp":1788505363.133713,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":177,"area":10895,"center_x":222,"center_y":91,"max_temp_raw":19760,"max_temp_c":25.236986}]}
{"timestamp":1788505363.174937,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":10952,"center_x":222,"center_y":91,"max_temp_raw":19760,"max_temp_c":25.236986}]}
{"timestamp":1788505363.2154844,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":10922,"center_x":222,"center_y":91,"max_temp_raw":19760,"max_temp_c":25.236986}]}
{"timestamp":1788505363.2570732,"detections":[{"id":6,"x":177,"y":0,"width":79,"height":178,"area":10864,"center_x":222,"center_y":91,"max_temp_raw":19760,"max_temp_c":25.236986}]}
frame    375 | active      0 | streak 0 | ambient 18816 | warm  10276 | raw 18968/19760 | blobs 1
event frame    375 | active      0 | warm  10276 | rawmax 19760 | blobs 1
{"timestamp":1788505363.2943182,"detections":[{"id":6,"x":178,"y":0,"width":78,"height":177,"area":10115,"center_x":223,"center_y":92,"max_temp_raw":19760,"max_temp_c":25.236986}]}
{"timestamp":1788505363.3350239,"detections":[{"id":6,"x":181,"y":0,"width":75,"height":189,"area":10262,"center_x":224,"center_y":98,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505363.3730068,"detections":[{"id":6,"x":189,"y":5,"width":67,"height":147,"area":5982,"center_x":229,"center_y":88,"max_temp_raw":19724,"max_temp_c":25.123562},{"id":7,"x":193,"y":145,"width":63,"height":47,"area":1975,"center_x":228,"center_y":174,"max_temp_raw":19752,"max_temp_c":25.21178}]}
{"timestamp":1788505363.412081,"detections":[{"id":6,"x":196,"y":52,"width":60,"height":106,"area":3655,"center_x":233,"center_y":115,"max_temp_raw":19720,"max_temp_c":25.110958},{"id":7,"x":218,"y":173,"width":38,"height":19,"area":375,"center_x":241,"center_y":185,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788505363.4488945,"detections":[{"id":7,"x":222,"y":131,"width":34,"height":61,"area":1107,"center_x":244,"center_y":169,"max_temp_raw":19708,"max_temp_c":25.07315}]}
event frame    380 | active      0 | warm     30 | rawmax 19668 | blobs 0
frame    400 | active      0 | streak 0 | ambient 18841 | warm      0 | raw 18944/19072 | blobs 0
frame    425 | active      0 | streak 0 | ambient 18866 | warm      0 | raw 18944/19060 | blobs 0
frame    450 | active      0 | streak 0 | ambient 18873 | warm      0 | raw 18936/19040 | blobs 0
frame    475 | active      0 | streak 0 | ambient 18873 | warm      0 | raw 18928/19036 | blobs 0
frame    500 | active   4447 | streak 0 | ambient 18873 | warm   1690 | raw 18924/19728 | blobs 3
event frame    500 | active   4447 | warm   1690 | rawmax 19728 | blobs 3
{"timestamp":1788505368.2898386,"detections":[{"id":8,"x":224,"y":97,"width":32,"height":51,"area":922,"center_x":243,"center_y":124,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":230,"y":63,"width":26,"height":28,"area":364,"center_x":243,"center_y":77,"max_temp_raw":19700,"max_temp_c":25.047945},{"id":10,"x":222,"y":164,"width":34,"height":28,"area":279,"center_x":243,"center_y":178,"max_temp_raw":19724,"max_temp_c":25.123562}]}
{"timestamp":1788505368.3297062,"detections":[{"id":8,"x":206,"y":46,"width":50,"height":146,"area":4063,"center_x":237,"center_y":111,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788505368.3718932,"detections":[{"id":8,"x":201,"y":36,"width":55,"height":156,"area":4007,"center_x":237,"center_y":108,"max_temp_raw":19744,"max_temp_c":25.186575}]}
{"timestamp":1788505368.4127464,"detections":[{"id":8,"x":196,"y":28,"width":60,"height":164,"area":4211,"center_x":235,"center_y":106,"max_temp_raw":19748,"max_temp_c":25.199179}]}
{"timestamp":1788505368.4508896,"detections":[{"id":8,"x":191,"y":23,"width":65,"height":169,"area":4827,"center_x":234,"center_y":107,"max_temp_raw":19768,"max_temp_c":25.262192}]}
event frame    505 | active   2244 | warm   5182 | rawmax 19768 | blobs 2
{"timestamp":1788505368.4908392,"detections":[{"id":8,"x":186,"y":28,"width":70,"height":164,"area":4840,"center_x":234,"center_y":118,"max_temp_raw":19768,"max_temp_c":25.262192},{"id":9,"x":185,"y":14,"width":71,"height":56,"area":931,"center_x":219,"center_y":38,"max_temp_raw":19708,"max_temp_c":25.07315}]}
{"timestamp":1788505368.5299523,"detections":[{"id":8,"x":181,"y":23,"width":75,"height":169,"area":5470,"center_x":232,"center_y":115,"max_temp_raw":19764,"max_temp_c":25.249588},{"id":9,"x":179,"y":6,"width":77,"height":56,"area":1092,"center_x":217,"center_y":29,"max_temp_raw":19700,"max_temp_c":25.047945}]}
{"timestamp":1788505368.5708523,"detections":[{"id":8,"x":176,"y":12,"width":80,"height":180,"area":5743,"center_x":231,"center_y":111,"max_temp_raw":19768,"max_temp_c":25.262192},{"id":9,"x":175,"y":0,"width":81,"height":49,"area":1005,"center_x":215,"center_y":19,"max_temp_raw":19696,"max_temp_c":25.035343}]}
{"timestamp":1788505368.6108162,"detections":[{"id":8,"x":176,"y":12,"width":80,"height":178,"area":5571,"center_x":231,"center_y":108,"max_temp_raw":19768,"max_temp_c":25.262192},{"id":9,"x":173,"y":0,"width":64,"height":37,"area":702,"center_x":204,"center_y":13,"max_temp_raw":19688,"max_temp_c":25.010138}]}
{"timestamp":1788505368.6494768,"detections":[{"id":8,"x":181,"y":12,"width":75,"height":175,"area":5249,"center_x":232,"center_y":105,"max_temp_raw":19768,"max_temp_c":25.262192},{"id":9,"x":176,"y":0,"width":46,"height":25,"area":320,"center_x":195,"center_y":7,"max_temp_raw":19684,"max_temp_c":24.997534}]}
event frame    510 | active      0 | warm   4978 | rawmax 19756 | blobs 1
{"timestamp":1788505368.6933658,"detections":[{"id":8,"x":183,"y":12,"width":73,"height":177,"area":4770,"center_x":233,"center_y":104,"max_temp_raw":19756,"max_temp_c":25.224384}]}
{"timestamp":1788505368.730109,"detections":[{"id":8,"x":183,"y":23,"width":73,"height":169,"area":4633,"center_x":233,"center_y":108,"max_temp_raw":19756,"max_temp_c":25.224384}]}
{"timestamp":1788505368.7709498,"detections":[{"id":8,"x":184,"y":36,"width":72,"height":156,"area":4166,"center_x":234,"center_y":110,"max_temp_raw":19748,"max_temp_c":25.199179}]}
{"timestamp":1788505368.8106403,"detections":[{"id":8,"x":186,"y":40,"width":70,"height":152,"area":4302,"center_x":234,"center_y":111,"max_temp_raw":19752,"max_temp_c":25.21178},{"id":9,"x":213,"y":13,"width":23,"height":21,"area":229,"center_x":224,"center_y":23,"max_temp_raw":19700,"max_temp_c":25.047945}]}
{"timestamp":1788505368.8490362,"detections":[{"id":8,"x":201,"y":42,"width":55,"height":150,"area":3935,"center_x":236,"center_y":110,"max_temp_raw":19744,"max_temp_c":25.186575},{"id":9,"x":213,"y":13,"width":24,"height":24,"area":280,"center_x":224,"center_y":24,"max_temp_raw":19700,"max_temp_c":25.047945}]}
event frame    515 | active      0 | warm   4649 | rawmax 19736 | blobs 2
{"timestamp":1788505368.8924596,"detections":[{"id":8,"x":202,"y":42,"width":54,"height":150,"area":3840,"center_x":235,"center_y":108,"max_temp_raw":19736,"max_temp_c":25.16137},{"id":9,"x":213,"y":9,"width":31,"height":34,"area":485,"center_x":227,"center_y":25,"max_temp_raw":19700,"max_temp_c":25.047945}]}
{"timestamp":1788505368.9308233,"detections":[{"id":8,"x":202,"y":47,"width":54,"height":94,"area":3053,"center_x":233,"center_y":95,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":213,"y":12,"width":31,"height":31,"area":517,"center_x":227,"center_y":25,"max_temp_raw":19700,"max_temp_c":25.047945},{"id":11,"x":222,"y":161,"width":34,"height":31,"area":451,"center_x":244,"center_y":181,"max_temp_raw":19732,"max_temp_c":25.148767}]}
{"timestamp":1788505368.9710271,"detections":[{"id":8,"x":203,"y":51,"width":53,"height":90,"area":2905,"center_x":233,"center_y":96,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":213,"y":12,"width":43,"height":31,"area":623,"center_x":231,"center_y":25,"max_temp_raw":19700,"max_temp_c":25.047945},{"id":11,"x":219,"y":161,"width":37,"height":31,"area":473,"center_x":243,"center_y":181,"max_temp_raw":19732,"max_temp_c":25.148767}]}
{"timestamp":1788505369.0129182,"detections":[{"id":8,"x":199,"y":71,"width":57,"height":78,"area":1990,"center_x":233,"center_y":102,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":214,"y":12,"width":42,"height":33,"area":852,"center_x":236,"center_y":27,"max_temp_raw":19700,"max_temp_c":25.047945},{"id":11,"x":203,"y":157,"width":53,"height":35,"area":848,"center_x":238,"center_y":179,"max_temp_raw":19756,"max_temp_c":25.224384},{"id":12,"x":182,"y":21,"width":27,"height":27,"area":314,"center_x":194,"center_y":33,"max_temp_raw":19696,"max_temp_c":25.035343}]}
{"timestamp":1788505369.0498354,"detections":[{"id":9,"x":194,"y":12,"width":62,"height":97,"area":2546,"center_x":222,"center_y":56,"max_temp_raw":19708,"max_temp_c":25.07315},{"id":11,"x":205,"y":157,"width":51,"height":35,"area":760,"center_x":239,"center_y":180,"max_temp_raw":19744,"max_temp_c":25.186575},{"id":8,"x":240,"y":110,"width":16,"height":33,"area":346,"center_x":248,"center_y":125,"max_temp_raw":19732,"max_temp_c":25.148767}]}
event frame    520 | active      0 | warm   3464 | rawmax 19732 | blobs 2
{"timestamp":1788505369.0903256,"detections":[{"id":9,"x":194,"y":26,"width":62,"height":83,"area":2393,"center_x":226,"center_y":67,"max_temp_raw":19716,"max_temp_c":25.098356},{"id":11,"x":207,"y":161,"width":49,"height":31,"area":673,"center_x":240,"center_y":181,"max_temp_raw":19732,"max_temp_c":25.148767}]}
{"timestamp":1788505369.1298828,"detections":[{"id":9,"x":197,"y":30,"width":59,"height":94,"area":2683,"center_x":229,"center_y":79,"max_temp_raw":19716,"max_temp_c":25.098356},{"id":11,"x":205,"y":160,"width":51,"height":32,"area":685,"center_x":239,"center_y":181,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788505369.17273,"detections":[{"id":9,"x":203,"y":45,"width":53,"height":83,"area":2537,"center_x":232,"center_y":89,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":207,"y":161,"width":49,"height":31,"area":597,"center_x":241,"center_y":182,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788505369.2111034,"detections":[{"id":9,"x":204,"y":52,"width":52,"height":76,"area":2395,"center_x":232,"center_y":92,"max_temp_raw":19724,"max_temp_c":25.123562},{"id":11,"x":213,"y":165,"width":43,"height":27,"area":492,"center_x":242,"center_y":183,"max_temp_raw":19728,"max_temp_c":25.136164}]}
{"timestamp":1788505369.2503958,"detections":[{"id":9,"x":204,"y":50,"width":52,"height":85,"area":2562,"center_x":232,"center_y":93,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":216,"y":165,"width":40,"height":27,"area":419,"center_x":243,"center_y":183,"max_temp_raw":19736,"max_temp_c":25.16137}]}
frame    525 | active      0 | streak 0 | ambient 18879 | warm   3045 | raw 18948/19736 | blobs 2
event frame    525 | active      0 | warm   3045 | rawmax 19736 | blobs 2
{"timestamp":1788505369.2906353,"detections":[{"id":9,"x":204,"y":52,"width":52,"height":86,"area":2500,"center_x":232,"center_y":96,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":222,"y":167,"width":34,"height":25,"area":354,"center_x":245,"center_y":183,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788505369.330031,"detections":[{"id":9,"x":207,"y":56,"width":49,"height":79,"area":2259,"center_x":233,"center_y":97,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":230,"y":170,"width":26,"height":22,"area":266,"center_x":247,"center_y":184,"max_temp_raw":19736,"max_temp_c":25.16137}]}
{"timestamp":1788505369.371119,"detections":[{"id":9,"x":209,"y":58,"width":47,"height":82,"area":2366,"center_x":235,"center_y":104,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":234,"y":170,"width":22,"height":22,"area":224,"center_x":248,"center_y":184,"max_temp_raw":19740,"max_temp_c":25.173973}]}
{"timestamp":1788505369.4104917,"detections":[{"id":8,"x":210,"y":65,"width":46,"height":80,"area":2264,"center_x":235,"center_y":110,"max_temp_raw":19724,"max_temp_c":25.123562},{"id":11,"x":234,"y":169,"width":22,"height":23,"area":232,"center_x":248,"center_y":184,"max_temp_raw":19740,"max_temp_c":25.173973}]}
{"timestamp":1788505369.4494395,"detections":[{"id":8,"x":212,"y":69,"width":44,"height":81,"area":2234,"center_x":236,"center_y":113,"max_temp_raw":19724,"max_temp_c":25.123562},{"id":11,"x":227,"y":169,"width":29,"height":23,"area":277,"center_x":247,"center_y":183,"max_temp_raw":19740,"max_temp_c":25.173973}]}
event frame    530 | active      0 | warm   2446 | rawmax 19740 | blobs 2
{"timestamp":1788505369.4905684,"detections":[{"id":8,"x":217,"y":77,"width":39,"height":77,"area":1915,"center_x":239,"center_y":120,"max_temp_raw":19724,"max_temp_c":25.123562},{"id":11,"x":224,"y":170,"width":32,"height":22,"area":303,"center_x":245,"center_y":184,"max_temp_raw":19740,"max_temp_c":25.173973}]}
{"timestamp":1788505369.5296931,"detections":[{"id":8,"x":221,"y":86,"width":35,"height":70,"area":1554,"center_x":240,"center_y":124,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":219,"y":170,"width":37,"height":22,"area":361,"center_x":243,"center_y":184,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":213,"y":79,"width":26,"height":25,"area":242,"center_x":225,"center_y":91,"max_temp_raw":19708,"max_temp_c":25.07315}]}
{"timestamp":1788505369.5705817,"detections":[{"id":8,"x":224,"y":86,"width":32,"height":70,"area":1447,"center_x":241,"center_y":123,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":213,"y":168,"width":43,"height":24,"area":493,"center_x":241,"center_y":183,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":214,"y":78,"width":29,"height":25,"area":295,"center_x":228,"center_y":90,"max_temp_raw":19712,"max_temp_c":25.085754}]}
{"timestamp":1788505369.610554,"detections":[{"id":8,"x":225,"y":87,"width":31,"height":66,"area":1332,"center_x":242,"center_y":123,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":213,"y":168,"width":43,"height":24,"area":494,"center_x":240,"center_y":183,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":9,"x":221,"y":78,"width":22,"height":24,"area":231,"center_x":230,"center_y":89,"max_temp_raw":19708,"max_temp_c":25.07315}]}
{"timestamp":1788505369.6497805,"detections":[{"id":8,"x":228,"y":95,"width":28,"height":59,"area":1047,"center_x":243,"center_y":128,"max_temp_raw":19728,"max_temp_c":25.136164},{"id":11,"x":225,"y":172,"width":31,"height":20,"area":291,"center_x":244,"center_y":185,"max_temp_raw":19720,"max_temp_c":25.110958},{"id":9,"x":221,"y":79,"width":29,"height":26,"area":229,"center_x":234,"center_y":93,"max_temp_raw":19712,"max_temp_c":25.085754}]}
event frame    535 | active      0 | warm   1171 | rawmax 19716 | blobs 3
{"timestamp":1788505369.6905644,"detections":[{"id":8,"x":230,"y":116,"width":26,"height":50,"area":679,"center_x":244,"center_y":143,"max_temp_raw":19716,"max_temp_c":25.098356},{"id":9,"x":228,"y":95,"width":28,"height":32,"area":260,"center_x":241,"center_y":111,"max_temp_raw":19704,"max_temp_c":25.060547},{"id":11,"x":232,"y":183,"width":24,"height":9,"area":149,"center_x":245,"center_y":187,"max_temp_raw":19712,"max_temp_c":25.085754}]}
{"timestamp":1788505369.729498,"detections":[{"id":8,"x":234,"y":114,"width":22,"height":61,"area":684,"center_x":246,"center_y":142,"max_temp_raw":19712,"max_temp_c":25.085754}]}
frame    550 | active      0 | streak 0 | ambient 18881 | warm      0 | raw 18940/19068 | blobs 0
frame    575 | active      0 | streak 0 | ambient 18881 | warm      0 | raw 18940/19048 | blobs 0
frame    600 | active      0 | streak 0 | ambient 18881 | warm      0 | raw 18928/19044 | blobs 0
frame    625 | active      0 | streak 0 | ambient 18881 | warm      0 | raw 18928/19044 | blobs 0
frame    650 | active      0 | streak 0 | ambient 18881 | warm      0 | raw 18924/19032 | blobs 0
^C
```

Help usage information:
```
➜  simple-thermal-imager git:(main) ✗ ./target/release/simple-thermal-imager --help
Usage: simple-thermal-imager [--device <device>] [--roi <roi>] [--min-area <min-area>]
 [--threshold <threshold>] [--diagnose] [--half <half>] [--packing <packing>] [--warmup <warmup>]
 [--settle <settle>] [--track-dist <track-dist>] [--track-miss <track-miss>] [--warm-delta <warm-delta>]
 [--cal0 <cal0>] [--cal1 <cal1>] [--no-temp] [--png-dir <png-dir>] [--png-every <png-every>]
 [--preview] [--stats]

Thermal motion detector / blob tracker

Options:
  --device          video device path (default: "/dev/video0") or index, e.g.
                    /dev/video0 or 0
  --roi             region of interest: x, y, w, h
  --min-area        minimum blob area in pixels (default: 100)
  --threshold       raw-value difference threshold for movement (default: 600)
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
... -- --threshold 600 --min-area 100

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
```
... -- --preview --stats --threshold 300 --min-area 60
```


## Debug

General status:
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
  with mean ~19000–21000 → **this is the radiometric config**
  (`--half bottom --packing high_uv`);
- `top_high_*` show the processed visual image;
- `bottom_high_y` is byte-swapped garbage.

Also, both `--preview` and `--stats` CLI args are useful for debugging.

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
