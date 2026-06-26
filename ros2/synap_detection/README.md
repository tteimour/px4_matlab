# synap_detection

YOLO multi-target detector for the SynapLine tracker-lock workflow.

```
Unity gimbal ── /synapsim/gimbal (CompressedImage)
                      │
                      ▼
              yolo_detector  (this package: YOLOv8 + ByteTrack)
                      │
                      ▼
              /detection/targets (vision_msgs/Detection2DArray, numbered)
                      │      shown as numbered boxes in the GCS LIVE FEED panel
                      ▼
   pilot presses L1/R1/L2/R2  ──►  GCS publishes the chosen box to
                                    /tracker/roi (sensor_msgs/RegionOfInterest)
                      │
                      ▼
              hybrid_tracker_vpi ─► /tracker/detection ─► pn_guidance ─► intercept
```

The detector only **finds and numbers** candidate targets. The high-quality
single-target lock (ego-shift, Kalman, re-acquire) stays in `hybrid_tracker_vpi`,
which it already does via the `/tracker/roi` seed — so this node just feeds that
seed when the pilot selects a box.

## Run

```bash
# in a colcon workspace that contains this package
colcon build --packages-select synap_detection
source install/setup.bash
ros2 run synap_detection yolo_detector
# or with params:
ros2 run synap_detection yolo_detector --ros-args \
  -p weights:=yolov8s.pt -p conf:=0.4 -p max_targets:=4 -p device:=cuda:0
```

## Topics
- **sub** `image_topic` (default `/synapsim/gimbal`) — `sensor_msgs/CompressedImage`
- **pub** `output_topic` (default `/detection/targets`) — `vision_msgs/Detection2DArray`
- **pub** `annotated_topic` (default `/detection/image`) — `sensor_msgs/Image` (rgb8),
  the frame with numbered/coloured boxes drawn (slot 1-4 = L1/R1/L2/R2). The
  MATLAB GCS displays this directly (it has no `vision_msgs`).
- **pub** `boxes_topic` (default `/detection/boxes`) — `std_msgs/Float32MultiArray`,
  `[N, (cx,cy,w,h)*N]` in slot order, for the joystick lock in the GCS.

Each `Detection2D` carries: `bbox.center.position.{x,y}` + `bbox.size_x/size_y`
(pixels), `id = str(track_id)` (stable numbering), and
`results[0].hypothesis.{class_id, score}` (label + confidence). The image +
Float32MultiArray are the MATLAB-friendly mirrors of the same data.

## Parameters
| param | default | note |
|-------|---------|------|
| `weights` | `yolov8n.pt` | any ultralytics `.pt`; COCO by default |
| `conf` / `iou` | 0.35 / 0.45 | detection thresholds |
| `imgsz` | 512 | matches the Unity gimbal (512×512) |
| `device` | auto | `cuda:0` / `cpu` |
| `max_targets` | 4 | L1/R1/L2/R2 → targets 1..4 |
| `classes` | all | COCO class-id filter, e.g. `[0,2,7]` person/car/truck |
| `tracker` | `bytetrack.yaml` | persistent-ID tracker config |

## Tuning for distant / small targets
If targets are only detected up close, the limit is small-object recall on the
512×512 gimbal with a COCO model. In rough order of impact:

1. **Bigger model** (biggest win, GPU handles it):
   `weights:=yolov8m.pt` or `yolov8x.pt` (downloaded on first use).
2. **Larger inference size** — finer detection grid, upsamples the 512 input:
   `imgsz:=1280` (launch default is 960).
3. **Lower confidence gate** — keeps weak distant detections:
   `conf:=0.2` (launch default 0.25).

```bash
ros2 run synap_detection yolo_detector --ros-args \
  -p weights:=yolov8x.pt -p imgsz:=1280 -p conf:=0.2 -p device:=cuda:0
```

If that is still not enough, the next levers are **tiled/sliced inference**
(split the frame into overlapping tiles, detect per tile, merge — best for
small objects on a fixed-resolution stream) and, the real domain fix,
**custom weights trained on Unity synthetic labelled data** (also adds
tank/building). Raising the Unity camera resolution would give more pixels per
object but changes the intrinsics the tracker/guidance assume (512, fx=fy=394.2).

## Classes caveat
Stock `yolov8*.pt` is **COCO**: it detects `person`, `car`, `truck`, `bus`,
etc. — it does **not** have `tank` or `building`. For those, train custom
weights on Unity synthetic labelled data and point `weights` at them; nothing
else in the pipeline changes.
