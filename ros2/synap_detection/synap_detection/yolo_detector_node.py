#!/usr/bin/env python3
"""YOLO multi-target detector for the SynapLine tracker-lock workflow.

Subscribes the Unity gimbal camera (sensor_msgs/CompressedImage), runs YOLOv8
with ByteTrack for persistent IDs, and publishes the targets three ways:

  /detection/targets  vision_msgs/Detection2DArray   standard interface
  /detection/image    sensor_msgs/Image (rgb8)        annotated frame (numbered,
                                                       coloured boxes) for the GCS
                                                       LIVE FEED panel
  /detection/boxes    std_msgs/Float32MultiArray      [N, (cx,cy,w,h)*N] in slot
                                                       order, for the joystick lock

The MATLAB GCS has no vision_msgs, so it consumes the image + Float32MultiArray
(both natively supported) instead of Detection2DArray. Pressing L1/R1/L2/R2 in
the GCS publishes slot k's box to /tracker/roi (sensor_msgs/RegionOfInterest),
which seeds hybrid_tracker_vpi -> /tracker/detection -> pn_guidance -> intercept.

Stable numbering: detections are sorted by persistent track id so slot "1" keeps
referring to the same physical object across frames -- essential for the L1..R2
binding. Slots are capped at max_targets (4 -> L1/R1/L2/R2).

vision_msgs is Humble (class_id is a string). Fields match the tracker's own
usage in tracker-vpi/src/{main,guidance_node}.cpp.

NOTE on classes: stock yolov8*.pt is COCO -- person/car/truck/bus etc., NOT
"tank"/"building". Point the `weights` param at custom weights (Unity synthetic
data) to add those; the rest of the pipeline is weights-agnostic.
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO

# Target slot -> joystick button + overlay colour (BGR, matches the GCS theme:
# cyan / amber / green / magenta).
SLOT_BTN   = ['L1', 'R1', 'L2', 'R2']
SLOT_COLOR = [(255, 255, 0), (46, 176, 255), (154, 210, 67), (224, 96, 224)]


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/synapsim/gimbal')
        self.declare_parameter('output_topic', '/detection/targets')
        self.declare_parameter('annotated_topic', '/detection/image')
        self.declare_parameter('boxes_topic', '/detection/boxes')
        self.declare_parameter('weights', 'yolov8n.pt')
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('imgsz', 512)
        self.declare_parameter('device', '')        # '' -> auto (cuda if available)
        self.declare_parameter('max_targets', 4)    # L1/R1/L2/R2 -> 4 slots
        self.declare_parameter('classes', [])       # COCO class ids to keep; [] = all
        self.declare_parameter('tracker', 'bytetrack.yaml')

        g = self.get_parameter
        self.image_topic = g('image_topic').value
        self.conf  = float(g('conf').value)
        self.iou   = float(g('iou').value)
        self.imgsz = int(g('imgsz').value)
        self.device = g('device').value or None
        self.max_targets = int(g('max_targets').value)
        cls = list(g('classes').value)
        self.classes = [int(c) for c in cls] if cls else None
        self.tracker_cfg = g('tracker').value

        self.get_logger().info(f'Loading YOLO weights: {g("weights").value}')
        self.model = YOLO(g('weights').value)

        self.pub_det = self.create_publisher(Detection2DArray, g('output_topic').value, 10)
        self.pub_img = self.create_publisher(Image, g('annotated_topic').value, 10)
        self.pub_box = self.create_publisher(Float32MultiArray, g('boxes_topic').value, 10)
        self.sub = self.create_subscription(
            CompressedImage, self.image_topic, self.on_image, 10)
        self.get_logger().info(
            f'YOLO detector up: {self.image_topic} -> {g("output_topic").value} '
            f'(+ {g("annotated_topic").value}, {g("boxes_topic").value}; '
            f'max_targets={self.max_targets}, device={self.device or "auto"})')

    def on_image(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode CompressedImage',
                                   throttle_duration_sec=5.0)
            return

        res = self.model.track(
            frame, persist=True, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
            classes=self.classes, tracker=self.tracker_cfg, device=self.device,
            verbose=False)[0]

        items = []
        boxes = res.boxes
        if boxes is not None and boxes.id is not None:
            xywh = boxes.xywh.cpu().numpy()         # center x, y, w, h (pixels)
            ids  = boxes.id.cpu().numpy().astype(int)
            clss = boxes.cls.cpu().numpy().astype(int)
            conf = boxes.conf.cpu().numpy()
            names = res.names
            for i in range(len(ids)):
                items.append((int(ids[i]), xywh[i], float(conf[i]),
                              names.get(int(clss[i]), str(int(clss[i])))))
            items.sort(key=lambda t: t[0])          # stable numbering by track id
            items = items[:self.max_targets]

        det_arr = Detection2DArray()
        det_arr.header = msg.header
        annotated = frame.copy()
        flat = [float(len(items))]                  # [N, (cx,cy,w,h) * N] in slot order

        for k, (tid, (cx, cy, w, h), score, label) in enumerate(items):
            col = SLOT_COLOR[k] if k < len(SLOT_COLOR) else (200, 200, 200)
            btn = SLOT_BTN[k] if k < len(SLOT_BTN) else '--'
            x1, y1 = int(round(cx - w / 2)), int(round(cy - h / 2))
            x2, y2 = int(round(cx + w / 2)), int(round(cy + h / 2))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), col, 2)
            tag = f'{k + 1} {btn}  {label} {score:.2f}'
            cv2.rectangle(annotated, (x1, max(0, y1 - 16)), (x1 + 8 * len(tag), y1), col, -1)
            cv2.putText(annotated, tag, (x1 + 2, max(11, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
            flat += [float(cx), float(cy), float(w), float(h)]

            det = Detection2D()
            det.header = msg.header
            det.id = str(tid)
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = score
            det.results.append(hyp)
            det_arr.detections.append(det)

        self.pub_det.publish(det_arr)

        img = Image()
        img.header = msg.header
        img.height = int(annotated.shape[0])
        img.width = int(annotated.shape[1])
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = 3 * img.width
        img.data = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB).tobytes()
        self.pub_img.publish(img)

        fa = Float32MultiArray()
        fa.data = flat
        self.pub_box.publish(fa)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
