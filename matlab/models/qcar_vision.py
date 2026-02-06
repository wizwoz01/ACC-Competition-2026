import numpy as np
import cv2
from ultralytics import YOLO


class _Track:
    __slots__ = ("track_id", "bbox", "label", "score", "hits", "age", "misses")

    def __init__(self, track_id: int, bbox: np.ndarray, label: str, score: float):
        self.track_id = track_id
        self.bbox = bbox.astype(np.float32)  # [x1,y1,x2,y2]
        self.label = label
        self.score = float(score)
        self.hits = 1
        self.age = 1
        self.misses = 0


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    xA = max(a[0], b[0])
    yA = max(a[1], b[1])
    xB = min(a[2], b[2])
    yB = min(a[3], b[3])
    inter_w = max(0.0, xB - xA)
    inter_h = max(0.0, yB - yA)
    inter = inter_w * inter_h
    if inter <= 0:
        return 0.0
    area_a = max(0.0, (a[2] - a[0])) * max(0.0, (a[3] - a[1]))
    area_b = max(0.0, (b[2] - b[0])) * max(0.0, (b[3] - b[1]))
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


class _GreedyTracker:
    """
    Deterministic tracker:
      - IoU match within same class label
      - greedy assignment
      - track removal after max_misses
    """
    def __init__(self, iou_thresh=0.35, max_misses=10):
        self.iou_thresh = float(iou_thresh)
        self.max_misses = int(max_misses)
        self._next_id = 1
        self.tracks = []

    def update(self, dets):
        """
        dets: list of (bbox, label, score)
        bbox: np.ndarray [x1,y1,x2,y2]
        """
        # Age all tracks
        for t in self.tracks:
            t.age += 1
            t.misses += 1

        assigned = set()

        # For each detection, match to best IoU track of same label
        for bbox, label, score in dets:
            best_iou = 0.0
            best_idx = -1
            for i, t in enumerate(self.tracks):
                if t.label != label:
                    continue
                iou = _iou(t.bbox, bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_idx = i

            if best_idx >= 0 and best_iou >= self.iou_thresh:
                t = self.tracks[best_idx]
                # Update track
                t.bbox = bbox.astype(np.float32)
                t.score = float(score)
                t.hits += 1
                t.misses = 0
                assigned.add(best_idx)
            else:
                # New track
                self.tracks.append(_Track(self._next_id, bbox, label, score))
                self._next_id += 1

        # Remove dead tracks
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        return self.tracks


class QCarVision:
    """
    Camera+depth perception for:
      - Traffic light detection + color classification
      - Stop sign detection + depth fusion distance
      - Temporal stability using tracking
    """

    def __init__(self):
        # COCO pretrained includes 'traffic light' and 'stop sign'
        self.model = YOLO("yolov8n.pt")

        self.conf_thresh = 0.35
        self.tl_label = "traffic light"
        self.stop_label = "stop sign"

        self.tracker = _GreedyTracker(iou_thresh=0.35, max_misses=8)

        # Decision stability (in addition to tracker)
        self._red_count = 0
        self._green_count = 0
        self._yellow_count = 0
        self._unknown_count = 0

        self._stop_count = 0

        # Stop decision thresholds
        self.stop_sign_trigger_m = 1.2  # meters
        self.stop_sign_confirm_frames = 3

        self.tl_confirm_frames = 3  # consecutive frames required to commit

    def step(self, rgb, depth):
        rgb = np.array(rgb, dtype=np.uint8)
        depth = np.array(depth, dtype=np.float32)

        # Normalize depth scale if it's in millimeters
        if np.nanmax(depth) > 50.0:
            depth = depth * 0.001  # mm -> m

        # Run detector
        dets = self._detect(rgb)

        # Update tracker
        tracks = self.tracker.update(dets)

        # Extract best tracks for traffic light and stop sign
        tl_track = self._best_track(tracks, self.tl_label)
        stop_track = self._best_track(tracks, self.stop_label)

        # Traffic light state
        tl_state = 0
        if tl_track is not None:
            tl_state = self._traffic_light_color(rgb, tl_track.bbox)
        tl_state = self._debounce_tl(tl_state)

        # Stop sign distance & stop decision
        stop_dist = float("inf")
        stop_required = False
        if stop_track is not None:
            stop_dist = self._bbox_depth_median(depth, stop_track.bbox)
            if stop_dist < self.stop_sign_trigger_m:
                self._stop_count += 1
            else:
                self._stop_count = 0
            stop_required = (self._stop_count >= self.stop_sign_confirm_frames)
        else:
            self._stop_count = 0

        # If traffic light is red, require stop
        if tl_state == 1:
            stop_required = True

        return int(tl_state), float(stop_dist), bool(stop_required)

    def _detect(self, rgb):
        # Ultralytics expects RGB image
        results = self.model.predict(source=rgb, verbose=False, conf=self.conf_thresh)
        r = results[0]

        dets = []
        if r.boxes is None or len(r.boxes) == 0:
            return dets

        names = r.names  # class id -> label
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        clss = r.boxes.cls.cpu().numpy().astype(int)

        for box, conf, cls_id in zip(boxes, confs, clss):
            label = str(names[int(cls_id)])
            if label not in (self.tl_label, self.stop_label):
                continue
            dets.append((box.astype(np.float32), label, float(conf)))

        return dets

    def _best_track(self, tracks, label):
        candidates = [t for t in tracks if t.label == label and t.hits >= 2]
        if not candidates:
            return None
        # Prefer most hits, then highest score
        candidates.sort(key=lambda t: (t.hits, t.score), reverse=True)
        return candidates[0]

    def _bbox_depth_median(self, depth_m, bbox):
        x1, y1, x2, y2 = bbox
        h, w = depth_m.shape[:2]
        x1 = int(max(0, min(w - 1, np.floor(x1))))
        x2 = int(max(0, min(w, np.ceil(x2))))
        y1 = int(max(0, min(h - 1, np.floor(y1))))
        y2 = int(max(0, min(h, np.ceil(y2))))

        roi = depth_m[y1:y2, x1:x2].astype(np.float32)
        if roi.size == 0:
            return float("inf")

        # Match Quanser validity window: 0.05 m < depth < 2 m
        valid = roi[(roi > 0.05) & (roi < 2.0)]
        if valid.size == 0:
            return float("inf")
        return float(np.median(valid))

    def _traffic_light_color(self, rgb, bbox):
        """
        Returns:
          1 red, 2 yellow, 3 green, 0 unknown
        """
        x1, y1, x2, y2 = bbox
        h, w = rgb.shape[:2]
        x1 = int(max(0, min(w - 1, np.floor(x1))))
        x2 = int(max(0, min(w, np.ceil(x2))))
        y1 = int(max(0, min(h - 1, np.floor(y1))))
        y2 = int(max(0, min(h, np.ceil(y2))))

        roi = rgb[y1:y2, x1:x2, :]
        if roi.size == 0:
            return 0

        # Convert ROI to HSV for robust color segmentation
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        # Masks for red (two hue ranges), yellow, green
        red1 = cv2.inRange(hsv, (0, 90, 90), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 90, 90), (179, 255, 255))
        red = cv2.bitwise_or(red1, red2)

        yellow = cv2.inRange(hsv, (18, 90, 90), (35, 255, 255))
        green = cv2.inRange(hsv, (40, 90, 90), (85, 255, 255))

        # Focus on bright pixels to reduce background
        v = hsv[:, :, 2]
        bright = (v > 140).astype(np.uint8) * 255

        red = cv2.bitwise_and(red, bright)
        yellow = cv2.bitwise_and(yellow, bright)
        green = cv2.bitwise_and(green, bright)

        # Count pixels as evidence
        r_cnt = int(cv2.countNonZero(red))
        y_cnt = int(cv2.countNonZero(yellow))
        g_cnt = int(cv2.countNonZero(green))

        # Require a minimum evidence threshold relative to ROI area
        area = roi.shape[0] * roi.shape[1]
        if area <= 0:
            return 0
        min_evidence = max(12, int(0.002 * area))  # deterministic threshold

        if r_cnt < min_evidence and y_cnt < min_evidence and g_cnt < min_evidence:
            return 0

        # Choose dominant
        if r_cnt >= y_cnt and r_cnt >= g_cnt:
            return 1
        if y_cnt >= r_cnt and y_cnt >= g_cnt:
            return 2
        return 3

    def _debounce_tl(self, state):
        # state: 0 unknown, 1 red, 2 yellow, 3 green
        if state == 1:
            self._red_count += 1
            self._green_count = self._yellow_count = self._unknown_count = 0
        elif state == 3:
            self._green_count += 1
            self._red_count = self._yellow_count = self._unknown_count = 0
        elif state == 2:
            self._yellow_count += 1
            self._red_count = self._green_count = self._unknown_count = 0
        else:
            self._unknown_count += 1
            self._red_count = self._green_count = self._yellow_count = 0

        if self._red_count >= self.tl_confirm_frames:
            return 1
        if self._green_count >= self.tl_confirm_frames:
            return 3
        if self._yellow_count >= self.tl_confirm_frames:
            return 2

        return 0