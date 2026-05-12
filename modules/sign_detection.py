import cv2
import time
from pathlib import Path


MODEL_PATH      = "models/yolov11_traffic_signs.pt"   # Path to fine-tuned weights
CAMERA_INDEX    = 0
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480
CONFIDENCE_MIN  = 0.40          # Minimum detection confidence
STOP_HOLD_TIME  = 2.0           # Seconds to hold stop before resuming

CLASS_ACTIONS = {
    "stop":           "STOP",
    "speed limit 40": "SPEED_40",
    "speed limit 70": "SPEED_70",
    # Extend as more classes are added to the dataset
}


class TrafficSignDetector:

    def __init__(self, model_path: str = MODEL_PATH):
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            print(f"[SignDetector] Model loaded from '{model_path}'")
        except ImportError:
            raise ImportError(
                "ultralytics is required: pip install ultralytics"
            )
        except Exception as exc:
            raise RuntimeError(f"[SignDetector] Failed to load model: {exc}")

        self._stop_triggered_at: float = 0.0
        self._current_speed_limit: int = 70   # default

    
    def detect(self, frame) -> list[dict]:

        results     = self.model(frame, verbose=False)
        detections  = []

        for result in results:
            for box in result.boxes:
                conf  = float(box.conf[0])
                if conf < CONFIDENCE_MIN:
                    continue

                cls_id = int(box.cls[0])
                label  = result.names[cls_id].lower()
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                action = CLASS_ACTIONS.get(label, "UNKNOWN")
                detections.append({
                    "label":      label,
                    "confidence": conf,
                    "box":        (x1, y1, x2, y2),
                    "action":     action,
                })

        return detections

    
    def process_action(self, action: str, serial_connection=None) -> None:

        now = time.time()

        if action == "STOP":
            if now - self._stop_triggered_at > STOP_HOLD_TIME:
                print("[SignDetector] STOP sign detected — halting vehicle.")
                self._stop_triggered_at = now
                self._send(serial_connection, "stop")

        elif action == "SPEED_40":
            print("[SignDetector] Speed limit 40 detected.")
            self._current_speed_limit = 40
            self._send(serial_connection, "speed_40")

        elif action == "SPEED_70":
            print("[SignDetector] Speed limit 70 detected.")
            self._current_speed_limit = 70
            self._send(serial_connection, "speed_70")


    @staticmethod
    def _send(serial_connection, message: str) -> None:
        if serial_connection is not None:
            try:
                serial_connection.write(message.encode())
            except Exception as exc:
                print(f"[SignDetector] Serial write error: {exc}")


    @staticmethod
    def annotate(frame, detections: list[dict]):
        """Draw bounding boxes and labels on the frame in-place."""
        for det in detections:
            x1, y1, x2, y2 = det["box"]
            label = f"{det['label']} ({det['confidence']:.2f})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, label,
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2,
            )
        return frame



def run(serial_connection=None, show_preview: bool = True):

    if not Path(MODEL_PATH).exists():
        print(
            f"[SignDetector] WARNING: model not found at '{MODEL_PATH}'.\n"
        )
        return

    detector = TrafficSignDetector()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera at index {CAMERA_INDEX}")

    print("[SignDetector] Starting — press 'q' to quit.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect(frame)

        for det in detections:
            detector.process_action(det["action"], serial_connection)

        if show_preview:
            annotated = detector.annotate(frame.copy(), detections)
            cv2.imshow("Traffic Sign Detector", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[SignDetector] Stopped.")


if __name__ == "__main__":
    run(serial_connection=None, show_preview=True)
