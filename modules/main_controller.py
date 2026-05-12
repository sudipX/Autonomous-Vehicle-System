import argparse
import sys
import time
import threading
import cv2

from serial_comm   import SerialComm, find_arduino_port, DEFAULT_PORT
from lane_detection import (
    preprocess_frame, apply_roi_mask, detect_lines,
    classify_lines, decide_direction, CAMERA_INDEX,
    FRAME_WIDTH, FRAME_HEIGHT,
)


SIGN_DETECTION_ENABLED = True    # Disable if model weights are not yet ready
SHOW_PREVIEW           = True
LOOP_DELAY             = 0.01    # Seconds between main loop iterations



class AutoTrackController:

    def __init__(self, serial_port: str, show_preview: bool = True):
        self.serial   = SerialComm(port=serial_port)
        self.preview  = show_preview
        self._running = False

        # Lane-detection state flags
        self._lane_state = {"a": True, "b": True, "c": False}
        self._last_cmd   = "none"

        # Sign detector (lazy load to avoid crash if ultralytics missing)
        self._sign_detector = None
        if SIGN_DETECTION_ENABLED:
            try:
                from sign_detection import TrafficSignDetector
                self._sign_detector = TrafficSignDetector()
                print("[Controller] Traffic sign detector loaded.")
            except Exception as exc:
                print(f"[Controller] Sign detector unavailable: {exc}")

        # Arduino feedback (obstacle alerts received on serial)
        self._obstacle_detected = False
        self._obstacle_lock     = threading.Lock()

    # ------------------------------------------------------------------
    def start(self) -> None:

        self.serial.connect()

        # Background thread to read Arduino feedback (obstacle alerts)
        self._running = True
        reader_thread = threading.Thread(target=self._read_arduino, daemon=True)
        reader_thread.start()

        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not cap.isOpened():
            print("[Controller] ERROR: Cannot open camera.")
            self.stop()
            return

        print("[Controller] AutoTrack running — press 'q' to quit.")

        try:
            self._loop(cap)
        except KeyboardInterrupt:
            print("\n[Controller] KeyboardInterrupt received.")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.stop()

    # ------------------------------------------------------------------
    def stop(self) -> None:

        self._running = False
        self.serial.send("stop")
        self.serial.disconnect()
        print("[Controller] AutoTrack stopped.")

    # ------------------------------------------------------------------
    def _loop(self, cap) -> None:
        while self._running:
            ret, frame = cap.read()
            if not ret:
                print("[Controller] Frame read failed.")
                break

            # Obstacle check (from Arduino ultrasonic feedback)
            with self._obstacle_lock:
                obstacle = self._obstacle_detected

            if obstacle:
                self._dispatch("stop")
                if self.preview:
                    self._show(frame, "OBSTACLE — STOPPED")
                time.sleep(LOOP_DELAY)
                continue

            # Traffic sign detection 
            sign_cmd = None
            if self._sign_detector is not None:
                detections = self._sign_detector.detect(frame)
                for det in detections:
                    action = det["action"]
                    if action == "STOP":
                        sign_cmd = "stop"
                    elif action == "SPEED_40":
                        sign_cmd = "speed_40"
                    elif action == "SPEED_70":
                        sign_cmd = "speed_70"

                    if self.preview:
                        self._sign_detector.annotate(frame, detections)

            if sign_cmd:
                self._dispatch(sign_cmd)

            # Lane detection 
            edges                        = preprocess_frame(frame)
            masked                       = apply_roi_mask(edges)
            lines                        = detect_lines(masked)
            left, right, annotated_frame = classify_lines(lines, frame)
            lane_cmd, self._lane_state   = decide_direction(left, right, self._lane_state)

            if lane_cmd != "none":
                self._dispatch(lane_cmd)

            # Preview 
            if self.preview:
                label = sign_cmd.upper() if sign_cmd else lane_cmd.upper()
                self._show(annotated_frame, label)
                cv2.imshow("Edges", masked)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(LOOP_DELAY)


    def _dispatch(self, command: str) -> None:
        if command != self._last_cmd:
            self.serial.send(command)
            self._last_cmd = command


    def _read_arduino(self) -> None:
        while self._running:
            line = self.serial.read_line()
            if line:
                print(f"[Arduino] {line}")
                with self._obstacle_lock:
                    self._obstacle_detected = "obstacle detected" in line.lower()
            time.sleep(0.05)

    @staticmethod
    def _show(frame, label: str) -> None:
        cv2.putText(
            frame, f"CMD: {label}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 255), 2,
        )
        cv2.imshow("AutoTrack", frame)



def parse_args():
    parser = argparse.ArgumentParser(description="AutoTrack — Self-Driving Car Controller")
    parser.add_argument(
        "port",
        type=str,
        default=None,
        help="Serial port for Arduino (e.g. /dev/ttyUSB0). Auto-detected if omitted.",
    )
    parser.add_argument(
        "no-preview",
        action="store_true",
        help="Disable OpenCV preview windows (useful for headless operation).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    port = args.port or find_arduino_port() or DEFAULT_PORT
    show = not args.no_preview

    controller = AutoTrackController(serial_port=port, show_preview=show)
    controller.start()
