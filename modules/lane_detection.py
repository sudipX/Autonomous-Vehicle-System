import cv2
import numpy as np


CAMERA_INDEX = 0          # Change if using a different camera port
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

# Canny edge detection thresholds
CANNY_LOW    = 50
CANNY_HIGH   = 150

# Hough transform parameters
HOUGH_RHO          = 1
HOUGH_THETA        = np.pi / 180
HOUGH_THRESHOLD    = 50
HOUGH_MIN_LENGTH   = 50
HOUGH_MAX_GAP      = 150

# Lane slope thresholds (degrees)
RIGHT_SLOPE_MIN = -80
RIGHT_SLOPE_MAX = -30
LEFT_SLOPE_MIN  =  30
LEFT_SLOPE_MAX  =  80

# Decision thresholds
LANE_COUNT_THRESHOLD = 10

# Y-region of interest (pixels)
ROI_Y_MIN = 250
ROI_Y_MAX = 600



def compute_slope(x1: int, x2: int, y1: int, y2: int) -> float:
    """Return the angle (degrees) of a line segment."""
    if x2 - x1 == 0:
        return 90.0
    return float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))


def preprocess_frame(frame: np.ndarray) -> np.ndarray:

    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
    return edges


def apply_roi_mask(edges: np.ndarray) -> np.ndarray:

    mask   = np.zeros_like(edges)
    height = edges.shape[0]
    width  = edges.shape[1]

    # Trapezoidal ROI
    roi_vertices = np.array([[
        (0,           height),
        (width // 4,  height // 2),
        (3 * width // 4, height // 2),
        (width,       height),
    ]], dtype=np.int32)

    cv2.fillPoly(mask, roi_vertices, 255)
    return cv2.bitwise_and(edges, mask)


def detect_lines(edges: np.ndarray):

    return cv2.HoughLinesP(
        edges,
        HOUGH_RHO,
        HOUGH_THETA,
        HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LENGTH,
        maxLineGap=HOUGH_MAX_GAP,
    )


def classify_lines(lines, frame: np.ndarray):
   
    left_count  = 0
    right_count = 0
    annotated   = frame.copy()

    if lines is None:
        return left_count, right_count, annotated

    for line in lines:
        x1, y1, x2, y2 = line[0]

        # Only consider lines inside the vertical ROI band
        if not (ROI_Y_MIN < y1 < ROI_Y_MAX and ROI_Y_MIN < y2 < ROI_Y_MAX):
            continue

        slope = compute_slope(x1, x2, y1, y2)

        if RIGHT_SLOPE_MIN <= round(slope) <= RIGHT_SLOPE_MAX:
            right_count += 1
            left_count   = 0
            cv2.line(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)

        elif LEFT_SLOPE_MIN <= round(slope) <= LEFT_SLOPE_MAX:
            left_count  += 1
            right_count  = 0
            cv2.line(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)

    return left_count, right_count, annotated


def decide_direction(left_count: int, right_count: int,
                     state: dict) -> tuple[str, dict]:
   
    command = "none"

    if left_count >= LANE_COUNT_THRESHOLD and state["a"]:
        command = "left"
        state.update({"a": False, "b": True, "c": True})

    elif right_count >= LANE_COUNT_THRESHOLD and state["b"]:
        command = "right"
        state.update({"a": True, "b": False, "c": True})

    elif left_count < LANE_COUNT_THRESHOLD and right_count < LANE_COUNT_THRESHOLD and state["c"]:
        command = "straight"
        state.update({"a": True, "b": True, "c": False})

    return command, state



def run(serial_connection=None, show_preview: bool = True):
   
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera at index {CAMERA_INDEX}")

    # Initial state flags
    state = {"a": True, "b": True, "c": False}
    last_command = "none"

    print("[LaneDetection] Starting — press 'q' to quit.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[LaneDetection] Frame read failed. Exiting.")
            break

        edges           = preprocess_frame(frame)
        masked          = apply_roi_mask(edges)
        lines           = detect_lines(masked)
        left, right, annotated = classify_lines(lines, frame)
        command, state  = decide_direction(left, right, state)

        if command != "none" and command != last_command:
            print(f"[LaneDetection] Command: {command.upper()}")
            last_command = command

            if serial_connection is not None:
                try:
                    serial_connection.write(command.encode())
                except Exception as exc:
                    print(f"[LaneDetection] Serial write error: {exc}")

        if show_preview:
            cv2.putText(
                annotated,
                f"CMD: {command.upper()}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )
            cv2.imshow("Lane Detection — Annotated", annotated)
            cv2.imshow("Lane Detection — Edges",     masked)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[LaneDetection] Stopped.")


if __name__ == "__main__":
    run(serial_connection=None, show_preview=True)
