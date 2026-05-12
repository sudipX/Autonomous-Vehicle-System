import time
import threading
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[SerialComm] WARNING: pyserial not installed. Run: pip install pyserial")


DEFAULT_PORT     = "/dev/ttyUSB0"   # Adjust: ttyUSB0 or ttyACM0 on RPi
DEFAULT_BAUDRATE = 9600
READ_TIMEOUT     = 1.0              # seconds
WRITE_DELAY      = 0.05            # seconds between consecutive writes


CMD_LEFT      = "left"
CMD_RIGHT     = "right"
CMD_STRAIGHT  = "straight"
CMD_STOP      = "stop"
CMD_SPEED_40  = "speed_40"
CMD_SPEED_70  = "speed_70"

VALID_COMMANDS = {CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_STOP, CMD_SPEED_40, CMD_SPEED_70}



class SerialComm:

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE):
        self.port      = port
        self.baudrate  = baudrate
        self._serial: Optional[serial.Serial] = None
        self._lock     = threading.Lock()
        self._last_send_time = 0.0

    
    def connect(self) -> bool:
 
        if not SERIAL_AVAILABLE:
            print("[SerialComm] pyserial missing — running in mock mode.")
            return False

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=READ_TIMEOUT,
            )
            time.sleep(2.0)   # Allow Arduino to reset after serial open
            print(f"[SerialComm] Connected to {self.port} @ {self.baudrate} baud.")
            return True
        except serial.SerialException as exc:
            print(f"[SerialComm] Connection failed: {exc}")
            self._serial = None
            return False

    
    def disconnect(self) -> None:
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                print("[SerialComm] Connection closed.")


    def send(self, command: str) -> bool:

        if command not in VALID_COMMANDS:
            print(f"[SerialComm] Unknown command '{command}'. Ignored.")
            return False

        # Throttle rapid consecutive sends
        now = time.time()
        if now - self._last_send_time < WRITE_DELAY:
            time.sleep(WRITE_DELAY - (now - self._last_send_time))

        with self._lock:
            if self._serial is None or not self._serial.is_open:
                print(f"[SerialComm] Not connected — mock send: '{command}'")
                self._last_send_time = time.time()
                return False

            try:
                self._serial.write((command + "\n").encode("utf-8"))
                self._serial.flush()
                self._last_send_time = time.time()
                print(f"[SerialComm] Sent: '{command}'")
                return True
            except serial.SerialException as exc:
                print(f"[SerialComm] Write error: {exc}")
                return False


    def read_line(self) -> Optional[str]:

        with self._lock:
            if self._serial is None or not self._serial.is_open:
                return None
            try:
                if self._serial.in_waiting:
                    return self._serial.readline().decode("utf-8").strip()
            except serial.SerialException as exc:
                print(f"[SerialComm] Read error: {exc}")
        return None


    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open


    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()


def find_arduino_port() -> Optional[str]:

    if not SERIAL_AVAILABLE:
        return None

    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if "arduino" in desc or "ch340" in desc or "cp210" in desc or "acm" in desc.lower():
            print(f"[SerialComm] Auto-detected Arduino at: {port.device}")
            return port.device

    print("[SerialComm] No Arduino port auto-detected.")
    return None



if __name__ == "__main__":
    port = find_arduino_port() or DEFAULT_PORT
    comm = SerialComm(port=port)

    if comm.connect():
        for cmd in [CMD_LEFT, CMD_STRAIGHT, CMD_RIGHT, CMD_STOP]:
            comm.send(cmd)
            time.sleep(1)

        comm.disconnect()
    else:
        print("[SerialComm] Demo: would have sent left / straight / right / stop.")
