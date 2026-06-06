/*
 * ============================================================
 *  AutoTrack — Arduino Motor & Sensor Controller
 *  Tribhuvan University, Pulchowk Campus
 *  Department of Electronics & Computer Engineering
 *
 *  Team:
 *    Sakshyam Luitel  (PUL078BCT073)
 *    Sijan Khadka     (PUL078BCT088)
 *    Sudip Basnet     (PUL078BCT092)
 *    Sujan Thapaliya  (PUL078BCT094)
 *
 *  Description:
 *    This sketch runs on the Arduino and is responsible for
 *    ALL low-level hardware control in the AutoTrack system.
 *    The Raspberry Pi handles high-level vision (lane detection,
 *    traffic sign detection) and sends text commands over UART.
 *    This Arduino code:
 *      1. Reads commands from the Raspberry Pi via Serial.
 *      2. Controls two DC motors through the L298N motor driver
 *         using PWM signals for direction and speed.
 *      3. Continuously polls the HC-SR04 ultrasonic sensor.
 *      4. Sends "obstacle detected" or "path is clear" messages
 *         back to the Raspberry Pi so it can halt if needed.
 *      5. Adjusts base driving speed when a speed-limit sign
 *         command (speed_40 or speed_70) is received.
 *
 *  Wiring Summary:
 *    --- L298N Motor Driver ---
 *    ENA  (Left motor speed)   -> Arduino Pin 5  (PWM)
 *    IN1  (Left motor dir A)   -> Arduino Pin 7
 *    IN2  (Left motor dir B)   -> Arduino Pin 8
 *    IN3  (Right motor dir A)  -> Arduino Pin 9
 *    IN4  (Right motor dir B)  -> Arduino Pin 10
 *    ENB  (Right motor speed)  -> Arduino Pin 6  (PWM)
 *    GND  -> Common GND (Arduino + battery pack)
 *    12V  -> Battery pack positive
 *    5V   -> Do NOT connect to Arduino 5V (use separate regulator)
 *
 *    --- HC-SR04 Ultrasonic Sensor ---
 *    VCC  -> Arduino 5V
 *    GND  -> Arduino GND
 *    TRIG -> Arduino Pin 3
 *    ECHO -> Arduino Pin 4
 *
 *    --- Raspberry Pi UART ---
 *    RPi TX (GPIO14) -> Arduino RX (Pin 0)  [through 3.3V–5V divider]
 *    RPi RX (GPIO15) <- Arduino TX (Pin 1)
 *    Common GND between RPi and Arduino
 *
 *  Serial Protocol:
 *    Baud rate : 9600
 *    Commands from RPi (newline-terminated strings):
 *      "left\n"       - Turn left
 *      "right\n"      - Turn right
 *      "straight\n"   - Go straight
 *      "stop\n"       - Stop all motors
 *      "speed_40\n"   - Set base speed to 40% PWM (~100/255)
 *      "speed_70\n"   - Set base speed to 70% PWM (~178/255)
 *    Messages sent to RPi:
 *      "obstacle detected\n"  - Object within STOP_DISTANCE_CM
 *      "path is clear\n"      - No object within range
 * ============================================================
 */

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────

// Left motor (Motor A on L298N)
const int PIN_ENA = 5;   // PWM speed control — left motor
const int PIN_IN1 = 7;   // Direction control A
const int PIN_IN2 = 8;   // Direction control B

// Right motor (Motor B on L298N)
const int PIN_ENB = 6;   // PWM speed control — right motor
const int PIN_IN3 = 9;   // Direction control A
const int PIN_IN4 = 10;  // Direction control B

// HC-SR04 Ultrasonic Sensor
const int PIN_TRIG = 3;
const int PIN_ECHO = 4;

// ─────────────────────────────────────────────────────────────
//  CONFIGURATION CONSTANTS
// ─────────────────────────────────────────────────────────────

/*
 * How PWM works on the L298N:
 *   analogWrite(pin, value) sends a PWM signal where value ranges
 *   from 0 (always LOW, motor stopped) to 255 (always HIGH, full speed).
 *   The L298N ENA/ENB pins accept this signal directly and use it to
 *   control how much voltage reaches the motor.
 *
 *   PWM_SPEED_BASE  : Normal driving speed (~70% of max).
 *   PWM_SPEED_TURN  : Speed of the faster wheel during a turn.
 *   PWM_SPEED_SLOW  : Speed of the slower wheel during a turn.
 *   PWM_SPEED_40    : Reduced speed when "speed limit 40" sign seen.
 *   PWM_SPEED_70    : Normal speed when "speed limit 70" sign seen.
 */
const int PWM_SPEED_BASE = 180;   // Default straight-line speed (~70%)
const int PWM_SPEED_TURN = 200;   // Outer wheel during a turn
const int PWM_SPEED_SLOW = 90;    // Inner wheel during a turn (tighter turn)
const int PWM_SPEED_40   = 100;   // Speed limit 40 sign (~39%)
const int PWM_SPEED_70   = 178;   // Speed limit 70 sign (~70%)

/*
 * Obstacle threshold: if the ultrasonic sensor measures a distance
 * less than STOP_DISTANCE_CM, the Arduino sends "obstacle detected"
 * to the Raspberry Pi, which then halts the vehicle.
 */
const float STOP_DISTANCE_CM = 30.0;

/*
 * How often (in milliseconds) the ultrasonic sensor is polled.
 * 100 ms means 10 readings per second, which is fast enough to
 * react to objects while not overloading the main loop.
 */
const unsigned long ULTRASONIC_INTERVAL_MS = 100;

/*
 * Maximum time (in microseconds) to wait for the echo pulse.
 * 30000 µs corresponds to ~5 metres. If no echo returns within
 * this time, we treat the path as clear.
 */
const unsigned long ECHO_TIMEOUT_US = 30000;

/*
 * Serial read buffer size. Commands from RPi are short strings
 * like "straight\n" (10 chars), so 32 bytes is more than enough.
 */
const int SERIAL_BUFFER_SIZE = 32;

// ─────────────────────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────────────────────

// Tracks the active speed so speed-limit commands persist across
// direction changes.
int  g_currentSpeed      = PWM_SPEED_BASE;

// Stores the last obstacle state so we only print when it changes,
// avoiding flooding the serial port.
bool g_obstacleDetected  = false;

// Timestamp of the last ultrasonic reading.
unsigned long g_lastUltrasonicTime = 0;

// Serial input accumulation buffer and index.
char g_serialBuf[SERIAL_BUFFER_SIZE];
int  g_serialIdx = 0;


// ─────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────
void motorStop();
void motorForward(int leftPWM, int rightPWM);
void motorTurnLeft(int outerPWM, int innerPWM);
void motorTurnRight(int outerPWM, int innerPWM);
void handleCommand(const char* cmd);
float measureDistanceCM();
void checkUltrasonic();


// ─────────────────────────────────────────────────────────────
//  setup()
//  Runs once when the Arduino powers on or is reset.
// ─────────────────────────────────────────────────────────────
void setup() {

  // --- Motor driver pins ---
  // Setting these as OUTPUT tells the ATmega chip to drive them
  // actively (push current out or sink current in) rather than
  // leaving them floating.
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  // --- Ultrasonic sensor pins ---
  // TRIG must be OUTPUT because we pulse it.
  // ECHO must be INPUT because we listen for the returning pulse.
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Ensure TRIG starts LOW so we don't fire a spurious pulse.
  digitalWrite(PIN_TRIG, LOW);

  // --- Safety: start with motors stopped ---
  // It is important to explicitly stop the motors on startup.
  // If the Arduino resets while the car is moving, this ensures
  // the motors halt immediately rather than continuing at whatever
  // the last PWM value was.
  motorStop();

  // --- Serial communication ---
  // 9600 baud matches DEFAULT_BAUDRATE in serial_comm.py.
  // At 9600 baud, each character takes ~104 µs, so even short
  // commands like "left\n" arrive in well under 1 ms.
  Serial.begin(9600);

  // Small delay to let the serial port stabilise after the
  // Arduino-reset that pyserial triggers when it opens the port.
  delay(500);

  Serial.println("[Arduino] AutoTrack controller ready.");
}


// ─────────────────────────────────────────────────────────────
//  loop()
//  Runs continuously after setup() returns.
//  Two tasks run here:
//    1. Non-blocking serial read  -> handleCommand()
//    2. Timed ultrasonic polling  -> checkUltrasonic()
// ─────────────────────────────────────────────────────────────
void loop() {

  // ── Task 1: Read incoming serial bytes ──────────────────────
  //
  // Serial.available() returns the number of bytes sitting in
  // the hardware receive buffer (64 bytes on most Arduinos).
  // We read one byte at a time so that the ultrasonic polling
  // (Task 2) is never blocked waiting for a full command.
  //
  // Commands from the Raspberry Pi are newline-terminated
  // (serial_comm.py appends "\n" to every send). We accumulate
  // characters into g_serialBuf until we see '\n', then process
  // the complete command string.
  //
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      // Newline signals end of command.
      if (g_serialIdx > 0) {
        g_serialBuf[g_serialIdx] = '\0';   // Null-terminate the C string.
        handleCommand(g_serialBuf);
        g_serialIdx = 0;                   // Reset buffer for next command.
      }
    } else {
      // Accumulate the character if buffer has space.
      if (g_serialIdx < SERIAL_BUFFER_SIZE - 1) {
        g_serialBuf[g_serialIdx++] = c;
      }
      // If buffer overflows (malformed long message), reset silently.
      else {
        g_serialIdx = 0;
      }
    }
  }

  // ── Task 2: Ultrasonic polling ───────────────────────────────
  //
  // millis() returns the number of milliseconds since the Arduino
  // last reset. By comparing the current time with the last time
  // we ran the ultrasonic check, we achieve a non-blocking timer
  // that fires every ULTRASONIC_INTERVAL_MS milliseconds.
  //
  // This approach (called "Blink Without Delay" in Arduino docs)
  // is critical here: using delay() would freeze the serial read
  // task for the duration of the delay.
  //
  unsigned long now = millis();
  if (now - g_lastUltrasonicTime >= ULTRASONIC_INTERVAL_MS) {
    g_lastUltrasonicTime = now;
    checkUltrasonic();
  }
}


// ─────────────────────────────────────────────────────────────
//  handleCommand()
//  Parses a null-terminated command string and takes action.
//
//  The strcmp() function compares two C strings character by
//  character and returns 0 if they are identical. We use it
//  to match the received command against known strings.
// ─────────────────────────────────────────────────────────────
void handleCommand(const char* cmd) {

  if (strcmp(cmd, "left") == 0) {
    // ── Turn Left ──────────────────────────────────────────
    // In a differential drive robot, turning left means the
    // LEFT wheel rotates slower (or even backward) while the
    // RIGHT wheel rotates faster. This causes the robot to
    // curve to the left.
    //
    // PWM_SPEED_TURN  applied to the right (outer) wheel.
    // PWM_SPEED_SLOW  applied to the left  (inner) wheel.
    motorTurnLeft(PWM_SPEED_TURN, PWM_SPEED_SLOW);
    Serial.println("[Arduino] Executing: LEFT");

  } else if (strcmp(cmd, "right") == 0) {
    // ── Turn Right ─────────────────────────────────────────
    // Mirror of left: left wheel is faster, right wheel slower.
    motorTurnRight(PWM_SPEED_TURN, PWM_SPEED_SLOW);
    Serial.println("[Arduino] Executing: RIGHT");

  } else if (strcmp(cmd, "straight") == 0) {
    // ── Go Straight ────────────────────────────────────────
    // Both wheels run at the current speed (which may have been
    // adjusted by a speed-limit command).
    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Executing: STRAIGHT");

  } else if (strcmp(cmd, "stop") == 0) {
    // ── Stop ───────────────────────────────────────────────
    // Halts both motors. Called on obstacle detection,
    // stop sign, or Raspberry Pi shutdown.
    motorStop();
    Serial.println("[Arduino] Executing: STOP");

  } else if (strcmp(cmd, "speed_40") == 0) {
    // ── Speed Limit 40 ─────────────────────────────────────
    // Reduce base speed. If we are currently moving forward,
    // apply the new speed immediately. The new g_currentSpeed
    // also affects any future "straight" commands.
    g_currentSpeed = PWM_SPEED_40;
    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Speed limit 40 applied.");

  } else if (strcmp(cmd, "speed_70") == 0) {
    // ── Speed Limit 70 ─────────────────────────────────────
    // Restore normal speed.
    g_currentSpeed = PWM_SPEED_70;
    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Speed limit 70 applied.");

  } else {
    // Unknown command — log and ignore. Do NOT change motor state.
    Serial.print("[Arduino] Unknown command: ");
    Serial.println(cmd);
  }
}


// ─────────────────────────────────────────────────────────────
//  MOTOR CONTROL FUNCTIONS
//
//  The L298N H-Bridge works like this:
//
//  To spin a motor FORWARD:
//    IN1 = HIGH, IN2 = LOW   (for left motor)
//    IN3 = HIGH, IN4 = LOW   (for right motor)
//
//  To spin a motor BACKWARD:
//    IN1 = LOW,  IN2 = HIGH
//    IN3 = LOW,  IN4 = HIGH
//
//  To BRAKE (short-circuit braking, fast stop):
//    IN1 = HIGH, IN2 = HIGH  (or both LOW)
//
//  Speed is controlled independently via ENA and ENB using
//  analogWrite(), which generates a PWM signal. The L298N
//  uses the duty cycle of this signal to regulate motor voltage.
// ─────────────────────────────────────────────────────────────

/*
 * motorStop()
 *   Sets both motor enable pins to 0 (no PWM) and sets direction
 *   pins to brake configuration. The motor coasts to a stop.
 */
void motorStop() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}


/*
 * motorForward(leftPWM, rightPWM)
 *   Drives both motors forward.
 *   leftPWM  : PWM duty cycle for the left motor  (0–255)
 *   rightPWM : PWM duty cycle for the right motor (0–255)
 *
 *   Passing different values for leftPWM and rightPWM allows
 *   fine-tuned straight-line correction if one motor is slightly
 *   stronger than the other (common with cheap DC motors).
 */
void motorForward(int leftPWM, int rightPWM) {
  // Left motor: forward direction
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  // Right motor: forward direction
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);

  // Apply PWM speeds
  analogWrite(PIN_ENA, leftPWM);
  analogWrite(PIN_ENB, rightPWM);
}


/*
 * motorTurnLeft(outerPWM, innerPWM)
 *   Executes a left turn using differential speed.
 *
 *   outerPWM : Speed of the right (outer) wheel — faster.
 *   innerPWM : Speed of the left  (inner) wheel — slower.
 *
 *   Both wheels still spin forward; the speed difference
 *   creates the arc. For a sharper turn, set innerPWM to 0
 *   (pivot turn) or a negative equivalent using backward spin.
 *
 *   In this implementation both wheels go forward, which
 *   creates a smooth curve suitable for lane following on
 *   the arena track.
 */
void motorTurnLeft(int outerPWM, int innerPWM) {
  // Left motor (inner): slower forward
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, innerPWM);

  // Right motor (outer): faster forward
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENB, outerPWM);
}


/*
 * motorTurnRight(outerPWM, innerPWM)
 *   Mirror of motorTurnLeft.
 *   outerPWM : Speed of the left  (outer) wheel — faster.
 *   innerPWM : Speed of the right (inner) wheel — slower.
 */
void motorTurnRight(int outerPWM, int innerPWM) {
  // Left motor (outer): faster forward
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, outerPWM);

  // Right motor (inner): slower forward
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENB, innerPWM);
}


// ─────────────────────────────────────────────────────────────
//  ULTRASONIC SENSOR FUNCTIONS
// ─────────────────────────────────────────────────────────────

/*
 * measureDistanceCM()
 *   Triggers a single ultrasonic pulse and measures the echo
 *   return time, converting it to a distance in centimetres.
 *
 *   Physics:
 *     Speed of sound in air at ~20°C is approximately 343 m/s,
 *     or 0.0343 cm/µs.
 *     The pulse travels TO the object and BACK, so the total
 *     path length is 2 * distance.
 *     Therefore:  distance (cm) = duration (µs) * 0.0343 / 2
 *
 *   HC-SR04 Timing:
 *     1. Pull TRIG LOW for at least 2 µs to ensure a clean start.
 *     2. Pull TRIG HIGH for exactly 10 µs to fire the pulse.
 *     3. Pull TRIG LOW again.
 *     4. The sensor automatically drives ECHO HIGH for the
 *        duration of the round-trip travel time.
 *     5. pulseIn() measures how long ECHO stays HIGH in µs.
 *
 *   Returns -1.0 if no echo is received within ECHO_TIMEOUT_US,
 *   which we treat as "no obstacle in range."
 */
float measureDistanceCM() {
  // Step 1: Clear the TRIG pin
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  // Step 2: Fire a 10-µs HIGH pulse on TRIG
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Step 3: Measure the duration of the ECHO pulse
  // pulseIn() blocks until the pin goes HIGH, then times how long
  // it stays HIGH. The timeout prevents permanent blocking if no
  // echo returns (e.g., the sensor is facing open sky).
  long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    // Timeout: no echo received — path is clear beyond max range.
    return -1.0;
  }

  // Step 4: Convert duration to distance
  float distance = (float)duration * 0.0343 / 2.0;
  return distance;
}


/*
 * checkUltrasonic()
 *   Measures distance and sends a status message to the Raspberry Pi
 *   only when the obstacle state CHANGES. This avoids spamming the
 *   serial port with hundreds of identical messages per second, which
 *   would slow down command processing.
 *
 *   The Raspberry Pi (serial_comm.py _read_arduino thread) monitors
 *   for the string "obstacle detected" and sets the obstacle flag,
 *   which causes main_controller.py to dispatch a "stop" command
 *   back to this Arduino.
 */
void checkUltrasonic() {
  float dist = measureDistanceCM();

  // Treat -1 (timeout / no echo) as "clear"
  bool obstacleNow = (dist > 0 && dist < STOP_DISTANCE_CM);

  if (obstacleNow && !g_obstacleDetected) {
    // State change: clear -> obstacle
    g_obstacleDetected = true;
    Serial.print("obstacle detected (");
    Serial.print(dist, 1);    // 1 decimal place
    Serial.println(" cm)");

  } else if (!obstacleNow && g_obstacleDetected) {
    // State change: obstacle -> clear
    g_obstacleDetected = false;
    Serial.println("path is clear");
  }

  // If state has not changed, print nothing.
}
