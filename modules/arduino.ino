/*
 * AutoTrack - Arduino Motor and Sensor Controller
 *
 * Receives navigation commands from the Raspberry Pi,
 * controls the L298N motor driver, and monitors obstacles
 * using an HC-SR04 ultrasonic sensor.
 */
 
const int PIN_ENA = 5;   // PWM speed control : left motor
const int PIN_IN1 = 7;   // Direction control A
const int PIN_IN2 = 8;   // Direction control B

const int PIN_ENB = 6;   // PWM speed control : right motor
const int PIN_IN3 = 9;   // Direction control A
const int PIN_IN4 = 10;  // Direction control B

const int PIN_TRIG = 3;
const int PIN_ECHO = 4;

const int PWM_SPEED_BASE = 180;   // Default straight-line speed (~70%)
const int PWM_SPEED_TURN = 200;   // Outer wheel during a turn
const int PWM_SPEED_SLOW = 90;    // Inner wheel during a turn (tighter turn)
const int PWM_SPEED_40   = 100;   // Speed limit 40 sign (~39%)
const int PWM_SPEED_70   = 178;   // Speed limit 70 sign (~70%)

const float STOP_DISTANCE_CM = 30.0;

const unsigned long ULTRASONIC_INTERVAL_MS = 100;

const unsigned long ECHO_TIMEOUT_US = 30000;

const int SERIAL_BUFFER_SIZE = 32;

int  g_currentSpeed      = PWM_SPEED_BASE;

bool g_obstacleDetected  = false;

unsigned long g_lastUltrasonicTime = 0;

char g_serialBuf[SERIAL_BUFFER_SIZE];
int  g_serialIdx = 0;

void motorStop();
void motorForward(int leftPWM, int rightPWM);
void motorTurnLeft(int outerPWM, int innerPWM);
void motorTurnRight(int outerPWM, int innerPWM);
void handleCommand(const char* cmd);
float measureDistanceCM();
void checkUltrasonic();

void setup() {

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);

  motorStop();

  Serial.begin(9600);

  delay(500);

  Serial.println("[Arduino] AutoTrack controller ready.");
}

void loop() {

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {

      if (g_serialIdx > 0) {
        g_serialBuf[g_serialIdx] = '\0';   // Null-terminate the C string.
        handleCommand(g_serialBuf);
        g_serialIdx = 0;                   // Reset buffer for next command.
      }
    } else {

      if (g_serialIdx < SERIAL_BUFFER_SIZE - 1) {
        g_serialBuf[g_serialIdx++] = c;
      }

      else {
        g_serialIdx = 0;
      }
    }
  }

  unsigned long now = millis();
  if (now - g_lastUltrasonicTime >= ULTRASONIC_INTERVAL_MS) {
    g_lastUltrasonicTime = now;
    checkUltrasonic();
  }
}

void handleCommand(const char* cmd) {

  if (strcmp(cmd, "left") == 0) {

    motorTurnLeft(PWM_SPEED_TURN, PWM_SPEED_SLOW);
    Serial.println("[Arduino] Executing: LEFT");

  } else if (strcmp(cmd, "right") == 0) {

    motorTurnRight(PWM_SPEED_TURN, PWM_SPEED_SLOW);
    Serial.println("[Arduino] Executing: RIGHT");

  } else if (strcmp(cmd, "straight") == 0) {

    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Executing: STRAIGHT");

  } else if (strcmp(cmd, "stop") == 0) {

    motorStop();
    Serial.println("[Arduino] Executing: STOP");

  } else if (strcmp(cmd, "speed_40") == 0) {

    g_currentSpeed = PWM_SPEED_40;
    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Speed limit 40 applied.");

  } else if (strcmp(cmd, "speed_70") == 0) {

    g_currentSpeed = PWM_SPEED_70;
    motorForward(g_currentSpeed, g_currentSpeed);
    Serial.println("[Arduino] Speed limit 70 applied.");

  } else {

    Serial.print("[Arduino] Unknown command: ");
    Serial.println(cmd);
  }
}

void motorStop() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

void motorForward(int leftPWM, int rightPWM) {

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);

  analogWrite(PIN_ENA, leftPWM);
  analogWrite(PIN_ENB, rightPWM);
}

void motorTurnLeft(int outerPWM, int innerPWM) {

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, innerPWM);

  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENB, outerPWM);
}

void motorTurnRight(int outerPWM, int innerPWM) {

  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, outerPWM);

  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENB, innerPWM);
}

float measureDistanceCM() {

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {

    return -1.0;
  }

  float distance = (float)duration * 0.0343 / 2.0;
  return distance;
}

void checkUltrasonic() {
  float dist = measureDistanceCM();

  bool obstacleNow = (dist > 0 && dist < STOP_DISTANCE_CM);

  if (obstacleNow && !g_obstacleDetected) {

    g_obstacleDetected = true;
    Serial.print("obstacle detected (");
    Serial.print(dist, 1);    // 1 decimal place
    Serial.println(" cm)");

  } else if (!obstacleNow && g_obstacleDetected) {

    g_obstacleDetected = false;
    Serial.println("path is clear");
  }

}
