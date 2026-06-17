#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>

Ultrasonic sonarLeft(12, 13);
Ultrasonic sonarRight(14, 27);
Ultrasonic sonarFrontLeft(26, 25);
Ultrasonic sonarFrontRight(33, 32);

static const int PIN_SERVO = 18;
static const int PIN_ESC   = 19;
static const int PIN_HALL  = 34;

volatile uint32_t hallCount  = 0;
volatile uint32_t lastHallUs = 0;
void IRAM_ATTR hallISR() {
  hallCount++;
  lastHallUs = micros();
}

// Protocole série (envoyé par lidar_ld19.py) :
//   format    : "<angle_deg>,<rpm>\n"
//   angle_deg : gap_center_deg × 0.75  (pré-mis à l'échelle côté Python, plage ≈ ±40°)
//   rpm       : fixé à 100 par lidar_ld19.py → consigne = 4 × 100 = 400 RPM après mise à l'échelle

// Angle du gap reçu, déjà pré-mis à l'échelle par 0.75 depuis lidar_ld19.py ; utilisé directement comme décalage servo
double gapAngleDeg = 0.0;

Servo servo;

float Kp = 0.8;
float Ki = 0.2;
float Kd = 0.03;

static const double ESC_NEUTRAL = 1500.0;
static const double ESC_MAX     = 1900.0;

double pid_rpm_input = 0.0, pid_rpm_output = 0.0, pid_rpm_setpoint = 0.0;
PID pidRpm(&pid_rpm_input, &pid_rpm_output, &pid_rpm_setpoint, Kp, Ki, Kd, DIRECT);
Servo esc;

static uint32_t prevHallCount  = 0;
static uint32_t prevHallMillis = 0;
static double   measuredRpm    = 0.0;

void updateRpm() {
  uint32_t now = millis();
  uint32_t dt  = now - prevHallMillis;
  if (dt < 50) return;
  noInterrupts();
  uint32_t cnt    = hallCount;
  uint32_t lastUs = lastHallUs;
  interrupts();
  uint32_t delta = cnt - prevHallCount;
  prevHallCount  = cnt;
  prevHallMillis = now;
  // Les fronts RISING et FALLING sont tous deux comptés, donc chaque rotation donne 2 impulsions → 15000 au lieu de 30000
  measuredRpm = (delta * 15000.0) / dt;
}

static uint32_t lastServoUpdateMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, RISING);
  prevHallMillis = millis();
  pidRpm.SetMode(AUTOMATIC);
  pidRpm.SetOutputLimits(0.0, ESC_MAX - ESC_NEUTRAL);
  pidRpm.SetSampleTime(100);
  ESP32PWM::allocateTimer(1);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC);
  servo.attach(PIN_SERVO);
  esc.writeMicroseconds(ESC_NEUTRAL);  // Séquence d'armement de l'ESC
  prevHallCount = hallCount;
  lastServoUpdateMs = millis();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      double gapAngleReceived = cmd.substring(0, commaIndex).toDouble();
      double rpmReceived      = cmd.substring(commaIndex + 1).toDouble();
      pid_rpm_setpoint = 4 * rpmReceived;  // lidar_ld19.py envoie 100 → consigne = 400 RPM
      gapAngleDeg = gapAngleReceived;
    }
  }

  updateRpm();
  pid_rpm_input = measuredRpm;

  // Limiter les écritures servo à des intervalles de 60 ms pour éviter de saturer le servo de commandes
  uint32_t nowMs = millis();
  if (nowMs - lastServoUpdateMs > 60) {
    lastServoUpdateMs = nowMs;
    double servoAngle = 90.0 + gapAngleDeg;
    float distRight = sonarRight.read();
    float distLeft  = sonarLeft.read();

    if (distRight < 7.0 && distRight > 0.0) servoAngle -= 30.0;  // obstacle à droite → braquer à gauche
    if (distLeft  < 7.0 && distLeft  > 0.0) servoAngle += 30.0;  // obstacle à gauche → braquer à droite

    servo.write(servoAngle);
  }

  if (pidRpm.Compute()) {
    double escCommand = ESC_NEUTRAL + pid_rpm_output;
    if (escCommand > ESC_MAX)             escCommand = ESC_MAX;
    if (escCommand < ESC_NEUTRAL) escCommand = ESC_NEUTRAL;  // pas de marche arrière
    esc.writeMicroseconds((int)escCommand);
  }
}
