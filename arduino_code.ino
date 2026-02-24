#include <Servo.h>

// -------------------- PINS --------------------
// Servos
const uint8_t SERVO_LID_PIN  = 5;
const uint8_t SERVO_DOOR_PIN = 6;

// IR Sensor
const uint8_t IR_SENSOR_PIN = 7; // HIGH = clear, LOW = object

// Ultrasonic sensors for 3 bins
const uint8_t TRIG_BIO     = 8;
const uint8_t ECHO_BIO     = 9;
const uint8_t TRIG_NONBIO  = 10;
const uint8_t ECHO_NONBIO  = 11;
const uint8_t TRIG_RECYCLE = 12;
const uint8_t ECHO_RECYCLE = 13;

// LEDs
const uint8_t LED_GREEN  = 2;   // Battery OK
const uint8_t LED_YELLOW = 3;   // Battery LOW
const uint8_t LED_RED    = A3;  // Any bin full

// -------------------- SERVO ANGLES --------------------
const int LID_HOME_ANGLE     = 90;
const int LID_ANGLE_BIO      = 5;
const int LID_ANGLE_NONBIO   = 90;
const int LID_ANGLE_RECYCLE  = 180;

const int DOOR_CLOSED_ANGLE  = 100;
const int DOOR_OPEN_ANGLE    = 0;

// Time & distance thresholds
const unsigned long MAX_WAIT_TIME   = 30000;
const unsigned long IR_CLEAR_DELAY  = 1000;
const int BIN_FULL_CM = 10;

Servo servoLid, servoDoor;

// -------------------- ULTRASONIC --------------------
long readUltrasonic(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  long cm = duration / 29 / 2;

  // // --------- DEBUG OUTPUT ----------
  // Serial.print("Ultrasonic [TRIG=");
  // Serial.print(trig);
  // Serial.print(", ECHO=");
  // Serial.print(echo);
  // Serial.print("] Distance: ");
  // Serial.print(cm);
  // Serial.println(" cm");
  // // ---------------------------------

  return cm;
}


bool anyBinFull() {
  long bio     = readUltrasonic(TRIG_BIO, ECHO_BIO);
  long nonbio  = readUltrasonic(TRIG_NONBIO, ECHO_NONBIO);
  long recycle = readUltrasonic(TRIG_RECYCLE, ECHO_RECYCLE);

  return (bio < BIN_FULL_CM || nonbio < BIN_FULL_CM || recycle < BIN_FULL_CM);
}

// -------------------- SERVO MOVEMENT --------------------
void moveServo(Servo &servo, int angle, int delayMs = 400) {
  servo.write(angle);
  delay(delayMs);
  servo.detach();
}

// -------------------- COMMAND PARSE --------------------
int parseCmd(String s) {
  s.trim();
  if (s == "BIO") return 0;
  if (s == "NON") return 1;
  if (s == "REC") return 2;
  return -1;
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(IR_SENSOR_PIN, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(TRIG_BIO, OUTPUT);
  pinMode(ECHO_BIO, INPUT);

  pinMode(TRIG_NONBIO, OUTPUT);
  pinMode(ECHO_NONBIO, INPUT);

  pinMode(TRIG_RECYCLE, OUTPUT);
  pinMode(ECHO_RECYCLE, INPUT);

  servoLid.attach(SERVO_LID_PIN);
  servoDoor.attach(SERVO_DOOR_PIN);

  servoLid.write(LID_HOME_ANGLE);
  servoDoor.write(DOOR_CLOSED_ANGLE);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.println("Arduino UNO READY...");
}

// -------------------- LOOP --------------------
void loop() {

  // ----- DISPLAY EVERYTHING RECEIVED FROM ESP32 CAM -----
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    Serial.print("ESP32 says: ");
    Serial.println(msg);

    // process classification commands
    if (msg == "BIO" || msg == "NON" || msg == "REC") {
      handleSort(msg);
    }
  }

  // ----- BIN FULL MONITOR -----
  if (anyBinFull()) digitalWrite(LED_YELLOW, HIGH);
  else digitalWrite(LED_YELLOW, LOW);

  // ----- BATTERY MONITOR -----
  int batteryRaw = analogRead(A0);
  float batteryVolt = batteryRaw * (12.0 / 1023.0 * (100+47)/47); 

  if (batteryVolt >= 11.5) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }
  delay (2000);
}

// ======================= SORTING FUNCTION =======================
// ======================= SORTING FUNCTION =======================
void handleSort(String msg) {

  int bin = parseCmd(msg);
  int angle =
      (bin == 0) ? LID_ANGLE_BIO :
      (bin == 1) ? LID_ANGLE_NONBIO :
                   LID_ANGLE_RECYCLE;

  Serial.print("Sorting to: ");
  Serial.println(msg);

  // ------------------ MOVE LID FIRST ------------------
  servoLid.attach(SERVO_LID_PIN);
  servoLid.write(angle);
  Serial.println("Moving lid to target angle...");
  delay(700);   // ensure lid reaches the angle

  Serial.println("Lid reached position.");

  // ------------------ OPEN DOOR AFTER LID ------------------
  servoDoor.attach(SERVO_DOOR_PIN);
  servoDoor.write(DOOR_OPEN_ANGLE);
  Serial.println("Door opened.");
  delay(400);  // let the door fully open

  // ------------------ WAIT FOR IR TO CLEAR ------------------
  Serial.println("Waiting for trash to clear IR...");

  unsigned long startTime = millis();
  bool cleared = false;

  while (millis() - startTime < MAX_WAIT_TIME) {

    if (digitalRead(IR_SENSOR_PIN) == HIGH) {  // HIGH = no object
      unsigned long clearStart = millis();
      while (digitalRead(IR_SENSOR_PIN) == HIGH) {
        if (millis() - clearStart >= IR_CLEAR_DELAY) {
          cleared = true;
          break;
        }
      }
      if (cleared) break;
    }
  }

  // ------------------ CLOSE DOOR ONLY WHEN CLEAR ------------------
  if (cleared) {
    Serial.println("IR clear. Closing door...");

    servoDoor.attach(SERVO_DOOR_PIN);
    servoDoor.write(DOOR_CLOSED_ANGLE);
    delay(500);

    // Return lid to home
    servoLid.attach(SERVO_LID_PIN);
    servoLid.write(LID_HOME_ANGLE);
    delay(700);

    Serial.println("Trash dropped. System ready.");

  } else {
    Serial.println("Timeout waiting for IR clear.");
  }
}

