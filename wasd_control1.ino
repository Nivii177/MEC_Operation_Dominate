// Keyboard drive: receives w/a/s/d/x from drive_wasd.py over USB
// Stops by itself if no command arrives for 300 ms (script closed / cable out)
// f = wiper on/off, r = park wiper at 160

#include <Servo.h>

const int ENA = 11, IN2 = 5, IN1 = 7;   // left motors
const int ENB = 3,  IN3 = 6, IN4 = 4;   // right motors
const int LOOK_PIN = 9, WIPER_PIN = 8;
Servo look, wiper;

int SPEED = 200;
unsigned long lastCmd = 0;

const int WIPE_MIN = 40, WIPE_MAX = 130, WIPE_PARK = 160;
bool wiping = false;
int wipeAngle = 90, wipeDir = 2;
unsigned long lastWipe = 0;

void sweep() {                      // one 2-degree step every 10 ms
  if (millis() - lastWipe < 10) return;
  lastWipe = millis();
  wipeAngle += wipeDir;
  if (wipeAngle >= WIPE_MAX)      { wipeAngle = WIPE_MAX; wipeDir = -2; }
  else if (wipeAngle <= WIPE_MIN) { wipeAngle = WIPE_MIN; wipeDir =  2; }
  wiper.write(wipeAngle);
}

void leftSide(int s) {
  digitalWrite(IN1, s > 0); digitalWrite(IN2, s < 0);
  analogWrite(ENA, abs(s));
}

void rightSide(int s) {
  digitalWrite(IN3, s > 0); digitalWrite(IN4, s < 0);
  analogWrite(ENB, abs(s));
}

void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  look.attach(LOOK_PIN);   look.write(90);
  wiper.attach(WIPER_PIN); wiper.write(90);
  leftSide(0); rightSide(0);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCmd = millis();
    if (c == 'w')      { leftSide(SPEED);  rightSide(SPEED); }
    else if (c == 's') { leftSide(-SPEED); rightSide(-SPEED); }
    else if (c == 'a') { leftSide(-SPEED); rightSide(SPEED); }
    else if (c == 'd') { leftSide(SPEED);  rightSide(-SPEED); }
    else if (c == 'x') { leftSide(0);      rightSide(0); }
    else if (c == '+') SPEED = min(SPEED + 20, 255);
    else if (c == '-') SPEED = max(SPEED - 20, 60);
    else if (c == 'f') wiping = !wiping;                       // wiper on/off
    else if (c == 'r') { wiping = false; wipeAngle = WIPE_PARK; wiper.write(WIPE_PARK); }  // park right
  }

  if (wiping) sweep();

  if (millis() - lastCmd > 300) {   // safety stop
    leftSide(0); rightSide(0);
  }
}
