// 1) SCAN: drive rows across the arena while the wiper sweeps. At a wall: back up
//    a little (leaves the pieces at the wall), U-turn into the next row.
// 2) After SCAN_MS: drive to a wall and hug it all the way round the box,
//    collecting the pieces. At each corner look left/right, turn to the
//    side with more room. Stop after CORNERS corners.

#include <Servo.h>

const int ENA = 11, IN2 = 5, IN1 = 7;   // left motors
const int ENB = 3,  IN3 = 6, IN4 = 4;   // right motors
const int TRIG = A0, ECHO = A1;
const int LOOK_PIN = 9;                 // servo the ultrasonic sits on
const int WIPER_PIN = 8;                // wiper servo
const int LOOK_FWD = 90, LOOK_LEFT = 10, LOOK_RIGHT = 170;   // CALIBRATE: swap LEFT/RIGHT if backwards

int SPEED = 200;
int TURN_90_MS = 500;     // CALIBRATE: time for a 90 degree turn
int LANE_MS = 500;        // CALIBRATE: shift between rows, about one robot width
int BACKUP_MS = 300;      // CALIBRATE: back off the wall to leave the pieces
const int WALL_CM = 20;            // CALIBRATE: stop this close to a wall while scanning
const int SWEEP_STOP_CM = 25;     // CALIBRATE: stop sweeping this close to a wall
const unsigned long SCAN_MS = 30000;   

const int HUG_CM = 10;       // CALIBRATE: distance kept from the side wall - raise if it scrapes
const int CORNER_CM = 25;    // CALIBRATE: turn at corners this far from the front wall
const int HUG_SPEED = 200;
const int STEER = 35;        // CALIBRATE: lower if it zigzags, raise if it drifts off the wall
const int CORNERS = 4;       // CALIBRATE: corners for one full lap (angled box corners may need more)
const int LOOK_SETTLE_MS = 500;   // CALIBRATE: time for the look servo to move - lower = less blind time

// ---------- wiper ----------
const int WIPE_MIN = 40, WIPE_MAX =130 ;     // CALIBRATE: narrow this if the wiper hits the chassis
const int WIPE_STEP = 2;                    // degrees per move (bigger = faster)
const unsigned long WIPE_INTERVAL = 10;     // ms between moves
const bool WIPE_IN_LAP = false;             // true = keep wiping during the wall lap
const int WIPE_PARK = 160;                  // CALIBRATE: wiper angle (all the way right) before a U-turn
const int WIPE_PARK_MS = 500;               // CALIBRATE: wait for the wiper to get there

// ---------- debounce ----------
const int CONFIRM_READS = 2;                // close readings in a row before reacting
const unsigned long PING_INTERVAL = 30;     // ms between pings (avoids old echoes)

Servo look, wiper;
bool turnRightNext = false;
bool hugRight = true;
unsigned long startTime;

int wipeAngle = WIPE_MIN, wipeDir = WIPE_STEP;
unsigned long lastWipe = 0, lastPing = 0;
long lastDist = 999;
int closeCount = 0;

// Non-blocking: moves the wiper one step each time it's due. Holds still near a wall.
void sweep(bool tooCloseToWall) {
  if (tooCloseToWall) return;
  if (millis() - lastWipe < WIPE_INTERVAL) return;
  lastWipe = millis();

  wipeAngle += wipeDir;
  if (wipeAngle >= WIPE_MAX)      { wipeAngle = WIPE_MAX; wipeDir = -WIPE_STEP; }
  else if (wipeAngle <= WIPE_MIN) { wipeAngle = WIPE_MIN; wipeDir =  WIPE_STEP; }
  wiper.write(wipeAngle);
}

// Like delay(), but keeps the wiper going if wipe is true.
void wait(unsigned long ms, bool wipe) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    if (wipe) sweep(false);
  }
}

// ---------- motors ----------
void leftSide(int s) {
  digitalWrite(IN1, s > 0); digitalWrite(IN2, s < 0);
  analogWrite(ENA, abs(s));
}

void rightSide(int s) {
  digitalWrite(IN3, s > 0); digitalWrite(IN4, s < 0);
  analogWrite(ENB, abs(s));
}

void stopMotors() { leftSide(0); rightSide(0); delay(200); }

void forward(int ms) {
  leftSide(SPEED); rightSide(SPEED);
  delay(ms);
  stopMotors();
}

void backward(int ms) {
  leftSide(-SPEED); rightSide(-SPEED);
  delay(ms);
  stopMotors();
}

void turn(bool right, int ms) {
  if (right) { leftSide(SPEED);  rightSide(-SPEED); }
  else       { leftSide(-SPEED); rightSide(SPEED); }
  delay(ms);
  stopMotors();
}

// ---------- ultrasonic ----------
long distance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);   // timeout so it can't freeze
  return us == 0 ? 999 : us / 58;
}

// Blocking check: true only if CONFIRM_READS readings in a row are within cm.
bool confirmClose(int cm) {
  for (int i = 0; i < CONFIRM_READS; i++) {
    if (i > 0) delay(PING_INTERVAL);
    if (distance() > cm) return false;
  }
  return true;
}

long lookAt(int angle) {
  look.write(angle);
  delay(LOOK_SETTLE_MS);
  return distance();
}

// Look left and right, turn towards the side with more room.
// Returns true if it turned right.
bool turnToMoreRoom() {
  long l = lookAt(LOOK_LEFT);
  long r = lookAt(LOOK_RIGHT);
  look.write(LOOK_FWD);
  delay(LOOK_SETTLE_MS);
  bool right = r > l;
  turn(right, TURN_90_MS);
  return right;
}

// ---------- 1) scan ----------
void nextRow() {
  wiper.write(WIPE_PARK);                 // move wiper all the way right first
  wipeAngle = WIPE_PARK;
  delay(WIPE_PARK_MS);                    // wait until it's there
  backward(BACKUP_MS);
  turn(turnRightNext, TURN_90_MS);
  if (confirmClose(WALL_CM)) {            // side wall: go back the other way
    turn(turnRightNext, 2 * TURN_90_MS);
    turnRightNext = !turnRightNext;
  }
  forward(LANE_MS);
  turn(turnRightNext, TURN_90_MS);
  turnRightNext = !turnRightNext;
}

void scanStep() {
  if (millis() - lastPing >= PING_INTERVAL) {
    lastPing = millis();
    lastDist = distance();
    closeCount = (lastDist <= WALL_CM) ? closeCount + 1 : 0;
  }

  sweep(lastDist <= SWEEP_STOP_CM);

  if (closeCount < CONFIRM_READS) {
    leftSide(SPEED); rightSide(SPEED);
  } else {
    stopMotors();
    nextRow();
    closeCount = 0;
    lastDist = 999;                       // fresh start for the new row
  }
}

// ---------- 2) wall lap ----------
void goToWall() {
  look.write(LOOK_FWD);
  delay(LOOK_SETTLE_MS);
  leftSide(SPEED); rightSide(SPEED);

  int n = 0;
  while (n < CONFIRM_READS) {
    n = (distance() <= WALL_CM) ? n + 1 : 0;
    wait(PING_INTERVAL, WIPE_IN_LAP);
  }

  stopMotors();
  backward(BACKUP_MS);
  hugRight = !turnToMoreRoom();          // turned left -> wall is on the right
}

// Corner if ANY of 3 front readings is close (forgiving)
bool cornerAhead() {
  look.write(LOOK_FWD);
  delay(LOOK_SETTLE_MS);
  for (int i = 0; i < 3; i++) {
    if (distance() <= CORNER_CM) return true;
    delay(PING_INTERVAL);
  }
  return false;
}

void wallLap() {
  int corners = 0;
  int side = hugRight ? LOOK_RIGHT : LOOK_LEFT;
  unsigned long lastFront = millis();
  lookAt(side);

  while (corners < CORNERS) {
        if (millis() - lastFront > 800) {     // glance forward for a corner
      stopMotors();                       // don't drive blind while looking
      if (cornerAhead()) {
        stopMotors();
        turnToMoreRoom();
        corners++;
        Serial.println(corners);
      }
      lookAt(side);
      lastFront = millis();
    }

    long d = distance();
    int steer = 0;                        // + = towards the wall
    if (d > HUG_CM + 4)      steer = STEER;
    else if (d < HUG_CM - 4) steer = -STEER;
    if (!hugRight) steer = -steer;
    leftSide(HUG_SPEED + steer);
    rightSide(HUG_SPEED - steer);
    wait(PING_INTERVAL, WIPE_IN_LAP);
  }
  stopMotors();
}

// ---------- main ----------
void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  look.attach(LOOK_PIN);
  look.write(LOOK_FWD);
  wiper.attach(WIPER_PIN);
   wiper.write(wipeAngle);
  stopMotors();
  delay(2000);
  turn(true, TURN_90_MS / 2);             // half right turn at the start
  startTime = millis();
  Serial.println("SETUP DONE");
}

void loop() {
  if (millis() - startTime < SCAN_MS) {
    scanStep();
    return;
  }
  goToWall();
  wallLap();
  while (true) {}          // done
}
