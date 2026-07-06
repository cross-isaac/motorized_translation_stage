#include <Arduino.h>

// ---------------- pins ----------------
const int STEP_PIN = 25;
const int DIR_PIN  = 26;

// ------------- pulse timing -----------
const int STEP_HIGH_US = 5;    // pulse width; A4988 minimum is 1 us
const int STEP_LOW_US  = 200;  // gap between pulses -> controls speed

// ------------- mechanics --------------
// Driver is in 1/16 microstepping: 3200 steps per revolution.
// Stage: 1 revolution = 0.025" of travel  ->  128000 steps per inch.
const long   STEPS_PER_REV  = 3200;
const double INCHES_PER_REV = 0.025;
const double STEPS_PER_INCH = STEPS_PER_REV / INCHES_PER_REV;  // = 128000

const double MIN_INCH = 0.0;   // soft travel limits
const double MAX_INCH = 1.0;

// Which DIR level moves the stage toward MAX_INCH (increasing position).
// If GOTO drives the wrong way, swap these two.
const int DIR_POS = HIGH;      // increasing inches
const int DIR_NEG = LOW;       // decreasing inches

const long REPORT_EVERY = 500; // stream a POS update every N steps during a move
const int  STOP_BYTE    = 0x1B;// ESC = emergency stop

// ------------- state ------------------
long currentSteps = 0;         // absolute position, in steps from the 0" datum
bool known = false;            // has the user SET a starting position yet?
String buf;

// ------------- helpers ----------------
long   inchToSteps(double inch) { return lround(inch * STEPS_PER_INCH); }
double stepsToInch(long s)      { return s / STEPS_PER_INCH; }

void reportPos() {
  if (!known) { Serial.println("POS unknown -- use SET <inches> first"); return; }
  Serial.print("POS ");
  Serial.print(stepsToInch(currentSteps), 5);
  Serial.print(" in  (");
  Serial.print(currentSteps);
  Serial.println(" steps)");
}

// Steps the motor while updating currentSteps live and streaming position.
// Bails out early if a STOP_BYTE (ESC) arrives. Returns steps actually done.
long pulseSteps(int dirLevel, int sgn, long n) {
  digitalWrite(DIR_PIN, dirLevel);
  delayMicroseconds(5);        // let DIR settle
  long done = 0;
  for (long i = 0; i < n; i++) {
    if (Serial.available() && Serial.read() == STOP_BYTE) break;  // emergency stop
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_LOW_US);
    currentSteps += sgn;       // keep position honest, one step at a time
    done++;
    if (done % REPORT_EVERY == 0) reportPos();   // live stream during the move
  }
  return done;
}

void doSet(double inch) {
  if (inch < MIN_INCH || inch > MAX_INCH) { Serial.println("ERR position out of range 0..1"); return; }
  currentSteps = inchToSteps(inch);
  known = true;
  Serial.println("OK set");
  reportPos();
}

void doGoto(double inch) {
  if (!known)                             { Serial.println("ERR position unknown -- SET first"); return; }
  if (inch < MIN_INCH || inch > MAX_INCH) { Serial.println("ERR target out of range 0..1");      return; }

  long target = inchToSteps(inch);
  long delta  = target - currentSteps;
  if (delta == 0) { Serial.println("OK already there"); reportPos(); return; }

  int  dir = (delta > 0) ? DIR_POS : DIR_NEG;
  int  sgn = (delta > 0) ? 1 : -1;
  long mag = (delta > 0) ? delta : -delta;

  long done = pulseSteps(dir, sgn, mag);
  Serial.println(done < mag ? "STOPPED" : "OK moved");
  reportPos();                 // final position (accurate even after a stop)
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String up = line; up.toUpperCase();
  if      (up.startsWith("SET"))  doSet(line.substring(3).toFloat());
  else if (up.startsWith("GOTO")) doGoto(line.substring(4).toFloat());
  else if (up.startsWith("POS"))  reportPos();
  else if (up.startsWith("HELP")) Serial.println("Commands: SET <in> | GOTO <in> | POS | ESC=stop");
  else                            Serial.println("ERR unknown -- type HELP");
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_POS);
  Serial.println("Stage ready. SET <in>, then GOTO <in>. ESC = stop.");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == STOP_BYTE)         { buf = ""; continue; }   // stray ESC when idle: ignore
    if (c == '\n' || c == '\r') { handleLine(buf); buf = ""; }
    else                        { buf += c; }
  }
}