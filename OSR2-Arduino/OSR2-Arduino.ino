// OSR-Release v3.6
// by TempestMAx 9-11-23
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control servos and vibration motors
// It can handle:
//   3x linear channels (L0, L1, L2)
//   3x rotation channels (R0, R1, R2) 
//   3x vibration channels (V0, V1, V2)
//   3x auxilliary channels (A0, A1, A2)
// This code is designed to drive the OSR2 stroker robot, but is also intended to be
// used as a template to be adapted to run other t-code controlled arduino projects
// Have fun, play safe!
// History:
// v3.0 - TCode v0.3 compatible, 8-5-2021
// v3.1 - Buffer overload bug fix by limiting to 3x4 T-Code channels, axis auto-smoothing for live commands added 7-7-2021
// v3.2 - Range preference variable storage bug fix, 24-8-21
// v3.3 - Low speed vibration channel start function added, 3-11-21
// v3.4 - Support for T-wist 4 added - standard servo now default, parallax is an option, 5-3-22
// v3.5 - TCode library moved into a separate file. Buttons functionality added. 4-11-23
// v3.6 - Buttons changed from Prev/Play/Next to left/ok/right 9-11-23


// ----------------------------
//  User Settings
// ----------------------------
// These are the setup parameters for an OSR2 on a Romeo BLE mini v2

// Device IDs, for external reference
#define FIRMWARE_ID "OSR2-Arduino Release v3.6"  // Device and firmware version
#define TCODE_VER "TCode v0.3"  // Current version of TCode

// Pin assignments
// T-wist feedback goes on digital pin 2
#define LeftServo_PIN 8    // Left Servo (change to 7 for Romeo v1.1)
#define RightServo_PIN 3   // Right Servo (change to 4 for Romeo v1.1)
#define PitchServo_PIN 9   // Pitch Servo (change to 8 for Romeo v1.1)
#define TwistServo_PIN 10  // Twist Servo
#define ValveServo_PIN 12  // Valve Servo
#define Vibe0_PIN 5        // Vibration motor 1
#define Vibe1_PIN 6        // Vibration motor 2

// Arm servo zeros
// Change these to adjust servo centre positions
// (1500 = centre, 1 complete step = 160)
#define LeftServo_ZERO 1500   // Right Servo
#define RightServo_ZERO 1500  // Left Servo
#define PitchServo_ZERO 1500  // Pitch Servo
#define TwistServo_ZERO 1500  // Twist Servo
#define ValveServo_ZERO 1500  // Valve Servo

#define USE_BUTTONS true      // Use a Pre-Play-Next-Edge resistor ladder
#define Buttons_PIN 14        // Buttons on analog pin A0 (== pin 14)
#define USE_EDGEBUTTON true  // Use a Pre-Play-Next-Edge resistor ladder
#define EdgeButton_PIN 15     // Buttons on analog pin A1 (== pin 15)

// Other functions
#define TWIST_PARALLAX false      // (true/false) Parallax 360 feedback servo on twist (t-wist3)
#define REVERSE_TWIST_SERVO false // (true/false) Reverse twist servo direction 
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999) 
#define REVERSE_VALVE_SERVO true  // (true/false) Reverse T-Valve direction
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define Lube_PIN 13               // Lube manual input button pin (Connect pin to +5V for ON)
#define Lube_SPEED 255            // Lube pump speed (0-255)
#define MIN_SMOOTH_INTERVAL 3     // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL 100   // Maximum auto-smooth ramp interval for live commands (ms)

// Libraries used
#include <Servo.h>  // Standard Arduino servo library
#include "TCode.h"  // Tempest's TCode library
#include "TButtons.h"  // Tempest's TCode library


// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above
TCode tcode(FIRMWARE_ID, TCODE_VER);

TButton edgeButton("edge",10);
TButton rightButton("right",10);
TButton okButton("ok",10);
TButton leftButton("left",10);

// Declare servos
Servo LeftServo;
Servo RightServo;
Servo PitchServo;
Servo TwistServo;
Servo ValveServo;

// Declare operating variables
// Position variables
int xLin,yLin,zLin;
// Rotation variables
int xRot,yRot,zRot;
// Vibration variables
int vibe0,vibe0set,vibe1,vibe1set;
// Lube variables
int lube;
// Valve variables
int valveCmd,suckCmd;
// Velocity tracker variables, for valve
int xLast;
unsigned long tLast;
float upVel,valvePos;
// Twist position monitor variables
volatile int twistPulseLength = 0;
volatile int twistPulseCycle = 1099;
volatile int twistPulseStart = 0;
float twistServoAngPos = 0.5;
int twistTurns = 0;
float twistPos;

// Setup function
// This is run once, when the arduino starts
void setup() {

  // Start serial connection and report status
  Serial.begin(115200);
  tcode.StringInput("D0");
  tcode.StringInput("D1");

  // Register device axes
  tcode.RegisterAxis("L0", "Up");
  //tcode.RegisterAxis("L1", "Forward"); (not used)
  //tcode.RegisterAxis("L2", "Left"); (not used)
  tcode.RegisterAxis("R0", "Twist");
  tcode.RegisterAxis("R1", "Roll");
  tcode.RegisterAxis("R2", "Pitch");
  tcode.RegisterAxis("V0", "Vibe0");
  if (!LUBE_V1) { tcode.RegisterAxis("V1", "Vibe1"); }
  tcode.RegisterAxis("A0", "Valve");
  tcode.RegisterAxis("A1", "Suck");
  tcode.AxisInput("A1",VALVE_DEFAULT,'I',3000);
  if (LUBE_V1) {
    tcode.RegisterAxis("A2", "Lube");
    tcode.AxisInput("A2",0,' ',0);
    pinMode(Lube_PIN,INPUT);
  }

  // Attach servos
  LeftServo.attach(LeftServo_PIN);
  RightServo.attach(RightServo_PIN);
  PitchServo.attach(PitchServo_PIN);
  TwistServo.attach(TwistServo_PIN);
  ValveServo.attach(ValveServo_PIN);

  // Set vibration PWM pins
  pinMode(Vibe0_PIN,OUTPUT);
  vibe0set = 0;
  pinMode(Vibe1_PIN,OUTPUT);
  vibe1set = 0;

  // If T-wist3, initiate position tracking for twist
  if (TWIST_PARALLAX) { attachInterrupt(0, twistRising, RISING); }

  if (USE_EDGEBUTTON) { pinMode(15, INPUT_PULLUP); }
  
  // Signal done
  Serial.println("Ready!");

}


// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop() {

  // Read serial and send to tcode class
  while (Serial.available() > 0) {
    // Send the serial bytes to the t-code object
    tcode.ByteInput(Serial.read());
  }

  // Read and update button states
  bool edge = false;
  if (USE_BUTTONS) {
    int buttons = analogRead(Buttons_PIN);
    if (buttons > 1013) { edge = true; }
    if (buttons > 501 && buttons < 521 ) { rightButton.update(true); } else { rightButton.update(false); }
    if (buttons > 331 && buttons < 351) { okButton.update(true); } else { okButton.update(false); }
    if (buttons > 245 && buttons < 265) { leftButton.update(true); } else { leftButton.update(false); }
  }
  if (USE_EDGEBUTTON) {
    if (!digitalRead(EdgeButton_PIN)) { edge = true; }
  }
  if (edge) {edgeButton.update(true);} else { edgeButton.update(false); }

  // Collect inputs
  // These functions query the t-code object for the position/level at a specified time
  // Number recieved will be an integer, 0-9999
  xLin = tcode.AxisRead("L0");
  //yLin = tcode.AxisRead("L1"); (not used)
  //zLin = tcode.AxisRead("L2"); (not used)
  xRot = tcode.AxisRead("R0");
  yRot = tcode.AxisRead("R1");
  zRot = tcode.AxisRead("R2");
  vibe0 = tcode.AxisRead("V0");
  if (!LUBE_V1) { vibe1 = tcode.AxisRead("V1"); }
  valveCmd = tcode.AxisRead("A0");
  suckCmd = tcode.AxisRead("A1");
  if (LUBE_V1) { lube = tcode.AxisRead("A2"); }

  // If you want to mix your servos differently, enter your code below:

  // If t-wist3, calculate twist position
  if (TWIST_PARALLAX) { 
    float dutyCycle = twistPulseLength;
    dutyCycle = dutyCycle/twistPulseCycle;
    float angPos = (dutyCycle - 0.029)/0.942;
    angPos = constrain(angPos,0,1) - 0.5;
    if (angPos - twistServoAngPos < - 0.8) { twistTurns += 1; }
    if (angPos - twistServoAngPos > 0.8) { twistTurns -= 1; }
    twistServoAngPos = angPos;
    twistPos = 1000*(angPos + twistTurns);
  }

  // Calculate valve position
  // Track receiver velocity
  unsigned long t = millis();
  float upVelNow;
  if (t > tLast) {
    upVelNow = xLin - xLast;
    upVelNow /= t - tLast;
    upVel = (upVelNow + 9*upVel)/10;
  }
  tLast = t;
  xLast = xLin;
  // Use suck command if most recent
  boolean suck;
  if (tcode.AxisLast("A1") >= tcode.AxisLast("A0")) {
    suck = true;
    valveCmd = suckCmd;
  } else {
    suck = false;
  }
  // Set valve position
  if (suck) {
    if (upVel < -5) {
      valveCmd = 0;  
    } else if ( upVel < 0 ) {
      valveCmd = map(100*upVel,0,-500,suckCmd,0);
    }
  }
  valvePos = (9*valvePos + map(valveCmd,0,9999,0,1000))/10;

  // Mix and send servo channels
  // Linear scale inputs to servo appropriate numbers
  int stroke,roll,pitch,valve,twist;
  stroke = map(xLin,0,9999,-350,350);
  roll   = map(yRot,0,9999,-180,180);
  pitch  = map(zRot,0,9999,-350,350);
  if (TWIST_PARALLAX) { 
    twist  = (xRot - map(twistPos,-1500,1500,9999,0))/5;
    twist  = constrain(twist, -750, 750);
  } else {
    twist  = map(xRot,0,9999,1000,-1000);
    if (REVERSE_TWIST_SERVO) { twist = -twist; }
  }
  valve  = valvePos -500;
  valve  = constrain(valve, -500, 500);
  if (REVERSE_VALVE_SERVO) { valve = -valve; }

  // Set servo output values
  // Note: 1000 = -45deg, 2000 = +45deg
  LeftServo.writeMicroseconds(LeftServo_ZERO + stroke + roll);
  RightServo.writeMicroseconds(RightServo_ZERO - stroke + roll);
  PitchServo.writeMicroseconds(PitchServo_ZERO - pitch);
  TwistServo.writeMicroseconds(TwistServo_ZERO + twist);
  ValveServo.writeMicroseconds(ValveServo_ZERO + valve);
  // Done with servo channels


  // Output vibration channels
  // These should drive PWM pins connected to vibration motors via MOSFETs or H-bridges.
  // Vibe 0 channel
  if (vibe0 <= 0) {
    vibe0set = 0;
  } else if (vibe0set == 0 && vibe0 > 0) {
    vibe0set = 5000;
  } else if (vibe0set > vibe0 + 10) {
    vibe0set -= 10;
  } else {
    vibe0set = vibe0;
  }
  if (vibe0set > 0 && vibe0set <= 9999) {
    analogWrite(Vibe0_PIN,map(vibe0set,1,9999,63,255));
  } else {
    analogWrite(Vibe0_PIN,0);
  }
  // Vibe 1 channel
  if (vibe1 <= 0) {
    vibe1set = 0;
  } else if (vibe1set == 0 && vibe1 > 0) {
    vibe1set = 5000;
  } else if (vibe1set > vibe1 + 10) {
    vibe1set -= 10;
  } else {
    vibe1set = vibe1;
  }
  if (!LUBE_V1 && vibe1 > 0 && vibe1 <= 9999) {
    analogWrite(Vibe1_PIN,map(vibe1,1,9999,63,255));
  } else {
    analogWrite(Vibe1_PIN,0);
  }
  // Vibe timeout functions - shuts the vibe channels down if not commanded for a specified interval
  if (millis() - tcode.AxisLast("V0") > VIBE_TIMEOUT) { tcode.AxisInput("V0",0,'I',500); }
  if (!LUBE_V1 && millis() - tcode.AxisLast("V1") > VIBE_TIMEOUT) { tcode.AxisInput("V1",0,'I',500); }
  
  // Done with vibration channels

  // Lube functions
  if (LUBE_V1) {
    if (lube > 0 && lube <= 9999) {
      analogWrite(Vibe1_PIN,map(lube,1,9999,127,255));
    } else if (digitalRead(Lube_PIN) == HIGH) {
      analogWrite(Vibe1_PIN,Lube_SPEED);
    } else { 
      analogWrite(Vibe1_PIN,0);
    }
    if (millis() - tcode.AxisLast("A2") > 500) { tcode.AxisInput("A2",0,' ',0); } // Auto cutoff
  }

  // Done with lube


}


// T-wist3 parallax position detection functions
void twistRising() {
  attachInterrupt(0, twistFalling, FALLING);
  twistPulseCycle = micros()-twistPulseStart;
  twistPulseStart = micros();
}
void twistFalling() {
  attachInterrupt(0, twistRising, RISING);
  twistPulseLength = micros()-twistPulseStart;
}
