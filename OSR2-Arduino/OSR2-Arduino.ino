// OSR-Release v2.5,
// by TempestMAx 23-7-20
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control servos and vibration motors
// Can handle three linear channels (L0, L1, L2), three rotation channels (R0, R1, R2) 
// and two vibration channels (V0, V1)
// This code is designed to drive the OSR series of robot, but is also intended to be
// used as a template to be adapted to run other t-code controlled arduino projects
// Have fun, play safe!
// History:
// v2.0 - TCode v0.2 compatible, 28-1-2020
// v2.1 - OSR2 release, 1-2-2020
// v2.2 - OSR2+ release, 1-3-2020
// v2.3 - T-Valve support added, 1-5-2020
// v2.4 - T-wist support added; LR servos now +/- 350 for the sake of Raser1's sanity, 1-7-2020
// v2.5 - Experimental build. Servo Library replaced with alternative capable of high frequencies. 23-7-2020
// v2.6 - Experimental build. For use with Parallax Feedback 360° Servo (900-00360) in T-wist. 23-9-2020

// ----------------------------
//   Settings
// ----------------------------

// Servo operating frequency
// (recommend 250 Hz or less)
#define Servo_FREQ 250  // (Hz)

// Pin assignments
// T-wist feedback goes on digital pin 2
#define Servo1_PIN 8  // Right Servo (change to 7 for Romeo v1.1)
#define Servo2_PIN 3  // Left Servo (change to 4 for Romeo v1.1)
#define Servo3_PIN 9  // Pitch Servo (change to 8 for Romeo v1.1)
#define Servo4_PIN 12  // Valve Servo
#define Servo5_PIN 10  // Twist Servo
#define Vibe0_PIN 5   // Vibration motor 1
#define Vibe1_PIN 6   // Vibration motor 2

// Arm servo zeros
// Change these to adjust arm positions
// (1500 = centre)
#define Servo0_ZERO 1500  // Fore-Aft Servo (OSR3 only)
#define Servo1_ZERO 1500  // Right Servo
#define Servo2_ZERO 1500  // Left Servo
#define Servo3_ZERO 1500  // Pitch Servo



// ----------------------------
//   Serial Comms Interface
// ----------------------------
// This is a t-code object that manages the serial comms from the computer
// Leave this section of the code alone unless you know what you're doing

class ToyComms {
  public:

    // Setup function
    ToyComms() {
      // Centralise everything
      xL1[0] = 500;
      xL1[1] = 500;
      xL1[2] = 500;
      xR1[0] = 500;
      xR1[1] = 500;
      xR1[2] = 500;
    }

    // Function to process serial input
    void serialRead(byte inByte) {
      switch (inByte) {

        // Start - Linear Motion input
        case 'l':
        case 'L':
          linear = true;
          vibration = false;
          rotation = false;
          device = false;
          inNum1 = 1;
          interval = false;
          velocity = false;
          inNum2 = 0;
        break;

        // Start - Vibration input
        case 'v':
        case 'V':
          vibration = true;
          rotation = false;
          linear = false;
          device = false;
          inNum1 = 1;
          interval = false;
          velocity = false;
          inNum2 = 0;
        break;

        // Start - Rotation input
        case 'r':
        case 'R':
          rotation = true;
          linear = false;
          vibration = false;
          device = false;
          inNum1 = 1;
          interval = false;
          velocity = false;
          inNum2 = 0;
        break;

        // Start - Device input
        case 'd':
        case 'D':
          device = true;
          rotation = false;
          linear = false;
          vibration = false;
          inNum1 = 0;
          interval = false;
          velocity = false;
          inNum2 = 0;
        break;

        // Append Interval
        case 'i':
        case 'I':
          if (linear || vibration || rotation) {
            interval = true;
            velocity = false;
            inNum2 = 0;
          }
        break;

        // Append Velocity
        case 's':
        case 'S':
          if (linear || vibration || rotation) {
            velocity = true;
            interval = false;
            inNum2 = 0;
          }
        break;

        // Record Number
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
          if (interval || velocity) {
            // Update the number
            inNum2 = inNum2*10;
            if (inNum2 > 10000) {inNum2 = inNum2 % 10000;}
            inNum2 = inNum2 + (inByte - 48);
          // If L,V or R  
          } else if (linear || vibration || rotation || device) {
            // If less than 4 digits so far, update the number
            if (inNum1 < 10000) {
              inNum1 = inNum1*10;
              inNum1 = inNum1 + (inByte - 48);
            }
            
          }
        break;


        // If any other character
          default:
          // Has a command been input?
          if (linear || vibration || rotation) {

            // Check a channel and a number have been entered
            if (inNum1 < 100) {
              linear = false;
              vibration = false;
              rotation = false;
            }

            // Increase to 4 digits, if not entered
            while (inNum1 < 10000) {
              inNum1 = inNum1*10;
            }
            // Eliminate "1" marker
            inNum1 = inNum1 - 10000;
            
            // Extract commanded position "x" from last three digits
            int x,i;
            x = inNum1 % 1000;
            // Extract commanded axis "i" from first digit
            i = (inNum1 - x)/1000;
            i = i % 10;

            // If the commanded axis exists, process command
            if (0<=i && i<=2) {

              //If it's a linear command
              if (linear) {

                // Save axis command as 1-1000
                xLbuff1[i] = x+1;
                
                if (interval) {
                  // Time interval
                  xLbuff2[i] = inNum2;
                  xLbuffSpd[i] = false;
                } else if (velocity) {
                  // Speed
                  xLbuff2[i] = inNum2;
                  xLbuffSpd[i] = true;
                } else {
                  // Instant
                  xLbuff2[i] = 0;
                }
                
              }


              //If it's a linear command
              if (rotation) {

                // Save axis command as 1-1000
                xRbuff1[i] = x+1;
                
                if (interval) {
                  // Time interval
                  xRbuff2[i] = inNum2;
                  xRbuffSpd[i] = false;
                } else if (velocity) {
                  // Speed
                  xRbuff2[i] = inNum2;
                  xRbuffSpd[i] = true;
                } else {
                  // Instant
                  xRbuff2[i] = 0;
                }
                
              }


              //If it's a linear command
              if (vibration) {

                // Save axis command as 1-1000
                xVbuff1[i] = x+1;
                
                if (interval) {
                  // Time interval
                  xVbuff2[i] = inNum2;
                  xVbuffSpd[i] = false;
                } else if (velocity) {
                  // Speed
                  xVbuff2[i] = inNum2;
                  xVbuffSpd[i] = true;
                } else {
                  // Instant
                  xVbuff2[i] = 0;
                }
                
              }
              
            }
          } else if (device) {
            Dbuff = inNum1;
            device = false;
          }



          // Clear input buffer
          linear = false;
          vibration = false;
          rotation = false;
          device = false;
          inNum1 = 0;
          interval = false;
          velocity = false;
          inNum2 = 0;

          // If it's a new line character
          if (inByte == 10) {

            // Mark input time
            unsigned long t;
            t = millis();

            byte n;
            // Execute Linear Channels
            for (n = 0; n <= 2; n++) {
              if (xLbuff1[n]>0) {

                // Execute control command
                xL0[n] = xLinear(n,t);
                xL1[n] = xLbuff1[n];
                // Write the initial time
                tL0[n] = t;
                
                // Is the second number a speed
                if (xLbuffSpd[n]) {
                  // Speed
                  tL1[n] = xL1[n]-xL0[n];
                  if (tL1[n] < 0) {tL1[n] = -tL1[n];}
                  tL1[n] = tL0[n] + (1000*tL1[n])/xLbuff2[n];  
                } else {
                  // Time interval
                  tL1[n] = tL0[n] + xLbuff2[n];
                }
                // Smoothing limit
                if (tL1[n]-tL0[n] < 20) { tL1[n] = tL0[n] + 20; }
                // Clear channel buffer
                xLbuff1[n] = 0;
                xLbuff2[n] = 0;
                xLbuffSpd[n] = false;
                  
              }
            }

            // Execute Rotation Channels
            for (n = 0; n <= 2; n++) {
              if (xRbuff1[n]>0) {

                // Execute control command
                xR0[n] = xRotate(n,t);
                xR1[n] = xRbuff1[n];
                // Write the initial time
                tR0[n] = t;
                
                // Is the second number a speed
                if (xRbuffSpd[n]) {
                  // Speed
                  tR1[n] = xR1[n]-xR0[n];
                  if (tR1[n] < 0) {tR1[n] = -tR1[n];}
                  tR1[n] = tR0[n] + (1000*tR1[n])/xRbuff2[n];  
                } else {
                  // Time interval
                  tR1[n] = tR0[n] + xRbuff2[n];
                }
                // Smoothing limit
                if (tR1[n]-tR0[n] < 20) { tR1[n] = tR0[n] + 20; }
                // Clear channel buffer
                xRbuff1[n] = 0;
                xRbuff2[n] = 0;
                xRbuffSpd[n] = false;
                  
              }
            }

            // Execute Vibration Channels
            for (n = 0; n <= 1; n++) {
              if (xVbuff1[n]>0) {

                // Execute control command
                xV0[n] = xVibe(n,t);
                xV1[n] = xVbuff1[n];
                // Write the initial time
                tV0[n] = t;
                
                // Is the second number a speed
                if (xVbuffSpd[n]) {
                  // Speed
                  tV1[n] = xV1[n]-xV0[n];
                  if (tV1[n] < 0) {tV1[n] = -tV1[n];}
                  tV1[n] = tV0[n] + (1000*tV1[n])/xVbuff2[n];  
                } else {
                  // Time interval
                  tV1[n] = tV0[n] + xVbuff2[n];
                }
                // Clear channel buffer
                xVbuff1[n] = 0;
                xVbuff2[n] = 0;
                xVbuffSpd[n] = false;
                  
              }
            }

            // Execute device commands
            if (Dbuff > 0) {
              if (Dbuff == 1) {
                identifyTCode();
              }


              Dbuff = 0;
            }
            
          }
        
        
        break;
                    
      }
    }

    // Establish linear position from time (1-1000)
    int xLinear(int i,unsigned long t) {
      // i is axis
      // t is time point
      // x will be the return value
      int x;
      
      // Ramp value
      if (t > tL1[i]) {
        x = xL1[i];
      } else if (t < tL0[i]) {
        x = xL0[i];
      } else {
        x = map(t,tL0[i],tL1[i],xL0[i],xL1[i]);
      }
      
      return x;
    }
    
    // Establish rotation position from time (1-1000)
    int xRotate(int i,unsigned long t) {
      // i is axis
      // t is time point
      // x will be the return value
      int x;
      
      // Ramp value
      if (t > tR1[i]) {
        x = xR1[i];
      } else if (t < tR0[i]) {
        x = xR0[i];
      } else {
        x = map(t,tR0[i],tR1[i],xR0[i],xR1[i]);
      }
      
      return x;
    }

    // Establish vibration level from time (1-1000)
    int xVibe(int i,unsigned long t) {
      // i is level
      // t is time point
      // x will be the return value
      int x;
      
      // Ramp value
      if (t > tV1[i]) {
        x = xV1[i];
      } else if (t < tV0[i]) {
        x = xV0[i];
      } else {
        x = map(t,tV0[i],tV1[i],xV0[i],xV1[i]);
      }
      
      return x;
    }

    // Function to identify the current TCode type over serial
    void identifyTCode() {
      Serial.println("TCode v0.2");
    }

  private:

    // Input parameters
    boolean linear;
    boolean vibration;
    boolean rotation;
    boolean device;
    int inNum1;
    boolean interval;
    boolean velocity;
    int inNum2;

    // Linear motion
    int xLbuff1[3];
    int xLbuff2[3];
    boolean xLbuffSpd[3];
    int xL0[3];
    int xL1[3];
    long tL0[3];
    long tL1[3];

    // Rotation
    int xRbuff1[3];
    int xRbuff2[3];
    boolean xRbuffSpd[3];
    int xR0[3];
    int xR1[3];
    long tR0[3];
    long tR1[3];

    // Vibration
    int xVbuff1[2];
    int xVbuff2[2];
    boolean xVbuffSpd[2];
    int xV0[2];
    int xV1[2];
    long tV0[2];
    long tV1[2];

    // Device commands
    int Dbuff;
    
};





// ----------------------------
//   Servo Interface
// ----------------------------
// This is a replacement for the standard arduino servo library that allows much
// more control over when and how often the servo pulses are sent

class TServo {
  public:

    TServo(){
    }

    // Function to register servos attached to pins
    void Attach(int pinNumber) {
      hasServo[pinNumber] = true;
      pulseLength[pinNumber] = 1500;
      switch (pinNumber) {
        case 2:  DDRD |= B00000100; break;
        case 3:  DDRD |= B00001000; break;
        case 4:  DDRD |= B00010000; break;
        case 5:  DDRD |= B00100000; break;
        case 6:  DDRD |= B01000000; break;
        case 7:  DDRD |= B10000000; break;
        case 8:  DDRB |= B00000001; break;
        case 9:  DDRB |= B00000010; break;
        case 10: DDRB |= B00000100; break;
        case 11: DDRB |= B00001000; break;
        case 12: DDRB |= B00010000; break;
        case 13: DDRB |= B00100000; break;
      }
      
    }

    // Function to register servos removed from pins
    void Detach(int pinNumber) {
      hasServo[pinNumber] = false;
      switch (pinNumber) {
        case 2:  DDRD &= B00000100; break;
        case 3:  DDRD &= B00001000; break;
        case 4:  DDRD &= B00010000; break;
        case 5:  DDRD &= B00100000; break;
        case 6:  DDRD &= B01000000; break;
        case 7:  DDRD &= B10000000; break;
        case 8:  DDRB &= B00000001; break;
        case 9:  DDRB &= B00000010; break;
        case 10: DDRB &= B00000100; break;
        case 11: DDRB &= B00001000; break;
        case 12: DDRB &= B00010000; break;
        case 13: DDRB &= B00100000; break;
      }
      
    }

    // Function to set the pulse length for a specified servo
    // (0 = no Max frequency)
    void SetMaxFreq(int pinNumber,int setFreq) {
      if (setFreq == 0) {
        minInterval[pinNumber] = 0;
      } else {
        minInterval[pinNumber] = 1000000/setFreq;
      }

    }

    // Function to set the pulse length for a specified servo
    void SetMicroseconds(int pinNumber,int setPulseLength) {
      pulseLength[pinNumber] = constrain(setPulseLength,500,2500);
    }


    // Function to execute a parallel servo pulse
    void Execute() {

      // Identify active pins to drive
      unsigned long timeNow;
      boolean pinActive[14];
      timeNow = micros();
      for (int i=0; i<14; i++) {
        if (hasServo[i] && ((minInterval[i] == 0) || (timeNow > (pinTimeOn[i] + minInterval[i])))) {
        //if (hasServo[i] && ((timeNow > (pinTimeOn[i] + minInterval[i])))) {
          servoOrder[i] = i;
          pinActive[i] = true;
        } else {
          servoOrder[i] = 0;
          pinActive[i] = false;
        }
      }

      // Sort pins into order of pulse length
      int copy;
      for (int j=13; j>0; j--) {
        for (int i=13; i>13-j; i--) {
          if (( servoOrder[i] == 0 && servoOrder[i-1] > 0 ) || (servoOrder[i]>0 && servoOrder[i-1]>0 && pulseLength[servoOrder[i]] < pulseLength[servoOrder[i-1]])){
            copy = servoOrder[i-1];
            servoOrder[i-1] = servoOrder[i];
            servoOrder[i] = copy;
          }
        } 
      }

      // Log start time and turn on pins
      byte dMod,bMod;
      dMod = 0;
      if (pinActive[2]) {dMod |= B00000100;}
      if (pinActive[3]) {dMod |= B00001000;}
      if (pinActive[4]) {dMod |= B00010000;}
      if (pinActive[5]) {dMod |= B00100000;}
      if (pinActive[6]) {dMod |= B01000000;}
      if (pinActive[7]) {dMod |= B10000000;}
      bMod = 0;
      if (pinActive[8]) {bMod |= B00000001;}
      if (pinActive[9]) {bMod |= B00000010;}
      if (pinActive[10]) {bMod |= B00000100;}
      if (pinActive[11]) {bMod |= B00001000;}
      if (pinActive[12]) {bMod |= B00010000;}
      if (pinActive[13]) {bMod |= B00100000;}   
      timeNow = micros();
      PORTD |= dMod;
      PORTB |= bMod;        

      // Calculate pin off times
      for (int i = 0; i<14; i++) {
        if (pinActive[i]) {
          pinTimeOff[i] = timeNow + pulseLength[i];
          pinTimeOn[i] = timeNow;
        }
      }

      // Watch time and turn servo pins off in sequence
      unsigned long timeOff;
      for (int i = 0; i<14; i++) {
        if (servoOrder[i]>0) {
          timeOff = pinTimeOff[servoOrder[i]];
          while(timeNow < timeOff) {
            timeNow = micros();
          }
          pinOff(servoOrder[i]);
        }
      }
       
    }
    

  private:

    // Function to turn off a specified pin
    void pinOff(int pinNumber) {
      switch (pinNumber) {
        case 2:  PORTD &= B11111011; break;
        case 3:  PORTD &= B11110111; break;
        case 4:  PORTD &= B11101111; break;
        case 5:  PORTD &= B11011111; break;
        case 6:  PORTD &= B10111111; break;
        case 7:  PORTD &= B01111111; break;
        case 8:  PORTB &= B11111110; break;
        case 9:  PORTB &= B11111101; break;
        case 10: PORTB &= B11111011; break;
        case 11: PORTB &= B11110111; break;
        case 12: PORTB &= B11101111; break;
        case 13: PORTB &= B11011111; break;
      }
      
    }

    boolean hasServo[14];            // Which pins have servos on
    unsigned long minInterval[14];   // This is the minimum time between starting servo pulses [microsec]
    int pulseLength[14];             // Length of pulses to send [microsec]
    int servoOrder[14];              // Index order in which servo pulses are to be turned off
    unsigned long pinTimeOn[14];     // Time at which pins were turned on [microsec]
    unsigned long pinTimeOff[14];    // Time at which pins are to be turned off [microsec]

};





// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above
ToyComms toy;
TServo servos; 

// Declare timing variables
unsigned long nextPulse;
int tick;

// Position variables
int xLin,yLin,zLin;
// Rotation variables
int xRot,yRot,zRot;
// Vibration variables
int vibe0,vibe1;
// Velocity tracker variables, for T-Valve
int xLast;
float xValve;
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

  // Start serial
  Serial.begin(115200);
  toy.identifyTCode();

  // Declare servos and set zero
  servos.Attach(Servo1_PIN);
  servos.Attach(Servo2_PIN);
  servos.Attach(Servo3_PIN);
  servos.Attach(Servo4_PIN);
  servos.SetMaxFreq(Servo4_PIN,50);
  servos.Attach(Servo5_PIN);
  servos.SetMaxFreq(Servo5_PIN,50);

  // Set vibration PWM pins
  pinMode(Vibe0_PIN,OUTPUT);
  pinMode(Vibe1_PIN,OUTPUT);
  // Test vibration channels
  analogWrite(Vibe0_PIN,127);
  delay(300);
  analogWrite(Vibe0_PIN,0);
  analogWrite(Vibe1_PIN,127);
  delay(300);
  analogWrite(Vibe1_PIN,0);

  // Set servo pulse interval
  tick = 1000/Servo_FREQ; //ms
  // Set time for first pulse
  nextPulse = millis() + tick;

  // Velocity tracker, for T-Valve
  xLast = 500;
  xValve = 0;

  // Initiate position tracking for twist
  attachInterrupt(0, twistRising, RISING);

  // Signal done
  Serial.println("Ready!");

}




// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously

void loop() {

  // Read serial
  // This will run continuously
  if (Serial.available() > 0) {
    // Send the serial bytes to the t-code object
    // This is the only required input for the object
    toy.serialRead(Serial.read());
  }

  // Pulse Servos based on time interval
  // This function will run every 20ms, sending a pulse to the servos
  if (millis() > nextPulse) {
    unsigned long t = nextPulse;
    nextPulse = nextPulse + tick;

    // Collect inputs
    // These functions query the t-code object for the position/level at a specified time
    // Number recieved will be an integer, 1-1000
    xLin = toy.xLinear(0,t);
    //yLin = toy.xLinear(1,t); (not used)
    //zLin = toy.xLinear(2,t); (not used)
    xRot = toy.xRotate(0,t);
    yRot = toy.xRotate(1,t);
    zRot = toy.xRotate(2,t);
    vibe0 = toy.xVibe(0,t);
    vibe1 = toy.xVibe(1,t);

    // If you want to mix your servos differently, enter your code below:

    // Calculate valve position
    float Vel,ValveCmd,suck;
    Vel = xLin - xLast;
    Vel = 50*Vel/tick;
    xLast = xLin;
    suck = 20;
    if (Vel > suck) {
      ValveCmd = Vel-suck;
    } else if (Vel < 0){
      ValveCmd = -Vel;
    } else {
      ValveCmd = 0;
    }
    xValve = (4*xValve + ValveCmd)/5;

    // Calculate twist position
    float dutyCycle = twistPulseLength;
    dutyCycle = dutyCycle/twistPulseCycle;
    float angPos = (dutyCycle - 0.029)/0.942;
    angPos = constrain(angPos,0,1) - 0.5;
    if (angPos - twistServoAngPos < - 0.8) { twistTurns += 1; }
    if (angPos - twistServoAngPos > 0.8) { twistTurns -= 1; }
    twistServoAngPos = angPos;
    twistPos = 1000*(angPos + twistTurns);
    

    // Mix and send servo channels
    // Linear scale inputs to servo appropriate numbers
    int stroke,fwd,roll,pitch,valve,twist;
    stroke = map(xLin,1,1000,-350,350);
    roll   = map(yRot,1,1000,-180,180);
    pitch  = map(zRot,1,1000,-350,350);
    valve  = 20*xValve;
    valve  = constrain(valve, 0, 1000);
    twist  = 2*(xRot - map(twistPos,-1500,1500,1000,1));
    twist  = constrain(twist, -750, 750);
    //Serial.println(map(twistPos,-1400,1400,1,1000));
    
    // Send signals to the servos
    // Note: 1000 = -45deg, 2000 = +45deg
    servos.SetMicroseconds(Servo1_PIN, Servo1_ZERO + stroke + roll);
    servos.SetMicroseconds(Servo2_PIN, Servo2_ZERO - stroke + roll);
    servos.SetMicroseconds(Servo3_PIN, Servo3_ZERO - pitch);
    servos.SetMicroseconds(Servo4_PIN, 2000 - valve);
    servos.SetMicroseconds(Servo5_PIN, 1500 + twist);
    servos.Execute();

    // Done with servo channels

    // Output vibration channels
    // These should drive PWM pins connected to vibration motors via MOSFETs or H-bridges.
    if ((vibe0 > 1) && (vibe0 <= 1000)) {
      analogWrite(Vibe0_PIN,map(vibe0,2,1000,31,255));
    } else {
      analogWrite(Vibe0_PIN,0);
    }
    if ((vibe1 > 1) && (vibe1 <= 1000)) {
      analogWrite(Vibe1_PIN,map(vibe1,2,1000,31,255));
    } else {
      analogWrite(Vibe1_PIN,0);
    }

    // Done with vibration channels
    
  }


}

// Twist position detection functions
void twistRising() {
  attachInterrupt(0, twistFalling, FALLING);
  twistPulseCycle = micros()-twistPulseStart;
  twistPulseStart = micros();
}
void twistFalling() {
  attachInterrupt(0, twistRising, RISING);
  twistPulseLength = micros()-twistPulseStart;
}
