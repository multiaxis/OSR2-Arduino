// OSR3-Release v1.1,
// by TempestVR 15-7-19
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control servos and vibration motors
// Can handle three linear channels (L0, L1, L2) and two vibration channels (V0, V1)
// This code is designed to drive the OSR-3 fleshlight robot, but is designed to be
// used as a template to be adapted to run other t-code controlled arduino projects

// Libraries to include
#include <Servo.h>


// ----------------------------
//   Serial Comms Interface
// ----------------------------
// This is a t-code object that manages the serial comms from the computer
// Leave this section of the code alone unless you know what you're doing

class ToyComms {
  public:

    // Setup function
    ToyComms() {
      xL1[0] = 500;
      xL1[1] = 500;
      xL1[2] = 500;
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
          inNum1 = 0;
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
          inNum1 = 0;
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
          } else if (linear || vibration || rotation) {
            // Update the number
            inNum1 = inNum1*10;
            if (inNum1 > 999) {inNum1 = inNum1 % 1000;}
            inNum1 = inNum1 + (inByte - 48);
          }
        break;


        // If any other character
          default:
          // Has a command been input?
          if (linear || vibration || rotation) {
            
            // Extract commanded position "x" from last two digits
            int x,i;
            x = inNum1 % 100;
            // Extract commanded axis "i" from third digit
            i = (inNum1 - x)/100;
            i = i % 10;

            // If the commanded axis exists, process command
            if (0<=i && i<=2) {

              //If it's a linear command
              if (linear) {

                // Save axis command as 1-100
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
              if (vibration) {

                // Save axis command as 1-100
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
          }



          // Clear input buffer
          linear = false;
          vibration = false;
          rotation = false;
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
                xL1[n] = map(xLbuff1[n],1,100,0,1000);
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
                // Clear channel buffer
                xLbuff1[n] = 0;
                xLbuff2[n] = 0;
                xLbuffSpd[n] = false;
                  
              }
            }

            // Execute Vibration Channels
            for (n = 0; n <= 1; n++) {
              if (xVbuff1[n]>0) {

                // Execute control command
                xV0[n] = xVibe(n,t);
                xV1[n] = map(xVbuff1[n],1,100,0,1000);
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
            
          }
        
        
        break;
                    
      }
    }

    // Establish linear position from time (0-1000)
    int xLinear(int i,long t) {
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

    // Establish vibration level from time (0-1000)
    int xVibe(int i,long t) {
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


  private:

    // Input parameters
    boolean linear;
    boolean vibration;
    boolean rotation;
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

    // Vibration
    int xVbuff1[2];
    int xVbuff2[2];
    boolean xVbuffSpd[2];
    int xV0[2];
    int xV1[2];
    long tV0[2];
    long tV1[2];
    
};






// ----------------------------
//   SETUP
// ----------------------------

// Declare class
// This uses the t-code object above
ToyComms toy; 

// Declare servos
Servo Servo0;  // Fore-Aft Servo
Servo Servo1;  // Right Servo
Servo Servo2;  // Left Servo

// Specify which pins are attached to what here
#define Servo0_PIN 8  // Fore-Aft Servo
#define Servo1_PIN 2  // Right Servo
#define Servo2_PIN 3  // Left Servo
#define Vibe0_PIN 5   // Vibration motor 1
#define Vibe1_PIN 6   // Vibration motor 2

// Declare timing variables
unsigned long nextPulse;
int tick;

// Position variables
int xLin,yLin,zLin;
// Vibration variables
int vibe0,vibe1;

// Setup function
// This is run once, when the arduino starts
void setup() {

  // Start serial
  Serial.begin(9600);
  Serial.println("Setup...");

  // Declare servos and set zero
  Servo0.attach(Servo0_PIN);
  Servo1.attach(Servo1_PIN);
  Servo2.attach(Servo2_PIN);
  delay(500);
  Servo0.writeMicroseconds(1500);
  Servo1.writeMicroseconds(1500);
  Servo2.writeMicroseconds(1500);

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
  tick = 20; //ms
  // Set time for first pulse
  nextPulse = millis() + tick;

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
    nextPulse = nextPulse + tick;

    // Output time index
    long t;
    t = nextPulse - tick;

    // Collect inputs
    // These functions query the t-code object for the position/level at a specified time
    // Number recieved will be an integer, 0-1000
    zLin = toy.xLinear(0,t);
    xLin = toy.xLinear(1,t);
    yLin = toy.xLinear(2,t);
    vibe0 = toy.xVibe(0,t);
    vibe1 = toy.xVibe(1,t);

    // If you want to control your servos differently, enter your code below:
    
    // Forward-Backward compensation
    // This calculates platform movement to account for the arc of the servos
    float lin1,lin2;
    int b2;
    lin1 = zLin-500;
    lin1 = lin1*0.00157079632;
    lin2 = 0.853-cos(lin1);
    lin2 = 1133*lin2;
    b2 = lin2;

    // Mix and send servo channels
    // Linear scale inputs to servo appropriate numbers
    int a,b,c;
    a = map(zLin,0,1000,100,900);
    b = map(xLin,0,1000,-250,250);
    c = map(yLin,0,1000,-250,250);
    // Send signals to the servos
    // Note: 1000 = -45deg, 2000 = +45deg
    Servo0.writeMicroseconds(1500 + b - b2);
    Servo1.writeMicroseconds(1000 + a - c);
    Servo2.writeMicroseconds(2000 - a - c);

    // Done with servo channels

    // Output vibration channels
    // These should drive PWM pins connected to vibration motors via MOSFETs or H-bridges.
    if ((vibe0 > 0) && (vibe0 <= 1000)) {
      analogWrite(Vibe0_PIN,map(vibe0,0,1000,63,255));
    } else {
      analogWrite(Vibe0_PIN,0);
    }
    if ((vibe1 > 0) && (vibe1 <= 1000)) {
      analogWrite(Vibe1_PIN,map(vibe1,0,1000,63,255));
    } else {
      analogWrite(Vibe1_PIN,0);
    }

    // Done with vibration channels
    
  }


}
