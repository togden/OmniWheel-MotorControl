#include <SoftwareSerial.h>
//#include <Servo.h>

////////////////////////////
// Define the Motor Pins //
////////////////////////////

// Front and Rear Motors - These drive the robot left and right
// These pins are used to control direction - Must be Digital Pins
#define rearMotorLeft 22
#define rearMotorRight 24
#define frontMotorLeft 26
#define frontMotorRight 28

// Left and Right Motors - These drive the robot forward and reverse
// These pins are used to control direction  - Must be Digital Pins
#define rightMotorForward 30
#define rightMotorReverse 32
#define leftMotorForward 34
#define leftMotorReverse 36

// These pins are used to control speed - Must be PWM pins
#define rearMotorSpeed 6
#define frontMotorSpeed 7
#define rightMotorSpeed 8
#define leftMotorSpeed 9

//////////////////////////
// Define the xBee Pins //
//////////////////////////

// XBee shield be set to DLINE to use D2 and D3
// Place a jumper wire from pin D2 to pin D11!

int pinRx = 11, pinTx = 3; // pins on Arduino
long BaudRate = 9600 , sysTick = 0;

///////////////////////////////////
// Initialize XBee Communication //
///////////////////////////////////
SoftwareSerial mySerial( pinRx , pinTx );

//////////////////////////////
// Initialize Servo Control //
//////////////////////////////
//Servo myServo;
//int servoPosition;

//////////////////////
// Define Variables //
//////////////////////
int commands[4];
double xPosition;
double yPosition;
double xScaled;
double yScaled;
double xMapped;
double yMapped;
int xSpeed;
int ySpeed;
int xDirection;
int yDirection;
int launcherSpeed;
int launcherPin = 12;
int maxSpeed = 128;
int minSpeed = 30;
int ballSensor;
int sensorPin = 5;
bool ballInHopper = false;
byte ledNotification = 128;

/////////////////////////////////////////////////////////////////////
// CREATE THRESHOLDS FOR WHICH THE JOYSTICK IS CONSIDERED CENTERED //
/////////////////////////////////////////////////////////////////////

// For X:
// |----------|----|----------|
// 0         501  521        1023
const int X_THRESHOLD_LOW = 501;
const int X_THRESHOLD_HIGH = 513;

// For Y:
// |----------|----|----------|
// 0         497  525        1023
const int Y_THRESHOLD_LOW = 494;
const int Y_THRESHOLD_HIGH = 528;

// Define the max range the joystick moves
double xRange = 1023 - X_THRESHOLD_HIGH;
double yRange = 1023 - Y_THRESHOLD_HIGH;

void setup()
{
  // Set the pinMode for each pin (OUTPUT)

  // Directional Pins
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(36, OUTPUT);

  // Speed Pins
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  
  //Launcher Pin
  pinMode(launcherPin, OUTPUT);
 
  ////////////////////////////////
  // Start Serial Communication //
  ////////////////////////////////
  Serial.begin(BaudRate);
  pinMode(pinRx, INPUT);
  pinMode(pinTx, OUTPUT);
  mySerial.begin(BaudRate);
  
  //////////////////////
  // Define Servo Pin //
  //////////////////////
  //myServo.attach(servoPin);
}

void loop()
{
  xDirection = 0;
  yDirection = 0;
 // servoPosition = 0;
  // Monitor Rx from PC , if the data is available then read
  // it to "GotChar".    Then ask XBee send the data out
  // wirelessly. (TX)
  //  if ( Serial.available() ) {
  //      GotChar = Serial.read();
  //
  //      Serial.print("Sending: ");
  //      Serial.println(GotChar);
  //
  //      mySerial.write(GotChar);
  //  }

  ////////////////////////////
  // Monitor data from XBee //
  ////////////////////////////
  if (mySerial.available() > 4) {

    if (mySerial.read() == 255) {

      // If we see the start byte, record the next few values
      commands[0] = mySerial.read();
      commands[1] = mySerial.read();
      commands[2] = mySerial.read();
      commands[3] = mySerial.read();

      // Check to make sure the end byte matches up
      if (commands[3] == 254) {
      
      // Check if the ball is in the launcher  
      if(analogRead(sensorPin) > 600 && ballInHopper == false){
        // The ball has entered the launcher
        
         // Turn on the motors at full speed momentarily to overcome the friction
         analogWrite(launcherPin, 255);
         ballInHopper = true;
       } else if(analogRead(sensorPin) > 600 && ballInHopper == true){
         // The ball is still in the launcher
         
           // Read in the launcher motor speed
           launcherSpeed = commands[2]*(maxSpeed/250.0);
           
           // Create a lower bound for the motor speed
           launcherSpeed = launcherSpeed < minSpeed ? minSpeed : launcherSpeed;
           
           // Set motors to the correct speed
           analogWrite(launcherPin, launcherSpeed);
           mySerial.write(ledNotification);
           Serial.println(launcherSpeed);
       } else {
         //If the ball isn't in the launcher, turn off the motors
         analogWrite(launcherPin, 0);
         ballInHopper = false;
       }
        ///////////////////////////////
        // Check for Button Commands //
        ///////////////////////////////
        
        if ( commands[0] == 251 && commands[1] == 251 ) {
          // If values recieved both equal 251, spin clockwise
          spinCW(128);
        } else if ( commands[0] == 252 && commands[1] == 252 ) {
          // If values recieved both equal 252, spin counter-clockwise
          spinCCW(128);
          //analogWrite(launcherPin, launcherSpeed);
          
          
        } else if ( commands[0] == 253 && commands[1] == 253 ) {
          // If values recieved both equal 253, launch the ball
          
          // Spin up the launcher motor
          analogWrite(launcherPin, launcherSpeed);

        } else {
          //analogWrite(launcherPin, 0);
          // If values sent aren't button commands, proceed with speed calculations

          /////////////////////////
          // Calculate the speed //
          /////////////////////////

          // Scale recieved data to full range of 1023
          xPosition = commands[0] * (1023.0 / 250);
          yPosition = commands[1] * (1023.0 / 250);

          // Scale the position of the pots between -1 and 1
          xScaled = (xPosition - xRange) / xRange;
          yScaled = (yPosition - yRange) / yRange;

          // Make sure -1 < x < 1 and -1 < y < 1
          if (xScaled > 1.00) {
            xScaled = 1.00;
          } else if (xScaled < -1.00) {
            xScaled = -1.00;
          }
          if (yScaled > 1.00) {
            yScaled = 1.00;
          } else if (yScaled < -1.00) {
            yScaled = -1.00;
          }

          // Map the output from the pots (which describe a square) to a circle, for use in polar
          // Multiply by 255 to scale the output
          xMapped = xScaled * sqrt(1 - ((yScaled * yScaled) / 2)) * 255;
          yMapped = yScaled * sqrt(1 - ((xScaled * xScaled) / 2)) * 255;
          xSpeed = abs(xMapped);
          ySpeed = abs(yMapped);

          //////////////////////////
          // Find Motor Direction //
          //////////////////////////

          // For the x-direction
          if (xPosition > X_THRESHOLD_HIGH) {
            // Positive x-direction
            xDirection = 1;
          } else if (xPosition < X_THRESHOLD_LOW) {
            // Negative x-direction
            xDirection = -1;
          }

          // For the y-direction
          if (yPosition > Y_THRESHOLD_HIGH) {
            // Positive y-direction
            yDirection = 1;
          } else if (yPosition < Y_THRESHOLD_LOW) {
            //Negative y-direction
            yDirection = -1;
          }

          // Set the speed to 0 if the joystick is centered
          if (xDirection == 0 && yDirection == 0) {

            // Set angle and speed to 0 when centered
            xSpeed = 0;
            ySpeed = 0;

          }

          //////////////////////////////
          // Output Commands to Motor //
          //////////////////////////////

          // For x-direction (front and back motors)
          if (xDirection == -1) {
            driveLeft(xSpeed);
          } else if (xDirection == 1) {
            driveRight(xSpeed);
          } else {
            xStop();
          }

          // For y-direction (left and right motors)
          if (yDirection == -1) {
            driveReverse(ySpeed);
          } else if (yDirection == 1) {
            driveForward(ySpeed);
          } else {
            yStop();
          }

        }

      }

    }

  }

}

/////////////////////////////////////////////
// Define a bunch of directional functions //
/////////////////////////////////////////////

/////////////
// FORWARD //
/////////////
void driveForward(byte speed) {

  // These set the Speed
  // Speed is a value from 0 to 255
  analogWrite(9, speed);
  analogWrite(8, speed);

  // Set the Direction
  // To make the RIGHT motor spin FORWARD, set rightMotorForward to HIGH and rightMotorReverse to LOW
  digitalWrite(30, HIGH);
  digitalWrite(32, LOW);

  // To make the LEFT motor spin FORWARD, set leftMotorForward to HIGH and leftMotorReverse to LOW
  digitalWrite(34, HIGH);
  digitalWrite(36, LOW);
}

/////////////
// REVERSE //
/////////////
void driveReverse(byte speed) {

  // These set the Speed
  // Speed is a value from 0 to 255
  analogWrite(9, speed);
  analogWrite(8, speed);

  // Set the Direction
  // To make the RIGHT motor spin in REVERSE, set rightMotorForward to LOW and rightMotorReverse to HIGH
  digitalWrite(32, HIGH);
  digitalWrite(30, LOW);

  // To make the LEFT motor spin in REVERSE, set leftMotorForward to LOW and leftMotorReverse to HIGH
  digitalWrite(36, HIGH);
  digitalWrite(34, LOW);

}

/////////////////
// Strafe LEFT //
/////////////////
void driveLeft(byte speed)
{
  // These set the Speed
  // Speed is a value from 0 to 255
  analogWrite(6, speed-10);
  analogWrite(7, speed);

  // Set the Direction
  // To make the FRONT motor spin to the LEFT, set frontMotorLeft to HIGH and frontMotorRight to LOW
  digitalWrite(26, HIGH);
  digitalWrite(28, LOW);

  // To make the REAR motor spin to the LEFT, set rearMotorLeft to HIGH and rearMotorRight to LOW
  digitalWrite(22, HIGH);
  digitalWrite(24, LOW);
}

//////////////////
// Strafe RIGHT //
//////////////////
void driveRight(byte speed)
{

  // These set the Speed
  // Speed is a value from 0 to 255
  analogWrite(6, speed-10);
  analogWrite(7, speed);

  // Set the Direction
  // To make the FRONT motor spin to the RIGHT, set frontMotorLeft to LOW and frontMotorRight to HIGH
  digitalWrite(28, HIGH);
  digitalWrite(26, LOW);

  // To make the REAR motor spin to the RIGHT, set rearMotorLeft to LOW and rearMotorRight to HIGH
  digitalWrite(24, HIGH);
  digitalWrite(22, LOW);

}

//////////////////////
// X-Direction STOP //
//////////////////////
void xStop() {

  // Set all speeds to 0
  analogWrite(6, 0);
  analogWrite(7, 0);

  // Reset all directional commands
  digitalWrite(22, LOW);
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);

}

//////////////////////
// Y-Direction STOP //
//////////////////////
void yStop() {

  // Set all speeds to 0
  analogWrite(8, 0);
  analogWrite(9, 0);

  // Reset all directional commands
  digitalWrite(30, LOW);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW);
}

////////////////////////////
// Spin RIGHT (Clockwise) //
////////////////////////////
void spinCW(byte speed)
{
  // Speed is a value from 0 to 255
  analogWrite(rearMotorSpeed, speed);
  analogWrite(frontMotorSpeed, speed);
  analogWrite(rightMotorSpeed, speed);
  analogWrite(leftMotorSpeed, speed);

  // To get the robot to spin counter-clockwise the motors need to spin in the following manner:
  // Front Motor -> RIGHT
  // Left Motor -> FORWARD
  // Rear Motor -> LEFT
  // Right Motor -> REVERSE

  // To make the FRONT motor spin to the RIGHT, set frontMotorLeft to LOW and frontMotorRight to HIGH
  digitalWrite(frontMotorLeft, LOW);
  digitalWrite(frontMotorRight, HIGH);

  // To make the REAR motor spin to the LEFT, set rearMotorLeft to HIGH and rearMotorRight to LOW
  digitalWrite(rearMotorLeft, HIGH);
  digitalWrite(rearMotorRight, LOW);

  // To make the LEFT motor spin FORWARD, set leftMotorReverse to LOW and leftMotorForward to HIGH
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorReverse, LOW);

  // To make the RIGHT motor spin in REVERSE set rightMotorReverse to HIGH and rightMotorForward to LOW
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorReverse, HIGH);
}

///////////////////////////////////
// Spin LEFT (Counter-Clockwise) //
///////////////////////////////////
void spinCCW(byte speed)
{
  // Speed is a value from 0 to 255
  analogWrite(rearMotorSpeed, speed);
  analogWrite(frontMotorSpeed, speed);
  analogWrite(rightMotorSpeed, speed);
  analogWrite(leftMotorSpeed, speed);

  // To get the robot to spin counter-clockwise the motors need to spin in the following manner:
  // Front Motor -> LEFT
  // Left Motor -> REVERSE
  // Rear Motor -> RIGHT
  // Right Motor -> FORWARD

  // To make the FRONT motor spin to the LEFT, set frontMotorLeft to HIGH and frontMotorRight to LOW
  digitalWrite(frontMotorLeft, HIGH);
  digitalWrite(frontMotorRight, LOW);

  // To make the REAR motor spin to the RIGHT, set rearMotorLeft to LOW and rearMotorRight to HIGH
  digitalWrite(rearMotorLeft, LOW);
  digitalWrite(rearMotorRight, HIGH);

  // To make the LEFT motor spin in REVERSE, set leftMotorReverse to HIGH and leftMotorForward to LOW
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorReverse, HIGH);

  // To make the RIGHT motor spin FORWARD set rightMotorReverse to LOW and rightMotorForward to HIGH
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorReverse, LOW);
}
