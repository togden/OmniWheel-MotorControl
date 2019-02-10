#include <SoftwareSerial.h>

//Create variables for each button on the Joystick Shield to assign the pin numbers
//   Right Button    |  Top Button  | Bottom Button
char launchButton = 3, rotateCW = 4, rotateCCW = 5;

// Define the pins on Arduino for XBee communication
// XBee shield be set to DLINE to use D2 and D3
// Place a jumper wire from pin D2 to pin D11
int pinRx = 2 , pinTx = 3; // pins on Arduino
long BaudRate = 9600 , sysTick = 0;

// Initialize XBee communication line with NewSoftSerial
SoftwareSerial mySerial( pinRx , pinTx );

// Set the joystick Potentiometer Pins to Analog 0 and Analog 1
const byte PIN_ANALOG_X = 0; //Set the X pot pin
const byte PIN_ANALOG_Y = 1; // Set the Y pot pin
byte xValue;
byte yValue;
int xPosition;
int yPosition;
int launcherPot;
byte launcherSpeed;
int ledNotification;
int ledPin = 10;


void setup(void)
{
  pinMode(launchButton, INPUT);      // Set the Joystick rightButton as an input
  digitalWrite(launchButton, HIGH);  // Enable the pull-up resistor on rightButton

  pinMode(rotateCW, INPUT);      //Set the Joystick topButton as an input
  digitalWrite(rotateCW, HIGH);  // Enable the pull-up resistor on topButton

  pinMode(rotateCCW, INPUT);      // Set the Joystick bottomButton as an input
  digitalWrite(rotateCCW, HIGH);  // Enable the pull-up resistor on bottomButton
  
  //pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, LOW);
  
  Serial.begin(BaudRate);           // Turn on the Serial Port at 9600 bps
  pinMode(pinRx, INPUT);
  pinMode(pinTx, OUTPUT);
  mySerial.begin(BaudRate);

}

void loop(void)
{
  ledNotification = 0;
  ////////////////////////////
  // Read Joystick Position //
  ////////////////////////////
  xPosition = analogRead(PIN_ANALOG_X);
  yPosition = analogRead(PIN_ANALOG_Y);
  
  /////////////////////////
  // Read Launcher Speed //
  /////////////////////////
  launcherPot = analogRead(A5);
  launcherSpeed = launcherPot * (250.0 / 1023);
  
  /////////////////////////
  // Scale Between 0-240 //
  /////////////////////////
  xValue = xPosition * (250.0 / 1023);
  yValue = yPosition * (250.0 / 1023);

  // Make sure not to send 43, because that can put the xBee into programming mode
  if ( xValue == 43) {
    xValue = 44;
  }
  if ( yValue == 43) {
    yValue = 44;
  }

  ////////////////////////
  // Check Button Input //
  ////////////////////////
  
  // If the right bumper is pressed
  if (digitalRead(rotateCW) == LOW) {
    xValue = 251;
    yValue = 251;
  }

  // If the left bumper is pressed
  if (digitalRead(rotateCCW) == LOW) {
    xValue = 252;
    yValue = 252;
  }

  // If the "Launch Button" is pressed
  if (digitalRead(launchButton) == LOW) {
    xValue = 253;
    yValue = 253;
  }

  ///////////////////
  // Send the Data //
  ///////////////////

  // Create an array of commands
  byte commands[] = { 255, xValue, yValue, launcherSpeed, 254};
  
  // Send the array!
  mySerial.write(commands, sizeof(commands));
  delay(50);
  
  if (mySerial.available()>0){
    ledNotification = mySerial.read();
  }
  Serial.println(ledNotification);
  /////////////////
  // Turn on LED //
  /////////////////
    analogWrite(ledPin, ledNotification);
  
}
