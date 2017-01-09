#include "SoftwareSerial.h"

#define    VERSION      "\n\nAndroTest V2.0 - @kas2014\ndemo for V5.x App"
#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

/* PIN DEFINITIONS */
#define    ledPin       13
#define    MOTOR2_PIN1  3
#define    MOTOR2_PIN2  5
#define    MOTOR1_PIN1  6
#define    MOTOR1_PIN2  9
#define    PROXI_PIN    7
#define    LIGHT_PIN    0    
/* END PIN DEFINITIONS */

// 0 - light pin
// 1, 2, 4 - proximity front - left, center, right
// 7 - proximity back
// 3, 5, 6, 9 - motor

SoftwareSerial mySerial(4,2);

int lastDistances[3] = {500, 500, 500};

/* BLUETOOTH VARIABLES */
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "lol";                          // message to Android device
/* END BLUETOOTH VARIABLES */

/* CONFIGURATION */
// global configuration variables
int turnDelay        = 3000;
int reverseTurnSpeed = 200;
int lightThreshold   = 750;

// start with light sensor
int useFlash         = 1;

// start with joystick control
int useJoystick      = 1;

// light sensor variables
int flashNumber      = 0;
int lastFlashNumber  = 0;
/* END CONFIGURATION */


void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  pinMode(PROXI_PIN, INPUT);

  // BLUETOOTH CONFIGURATION
  Serial.begin(9600);
  mySerial.begin(115200);
  mySerial.print("$");
  mySerial.print("$");
  mySerial.print("$");
  delay(100);
  mySerial.println("U,9600,N");
  mySerial.begin(9600);  // Start bluetooth serial at 9600                            // 57600 = max value for softserial
  pinMode(ledPin, OUTPUT);
  Serial.println(VERSION);
  while (mySerial.available())  mySerial.read();        // empty RX buffer
  
  Serial.begin(9600);
}

void loop() {
  if( useJoystick ) { // robot will be controlled by joystick
    joystickControl();
  }
  else { // robot will be controlled by flash
    flashControl();

    int *distances = (int *) malloc (3*sizeof(int));
    int *trends = (int *) malloc (3*sizeof(int));
    
    readDistances( distances );

    // calculate distance trends
    // -1 : descending
    //  0 : stationary
    // +1 : ascending

    int bestTrend = -1;
    int bestDirection = -1;
    
    for(int i=0;i<3;i++) {
      if( distances[i] > 0 && distances[i] < 300 ) {
        trends[i] = distances[i] / lastDistances[i];

        if( trends[i] > bestTrend ) {
          bestTrend = trends[i];
          bestDirection = i;
        }
        
        lastDistances[i] = distances[i];
      }
    }

    switch( bestTrend ) {
      case -1:
        goBack();
        break;
      case 0:
        goLeft();
        break;
      case 2:
        goRight();
        break;
      default:
        goFwd();
        break;
    }


    /*
    if( digitalRead(PROXI_PIN) ) { // no obstacle found, move foward
      goFwd();
    }
    else { // obstacle found
      handleObstacle();
    }
    */
    
    delay(250);
  }
}

void readDistances( int *distances ) {
  distances[0] = readDistanceAverage(5,1); // left
  distances[1] = readDistanceAverage(5,2); // center
  distances[2] = readDistanceAverage(5,4); // right

  /*
  if( (distances[0]>0 && distances[0]<30) || (distances[0]>0 && distances[0]<30) || (distances[0]>0 && distances[0]<30) ) {
    Stop();
  }
  */
}

void goBack() { go(-255,-255); }
void goFwd() { go(255,255); }
void goLeft() { go(-reverseTurnSpeed,255); }
void goRight() { go(255,-reverseTurnSpeed); }
void Stop() { go(0,0); }

void go(int speedLeft, int speedRight) {
  if (speedLeft >= 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  } 
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }
 
  if (speedRight >= 0) {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, speedRight);
  }else {
    analogWrite(MOTOR2_PIN1, -speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  }
}

/**
 * chooseTurnDirection
 * @returns int
 * return values:
 *  -1: undecided
 *   0: left
 *   1: right
 */
int chooseTurnDirection() {
  int chosenDirection = 0; // chosen direction, 0=left, 1=right
  int leftOk = 0;
  int rightOk = 0;

  // turn left and check the proximity sensor
  goLeft();
  delay(turnDelay);
  leftOk = digitalRead(PROXI_PIN);

  // reset the position
  goRight();
  delay(turnDelay);

  // turn right and check the proximity sensor
  goRight();
  delay(turnDelay);
  rightOk = digitalRead(PROXI_PIN);

  // decide which direction to turn

  if( leftOk && rightOk ) { // if both directions are OK, choose random one
    return random(2);
  }
  if( leftOk ) { // left direction chosen
      return 0;
  }
  if( rightOk ) { // right direction chosen
      return 1;
  }

  // undecided
  return -1;
}

/*
 * readLightSensor
 * @returns int
 * @description Reads twice the value recorded by the light sensor and returns the higher one.
 */
int readLightSensor() {
  int v1 = analogRead(LIGHT_PIN);
  delay(100);
  int v2 = analogRead(LIGHT_PIN);
  return (v1 > v2) ? v1 : v2;
}

/*
 * isFlash
 * @returns int
 * @description Decides is a certain value returned by the light sensor is a flash of light.
 * If the value is greater than the threshold, then it will be considered as a flash.
 */
int isFlash( int value ) {
  if( value > lightThreshold ) {
    return 1;
  }
  return 0;
}

/*
 * flashControl
 * @returns void
 * @description Controls the behaviour when controlled by flashes of light.
 * First flash sets the machine in motion.
 * Subsequent flashes turn the machine 180 degrees.
 */
void flashControl() {
  int lightValue = readLightSensor();
  if( isFlash(lightValue) ) {
    lastFlashNumber = flashNumber;
    flashNumber++;
  }

  if( !flashNumber ) {
    delay(250);
    return;
  }

  if( flashNumber != lastFlashNumber ) {
    lastFlashNumber = flashNumber;
    if( flashNumber > 1 ) {
      goLeft();
      delay(turnDelay*2);
    }
  }
}

/*
 * handleObstacle
 * @returns void
 * @description Handle an obstacle
 */
void handleObstacle() {
  //stop
  Stop();
  delay(2000);

  // move back
  int movedBack = 0;
  while(!digitalRead(PROXI_PIN)) { // move back only if obstacle is still there
    goBack();
    delay(2500);
    movedBack = 1;
  }
  
  if( movedBack ) {  
    // move back another step, to have space to turn
    goBack();
    delay(1000);

    // choose turn direction
    int chosenDirection = chooseTurnDirection();
    while( chosenDirection < 0 ) {
        goBack();
        delay(1000);
        chosenDirection = chooseTurnDirection();
    }

    // turn towards selected direction
    if( chosenDirection ) {
        goRight();
        delay(turnDelay);
    } else {
        goLeft();
        delay(turnDelay);
    }
  }
}

void joystickControl() {
  if (mySerial.available())  {                          // data received from smartphone
    delay(2);
    cmd[0] =  mySerial.read();
    if (cmd[0] == STX)  {
      int i = 1;
      while (mySerial.available())  {
        delay(1);
        cmd[i] = mySerial.read();
        if (cmd[i] > 127 || i > 7)                 break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      if     (i == 2)          getButtonState(cmd[1]);  // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)          getJoystickState(cmd);  // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
  sendBlueToothData();
}


void sendBlueToothData()  {
  static long previousMillis = 0;
  long currentMillis = millis();
  if (currentMillis - previousMillis > sendInterval) {  // send data back to smartphone
    previousMillis = currentMillis;

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    mySerial.print((char)STX);                                             // Start of Transmission
    mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
    mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
    mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
    mySerial.print(displayStatus);                                         // datafield #3
    mySerial.print((char)ETX);                                             // End of Transmission
  }
}


String getButtonStatusString()  {
  String bStatus = "";
  for (int i = 0; i < 6; i++)  {
    if (buttonStatus & (B100000 >> i))      bStatus += "1";
    else                                  bStatus += "0";
  }
  return bStatus;
}

int GetdataInt1()  {              // Data dummy values sent to Android device for demo purpose
  static int i = -30;             // Replace with your own code
  i ++;
  if (i > 0)    i = -30;
  return i;
}

float GetdataFloat2()  {           // Data dummy values sent to Android device for demo purpose
  static float i = 50;             // Replace with your own code
  i -= .5;
  if (i < -50)    i = 50;
  return i;
}

void getJoystickState(byte data[8])    {
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)     return; // commmunication error

  // Your code here ...
  Serial.print("Joystick position:  ");
  Serial.print(joyX);
  Serial.print(", ");
  Serial.println(joyY);


  if (joyY >= 90) {
    goFwd();
    Serial.println("inainte");
  } else if (joyX == 0 && joyY == 0) {
    Stop();
    Serial.println("stop");

  }
  if (joyY < -90) {
    goBack();
    Serial.println("inapoi");
  } else if (joyX == 0 && joyY == 0) {
    Stop();
    Serial.println("stop");
  }

  if (joyX >= 90) {
    goRight();
    Serial.println("dreapta");

  } else if (joyX == 0 && joyY == 0) {
    Stop();
    Serial.println("stop");
  }
  if (joyX < -90) {
    goLeft();
    Serial.println("stanga");
  } else if (joyX == 0 && joyY == 0) {
    Stop();
    Serial.println("stop");
  }

}

void getButtonState(int bStatus)  {
  switch (bStatus) {
      // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      Serial.println("\n** Button_1: ON **");
      // your code...
      displayStatus = "LED <ON>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, HIGH);
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      Serial.println("\n** Button_1: OFF **");
      // your code...
      displayStatus = "LED <OFF>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, LOW);
      break;

      // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      Serial.println("\n** Button_2: ON **");
      // your code...
      displayStatus = "Button2 <ON>";
      Serial.println(displayStatus);
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      Serial.println("\n** Button_2: OFF **");
      // your code...
      displayStatus = "Button2 <OFF>";
      Serial.println(displayStatus);
      break;

      // -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      Serial.println("\n** Button_3: ON **");
      // your code...
      displayStatus = "Motor #1 enabled"; // Demo text message
      Serial.println(displayStatus);
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      Serial.println("\n** Button_3: OFF **");
      // your code...
      displayStatus = "Motor #1 stopped";
      Serial.println(displayStatus);
      break;

      // -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      Serial.println("\n** Button_4: ON **");
      // your code...
      displayStatus = "Datafield update <FAST>";
      Serial.println(displayStatus);
      sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      Serial.println("\n** Button_4: OFF **");
      // your code...
      displayStatus = "Datafield update <SLOW>";
      Serial.println(displayStatus);
      sendInterval = SLOW;
      break;

      // -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
      //      buttonStatus |= B010000;        // ON
      Serial.println("\n** Button_5: ++ pushed ++ **");
      // your code...
      displayStatus = "Button5: <pushed>";
      break;
      //   case 'J':
      //     buttonStatus &= B101111;        // OFF
      //     // your code...
      //     break;

      // -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      Serial.println("\n** Button_6: ON **");
      // your code...
      displayStatus = "Button6 <ON>"; // Demo text message
      break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      Serial.println("\n** Button_6: OFF **");
      // your code...
      displayStatus = "Button6 <OFF>";
      break;
  }
  // ---------------------------------------------------------------
}

int readDistanceAverage(int count, int pin) {
  int sum = 0;
  for (int i = 0; i<count; i++) {
    float volts = analogRead(pin) * ((float) 5 / 1024);
    float distance = 65 * pow(volts, -1.10); 
    sum = sum + distance;
    delay(1);
  }
  return (int) (sum/count);
}

