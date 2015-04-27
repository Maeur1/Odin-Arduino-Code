
/*
 This Is a sketch for Odin, the mecanum wheeled robot made by Mayur Panchal.
 It is controlled by a PS3 controller
 */

#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>
#include <NewPing.h>
#include <SPI.h>

int MaxSensorDistance = 400;
int MinRoll = 160;
int MaxRoll = 200;
int LeftRight = 8;
int FrontBack = 8;

USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd, 0x00, 0x11, 0x22, 0x98, 0x76, 0x54); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
Servo LeftFront;
Servo LeftRear;
Servo RightFront;
Servo RightRear;
NewPing sonarfront(A0, A1, MaxSensorDistance);
NewPing sonarback(A3, A2, MaxSensorDistance);
NewPing sonarright(A5, A4, MaxSensorDistance);
NewPing sonarleft(A9, A8, MaxSensorDistance);

unsigned int duration;
unsigned int distancefront;
unsigned int distanceback;
unsigned int distanceleft;
unsigned int distanceright;
unsigned int Time;

boolean MotionMove;
boolean SensorMove;

void setup() {
  Serial.begin(115200);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  LeftFront.attach(6);
  LeftRear.attach(4);
  RightFront.attach(7);
  RightRear.attach(5);
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  LeftFront.writeMicroseconds(1500);
  LeftRear.writeMicroseconds(1500);
  RightFront.writeMicroseconds(1500);
  RightRear.writeMicroseconds(1500);
}
void loop() {
  Usb.Task();
  if (PS3.PS3Connected) {
    if (!MotionMove && !SensorMove) {
      if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
        Serial.print(F("\r\nLeftHatX: "));
        Serial.print(PS3.getAnalogHat(LeftHatX));
        Serial.print(F("\tLeftHatY: "));
        Serial.print(PS3.getAnalogHat(LeftHatY));
        if (!PS3.PS3NavigationConnected) {
          Serial.print(F("\tRightHatX: "));
          Serial.print(PS3.getAnalogHat(RightHatX));
          Serial.print(F("\tRightHatY: "));
          Serial.print(PS3.getAnalogHat(RightHatY));
        }
      }

      //Stay stationary when sitting there
      if (PS3.getAnalogHat(LeftHatX) == 128 && PS3.getAnalogHat(LeftHatY) == 128 && PS3.getAnalogHat(RightHatX) == 128 && PS3.getAnalogHat(RightHatY) == 128) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      //Stay Stationary when controller doesn't connect
      if (PS3.getAnalogHat(LeftHatX) == 0 && PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(RightHatX) == 0 && PS3.getAnalogHat(RightHatY) == 0) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      //Move Forward
      if (PS3.getAnalogHat(LeftHatY) < 117) {
        //Move Forward Right
        if (PS3.getAnalogHat(RightHatX) > 137) {
          scanfront();
          //Check that the way is clear with ultrasonic sensor
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distancefront);
          if (distancefront >= FrontBack || distancefront == 0) {
            LeftFront.writeMicroseconds(1000);
            LeftRear.writeMicroseconds(1000);
            RightFront.writeMicroseconds(1600);
            RightRear.writeMicroseconds(1600);
          }
          //If the way isn't clear, DONT MOVE
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
        //Move Forward Left
        else if (PS3.getAnalogHat(RightHatX) < 117) {
          scanfront();
          //Check that the way is clear with ultrasonic sensor
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distancefront);
          if (distancefront >= FrontBack || distancefront == 0) {
            LeftFront.writeMicroseconds(1400);
            LeftRear.writeMicroseconds(1400);
            RightFront.writeMicroseconds(2000);
            RightRear.writeMicroseconds(2000);
          }
          //If the way isn't clear, DONT MOVE
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
        //Okay so if you aren't moving left or right, so continue with straight movement
        else {
          scanfront();
          //Check that the way is clear with ultrasonic sensor
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distancefront);
          if (distancefront >= FrontBack || distancefront == 0) {
            LeftFront.writeMicroseconds(1500 - (128 - PS3.getAnalogHat(LeftHatY)) * (500 / 128));
            LeftRear.writeMicroseconds(1500 - (128 - PS3.getAnalogHat(LeftHatY)) * (500 / 128));
            RightFront.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(LeftHatY)) * (500 / 128));
            RightRear.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(LeftHatY)) * (500 / 128));
          }
          //If the way isn't clear, DONT MOVE
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
      }

      //Move Backwards
      if (PS3.getAnalogHat(LeftHatY) > 137) {
        //Move Backwards Left
        if (PS3.getAnalogHat(RightHatX) < 117) {
          scanback();
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distanceback);
          //Check that the way is clear with ultrasonic sensor
          if (distanceback >= FrontBack || distanceback == 0) {
            LeftFront.writeMicroseconds(2000);
            LeftRear.writeMicroseconds(2000);
            RightFront.writeMicroseconds(1400);
            RightRear.writeMicroseconds(1400);
          }
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
        //Move Backwards Right
        else if (PS3.getAnalogHat(RightHatX) > 137) {
          scanback();
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distanceback);
          //Check that the way is clear with ultrasonic sensor
          if (distanceback >= FrontBack || distanceback == 0) {
            LeftFront.writeMicroseconds(1600);
            LeftRear.writeMicroseconds(1600);
            RightFront.writeMicroseconds(1000);
            RightRear.writeMicroseconds(1000);
          }
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
        //If you ain't moving left or right, you can continue to go backwards
        else {
          scanback();
          Serial.print(F("\tSensor Distance: "));
          Serial.println(distanceback);
          //Check that the way is clear with ultrasonic sensor
          if (distanceback >= FrontBack || distanceback == 0) {
            LeftFront.writeMicroseconds(1500 + (PS3.getAnalogHat(LeftHatY) - 128) * (500 / 127));
            LeftRear.writeMicroseconds(1500 + (PS3.getAnalogHat(LeftHatY) - 128) * (500 / 127));
            RightFront.writeMicroseconds(1500 - (PS3.getAnalogHat(LeftHatY) - 128) * (500 / 127));
            RightRear.writeMicroseconds(1500 - (PS3.getAnalogHat(LeftHatY) - 128) * (500 / 127));
          }
          else {
            LeftFront.writeMicroseconds(1500);
            LeftRear.writeMicroseconds(1500);
            RightFront.writeMicroseconds(1500);
            RightRear.writeMicroseconds(1500);
          }
        }
      }

      //Move diagonaly Forward Left (Only two wheels spin)
      if (PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(LeftHatX) == 0) {
        scanfront();
        //Check that the way is clear with ultrasonic sensor
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distancefront);
        if (distancefront >= FrontBack || distancefront == 0) {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1000);
          RightFront.writeMicroseconds(2000);
          RightRear.writeMicroseconds(1500);
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Move Diagonally Forward Right (Only two wheels spin)
      if (PS3.getAnalogHat(LeftHatY) == 0 && PS3.getAnalogHat(LeftHatX) == 255) {
        scanfront();
        //Check that the way is clear with ultrasonic sensor
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distancefront);
        if (distancefront >= FrontBack || distancefront == 0) {
          LeftFront.writeMicroseconds(1000);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(2000);
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Move Diagonally Backward Left (Only two wheels spin)
      if (PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(LeftHatX) == 0) {
        scanback();
        //Check that the way is clear with ultrasonic sensor
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceback);
        if (distanceback >= FrontBack || distancefront == 0) {
          LeftFront.writeMicroseconds(2000);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1000);
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Move Diagonally Backward Right (Only two wheels spin)
      if (PS3.getAnalogHat(LeftHatY) == 255 && PS3.getAnalogHat(LeftHatX) == 255) {
        scanback();
        //Check that the way is clear with ultrasonic sensor
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceback);
        if (distanceback >= FrontBack || distancefront == 0) {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(2000);
          RightFront.writeMicroseconds(1000);
          RightRear.writeMicroseconds(1500);
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Move Paralell Left <--- This looks cool
      if (PS3.getAnalogHat(LeftHatX) < 117 && PS3.getAnalogHat(LeftHatY) == 128) {
        scanleft();
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceleft);
        if (distanceleft >= LeftRight || distanceleft == 0) {
          LeftFront.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(LeftHatX)) * (500 / 128));
          LeftRear.writeMicroseconds(1500 - (128 - PS3.getAnalogHat(LeftHatX)) * (500 / 128));
          RightFront.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(LeftHatX)) * (500 / 128));
          RightRear.writeMicroseconds(1500 - (128 - PS3.getAnalogHat(LeftHatX)) * (500 / 128));
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Move Paralell Right <--- This looks cool too
      if (PS3.getAnalogHat(LeftHatX) > 137 && PS3.getAnalogHat(LeftHatY) == 128) {
        scanright();
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceright);
        if (distanceright >= LeftRight || distanceright == 0) {
          LeftFront.writeMicroseconds(1500 - (PS3.getAnalogHat(LeftHatX) - 128) * (500 / 127));
          LeftRear.writeMicroseconds(1500 + (PS3.getAnalogHat(LeftHatX) - 128) * (500 / 127));
          RightFront.writeMicroseconds(1500 - (PS3.getAnalogHat(LeftHatX) - 128) * (500 / 127));
          RightRear.writeMicroseconds(1500 + (PS3.getAnalogHat(LeftHatX) - 128) * (500 / 127));
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      //Rotate Right "Spin ma head right round"
      if (PS3.getAnalogHat(RightHatX) > 137 && PS3.getAnalogHat(LeftHatY) == 128) {
        LeftFront.writeMicroseconds(1500 - (PS3.getAnalogHat(RightHatX) - 128) * (500 / 127));
        LeftRear.writeMicroseconds(1500 - (PS3.getAnalogHat(RightHatX) - 128) * (500 / 127));
        RightFront.writeMicroseconds(1500 - (PS3.getAnalogHat(RightHatX) - 128) * (500 / 127));
        RightRear.writeMicroseconds(1500 - (PS3.getAnalogHat(RightHatX) - 128) * (500 / 127));
      }

      //Rotate Left "Right round"
      if (PS3.getAnalogHat(RightHatX) < 117 && PS3.getAnalogHat(LeftHatY) == 128) {
        LeftFront.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(RightHatX)) * (500 / 128));
        LeftRear.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(RightHatX)) * (500 / 128));
        RightFront.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(RightHatX)) * (500 / 128));
        RightRear.writeMicroseconds(1500 + (128 - PS3.getAnalogHat(RightHatX)) * (500 / 128));
      }


      //Analog button values can be read from almost all buttons
      if (PS3.getAnalogButton(L2) || PS3.getAnalogButton(R2)) {
        Serial.print(F("\r\nL2: "));
        Serial.print(PS3.getAnalogButton(L2));
        if (!PS3.PS3NavigationConnected) {
          Serial.print(F("\tR2: "));
          Serial.print(PS3.getAnalogButton(R2));
        }
      }
      if (PS3.getButtonClick(PS)) {
        Serial.print(F("\r\nPS"));
        /*PS3.disconnect();
        Dangerous stuff here, It turns the controller off if you press it....
        */
      }

      else {
        if (PS3.getButtonClick(TRIANGLE))
          Serial.print(F("\r\nTraingle"));
        if (PS3.getButtonClick(CIRCLE))
          Serial.print(F("\r\nCircle"));
        if (PS3.getButtonClick(CROSS))
          Serial.print(F("\r\nCross"));
        if (PS3.getButtonClick(SQUARE))
          Serial.print(F("\r\nSquare"));

        if (PS3.getButtonClick(UP)) {
          Serial.print(F("\r\nUp"));
        }
        if (PS3.getButtonClick(RIGHT)) {
          Serial.print(F("\r\nRight"));
        }
        if (PS3.getButtonClick(DOWN)) {
          Serial.print(F("\r\nDown"));
        }
        if (PS3.getButtonClick(LEFT)) {
          Serial.print(F("\r\nLeft"));
        }

        if (PS3.getButtonClick(L1))
          Serial.print(F("\r\nL1"));
        if (PS3.getButtonClick(L3))
          Serial.print(F("\r\nL3"));
        if (PS3.getButtonClick(R1))
          Serial.print(F("\r\nR1"));
        if (PS3.getButtonClick(R3))
          Serial.print(F("\r\nR3"));

        if (PS3.getButtonClick(SELECT)) {
          Serial.print(F("\r\nSelect - "));
          PS3.printStatusString();
          if (PS3.PS3Connected) {
            PS3.setAllOff();
            PS3.setLedOn(LED3);
          }
          SensorMove = !SensorMove;
        }
        if (PS3.getButtonClick(START)) {
          Serial.print(F("\r\nStart"));
          if (PS3.PS3Connected) {
            PS3.setAllOff();
            PS3.setLedOn(LED2);
          }
          MotionMove = !MotionMove;
        }
      }
    }
    if (SensorMove) {
      scanfront();
      scanback();
      scanleft();
      scanright();
      Serial.print(F("\r\nSensor Left "));
      Serial.print(distanceleft);
      Serial.print(F("\tSensor Right "));
      Serial.print(distanceright);
      Serial.print(F("\tSensor Front "));
      Serial.print(distancefront);
      Serial.print(F("\tSensor Back "));
      Serial.print(distanceback);

      if (distancefront < FrontBack && distancefront != 0 && distanceleft < LeftRight && distanceleft != 0 && (distanceback == 0 || distanceback >= FrontBack) && (distanceright == 0 || distanceright >= LeftRight)) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(2000);
        RightFront.writeMicroseconds(1000);
        RightRear.writeMicroseconds(1500);
      }

      if (distanceback < FrontBack && distanceback != 0 && distanceright < LeftRight && distanceright != 0 && (distancefront == 0 || distancefront >= FrontBack) && (distanceleft == 0 || distanceleft >= LeftRight)) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1000);
        RightFront.writeMicroseconds(2000);
        RightRear.writeMicroseconds(1500);
      }

      if (distanceback < FrontBack && distanceback != 0 && distanceleft < LeftRight && distanceleft != 0 && (distancefront == 0 || distancefront >= FrontBack) && (distanceright == 0 || distanceright >= LeftRight)) {
        LeftFront.writeMicroseconds(1000);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(2000);
      }

      if (distancefront < FrontBack && distancefront != 0 && distanceright < LeftRight && distanceright != 0 && (distanceleft == 0 || distanceleft >= LeftRight) && (distanceback == 0 || distanceback >= FrontBack)) {
        LeftFront.writeMicroseconds(2000);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1000);
      }

      if (distancefront < FrontBack && distancefront != 0 && distanceback < FrontBack && distanceback != 0 && (distanceleft == 0 || distanceleft >= LeftRight) && (distanceright == 0 || distanceright >= LeftRight)) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      if (distanceright < LeftRight && distanceright != 0 && distanceleft < LeftRight && distanceleft != 0 && (distanceback == 0 || distanceback >= FrontBack) && (distancefront == 0 || distancefront >= FrontBack)) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      if (distancefront < FrontBack && distancefront != 0 && distanceleft < LeftRight && distanceleft != 0 && distanceright < LeftRight && distanceright != 0 && (distanceback == 0 || distanceback >= FrontBack)) {
        LeftFront.writeMicroseconds(1500 + (8 - distancefront) * (500 / 6));
        LeftRear.writeMicroseconds(1500 + (8 - distancefront) * (500 / 6));
        RightFront.writeMicroseconds(1500 - (8 - distancefront) * (500 / 6));
        RightRear.writeMicroseconds(1500 - (8 - distancefront) * (500 / 6));
      }

      if (distanceback < FrontBack && distanceback != 0 && distanceleft < LeftRight && distanceleft != 0 && distanceright < LeftRight && distanceright != 0 && (distancefront == 0 || distancefront >= FrontBack)) {
        LeftFront.writeMicroseconds(1500 - (8 - distanceback) * (500 / 6));
        LeftRear.writeMicroseconds(1500 - (8 - distanceback) * (500 / 6));
        RightFront.writeMicroseconds(1500 + (8 - distanceback) * (500 / 6));
        RightRear.writeMicroseconds(1500 + (8 - distanceback) * (500 / 6));
      }

      if (distanceleft < LeftRight && distanceleft != 0 && distancefront < FrontBack && distancefront != 0 && distanceback < FrontBack && distanceback != 0 && (distanceright == 0 || distanceright >= LeftRight)) {
        LeftFront.writeMicroseconds(1500 - (8 - distanceleft) * (500 / 6));
        LeftRear.writeMicroseconds(1500 + (8 - distanceleft) * (500 / 6));
        RightFront.writeMicroseconds(1500 - (8 - distanceleft) * (500 / 6));
        RightRear.writeMicroseconds(1500 + (8 - distanceleft) * (500 / 6));
      }

      if (distanceright < LeftRight && distanceright != 0 && distancefront < FrontBack && distancefront != 0 && distanceback < FrontBack && distanceback != 0 && (distanceleft == 0 || distanceleft >= LeftRight)) {
        LeftFront.writeMicroseconds(1500 + (8 - distanceright) * (500 / 6));
        LeftRear.writeMicroseconds(1500 - (8 - distanceright) * (500 / 6));
        RightFront.writeMicroseconds(1500 + (8 - distanceright) * (500 / 6));
        RightRear.writeMicroseconds(1500 - (8 - distanceright) * (500 / 6));
      }

      if (distancefront < FrontBack && distancefront != 0 && (distanceback == 0 || distanceback >= FrontBack) && (distanceright == 0 || distanceright >= LeftRight) && (distanceleft == 0 || distanceleft >= LeftRight)) {
        LeftFront.writeMicroseconds(1500 + (8 - distancefront) * (500 / 6));
        LeftRear.writeMicroseconds(1500 + (8 - distancefront) * (500 / 6));
        RightFront.writeMicroseconds(1500 - (8 - distancefront) * (500 / 6));
        RightRear.writeMicroseconds(1500 - (8 - distancefront) * (500 / 6));
      }

      if (distanceback < FrontBack && distanceback != 0 && (distancefront == 0 || distancefront >= FrontBack) && (distanceright == 0 || distanceright >= LeftRight) && (distanceleft == 0 || distanceleft >= LeftRight)) {
        LeftFront.writeMicroseconds(1500 - (8 - distanceback) * (500 / 6));
        LeftRear.writeMicroseconds(1500 - (8 - distanceback) * (500 / 6));
        RightFront.writeMicroseconds(1500 + (8 - distanceback) * (500 / 6));
        RightRear.writeMicroseconds(1500 + (8 - distanceback) * (500 / 6));
      }

      if (distanceright < LeftRight && distanceright != 0 && (distanceleft == 0 || distanceleft >= LeftRight) && (distanceback == 0 || distanceback >= 8) && (distancefront == 0 || distancefront >= FrontBack)) {
        LeftFront.writeMicroseconds(1500 + (8 - distanceright) * (500 / 6));
        LeftRear.writeMicroseconds(1500 - (8 - distanceright) * (500 / 6));
        RightFront.writeMicroseconds(1500 + (8 - distanceright) * (500 / 6));
        RightRear.writeMicroseconds(1500 - (8 - distanceright) * (500 / 6));
      }

      if (distanceleft < LeftRight && distanceleft != 0 && (distanceright == 0 || distanceright >= LeftRight) && (distanceback == 0 || distanceback >= FrontBack) && (distancefront == 0 || distancefront >= FrontBack)) {
        LeftFront.writeMicroseconds(1500 - (8 - distanceleft) * (500 / 6));
        LeftRear.writeMicroseconds(1500 + (8 - distanceleft) * (500 / 6));
        RightFront.writeMicroseconds(1500 - (8 - distanceleft) * (500 / 6));
        RightRear.writeMicroseconds(1500 + (8 - distanceleft) * (500 / 6));
      }

      if (distanceleft < LeftRight && distanceleft != 0 && distanceright < LeftRight && distanceright != 0 && distancefront < FrontBack && distancefront != 0 && distanceback < FrontBack && distanceback != 0) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      if ((distanceleft == 0 || distanceleft >= LeftRight) && (distanceright == 0 || distanceright >= LeftRight) && (distanceback == 0 || distanceback >= FrontBack) && (distancefront == 0 || distancefront >= FrontBack)) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      if (PS3.getButtonClick(SELECT)) {
        Serial.print(F("\r\nSelect - "));
        PS3.printStatusString();
        if (PS3.PS3Connected) {
          PS3.setAllOff();
          PS3.setLedOn(LED1);
        }
        SensorMove = !SensorMove;
      }
    }

    if (MotionMove) {
      Serial.print(F("\r\nPitch: "));
      Serial.print(PS3.getAngle(Pitch));

      if (PS3.getAnalogButton(L2) > 1 && PS3.getAngle(Pitch) >= 160 && PS3.getAngle(Pitch) <= 200 && PS3.getAngle(Roll) >= 160 && PS3.getAngle(Roll) <= 200) {
        LeftFront.writeMicroseconds(1500 + (PS3.getAnalogButton(L2)) * (500 / 255));
        LeftRear.writeMicroseconds(1500 + (PS3.getAnalogButton(L2)) * (500 / 255));
        RightFront.writeMicroseconds(1500 + (PS3.getAnalogButton(L2)) * (500 / 255));
        RightRear.writeMicroseconds(1500 + (PS3.getAnalogButton(L2)) * (500 / 255));
      }

      if (PS3.getAnalogButton(R2) > 1 && PS3.getAngle(Pitch) >= 160 && PS3.getAngle(Pitch) <= 200 && PS3.getAngle(Roll) >= 160 && PS3.getAngle(Roll) <= 200) {
        LeftFront.writeMicroseconds(1500 - (PS3.getAnalogButton(R2)) * (500 / 255));
        LeftRear.writeMicroseconds(1500 - (PS3.getAnalogButton(R2)) * (500 / 255));
        RightFront.writeMicroseconds(1500 - (PS3.getAnalogButton(R2)) * (500 / 255));
        RightRear.writeMicroseconds(1500 - (PS3.getAnalogButton(R2)) * (500 / 255));
      }
      if (PS3.getAngle(Pitch) >= 160 && PS3.getAngle(Pitch) <= 200 && PS3.getAngle(Roll) >= 160 && PS3.getAngle(Roll) <= 200 && PS3.getAnalogButton(R2) == 0 && PS3.getAnalogButton(L2) == 0) {
        LeftFront.writeMicroseconds(1500);
        LeftRear.writeMicroseconds(1500);
        RightFront.writeMicroseconds(1500);
        RightRear.writeMicroseconds(1500);
      }

      if (PS3.getAngle(Pitch) < 160) {
        scanback();
        Serial.print(F("\tRear Sensor Distance: "));
        Serial.println(distanceback);
        if (distanceback >= FrontBack || distanceback == 0) {
          if (PS3.getAngle(Roll) > 230) {
            LeftFront.writeMicroseconds(2000);
            LeftRear.writeMicroseconds(2000);
            RightFront.writeMicroseconds(1400);
            RightRear.writeMicroseconds(1400);
          }
          else if (PS3.getAngle(Roll) < 130) {
            LeftFront.writeMicroseconds(1600);
            LeftRear.writeMicroseconds(1600);
            RightFront.writeMicroseconds(1000);
            RightRear.writeMicroseconds(1000);
          }
          else {
            LeftFront.writeMicroseconds(1500 + (160 - PS3.getAngle(Pitch)) * (500 / 50));
            LeftRear.writeMicroseconds(1500 + (160 - PS3.getAngle(Pitch)) * (500 / 50));
            RightFront.writeMicroseconds(1500 - (160 - PS3.getAngle(Pitch)) * (500 / 50));
            RightRear.writeMicroseconds(1500 - (160 - PS3.getAngle(Pitch)) * (500 / 50));
          }
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      Serial.print(F("\tRoll: "));
      Serial.print(PS3.getAngle(Roll));
      Serial.print(F("\tL2: "));
      Serial.print(PS3.getAnalogButton(L2));
      Serial.print(F("\tR2: "));
      Serial.print(PS3.getAnalogButton(R2));

      if (PS3.getAngle(Pitch) > 200) {
        scanfront();
        Serial.print(F("\tFront Sensor Distance: "));
        Serial.println(distancefront);
        if (distancefront >= FrontBack || distancefront == 0) {
          if (PS3.getAngle(Roll) > 230) {
            LeftFront.writeMicroseconds(1000);
            LeftRear.writeMicroseconds(1000);
            RightFront.writeMicroseconds(1600);
            RightRear.writeMicroseconds(1600);
          }
          else if (PS3.getAngle(Roll) < 130) {
            LeftFront.writeMicroseconds(1400);
            LeftRear.writeMicroseconds(1400);
            RightFront.writeMicroseconds(2000);
            RightRear.writeMicroseconds(2000);
          }
          else {
            LeftFront.writeMicroseconds(1500 - (PS3.getAngle(Pitch) - 200) * (500 / 50));
            LeftRear.writeMicroseconds(1500 - (PS3.getAngle(Pitch) - 200) * (500 / 50));
            RightFront.writeMicroseconds(1500 + (PS3.getAngle(Pitch) - 200) * (500 / 50));
            RightRear.writeMicroseconds(1500 + (PS3.getAngle(Pitch) - 200) * (500 / 50));
          }
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }
      if (PS3.getAngle(Roll) < 160 && PS3.getAngle(Pitch) > 160 && PS3.getAngle(Pitch) < 200) {
        scanleft();
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceleft);
        if (distanceleft >= LeftRight || distanceleft == 0) {
          LeftFront.writeMicroseconds(1500 + (160 - PS3.getAngle(Roll)) * (500 / 50));
          LeftRear.writeMicroseconds(1500 - (160 - PS3.getAngle(Roll)) * (500 / 50));
          RightFront.writeMicroseconds(1500 + (160 - PS3.getAngle(Roll)) * (500 / 50));
          RightRear.writeMicroseconds(1500 - (160 - PS3.getAngle(Roll)) * (500 / 50));
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }
      if (PS3.getAngle(Roll) > 200 && PS3.getAngle(Pitch) > 160 && PS3.getAngle(Pitch) < 200) {
        scanright();
        Serial.print(F("\tSensor Distance: "));
        Serial.println(distanceright);
        if (distanceright >= LeftRight || distanceright == 0) {
          LeftFront.writeMicroseconds(1500 - (PS3.getAngle(Roll) - 200) * (500 / 50));
          LeftRear.writeMicroseconds(1500 + (PS3.getAngle(Roll) - 200) * (500 / 50));
          RightFront.writeMicroseconds(1500 - (PS3.getAngle(Roll) - 200) * (500 / 50));
          RightRear.writeMicroseconds(1500 + (PS3.getAngle(Roll) - 200) * (500 / 50));
        }
        else {
          LeftFront.writeMicroseconds(1500);
          LeftRear.writeMicroseconds(1500);
          RightFront.writeMicroseconds(1500);
          RightRear.writeMicroseconds(1500);
        }
      }

      if (PS3.getButtonClick(START)) {
        Serial.print(F("\r\nStart"));
        if (PS3.PS3Connected) {
          PS3.setAllOff();
          PS3.setLedOn(LED1);
        }
        MotionMove = !MotionMove;
      }
    }
  }
}

void scanfront()
{
  delay(1);
  unsigned int uS = sonarfront.ping();
  distancefront = sonarfront.convert_cm(uS);
}

void scanleft()
{
  delay(1);
  unsigned int uS = sonarleft.ping();
  distanceleft = sonarleft.convert_cm(uS);
}

void scanright()
{
  delay(1);
  unsigned int uS = sonarright.ping();
  distanceright = sonarright.convert_cm(uS);
}

void scanback()
{
  delay(1);
  unsigned int uS = sonarback.ping();
  distanceback = sonarback.convert_cm(uS);
}

