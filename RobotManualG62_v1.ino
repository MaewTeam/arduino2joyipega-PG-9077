#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <Servo.h>
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include "hidjoystickrptparser.h"

Servo arm;
Servo hand;
Servo gripper1;
Servo gripper2;
Servo gripper3;


USB Usb;

USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

//============= pin ===============
//MotorA ขวา
int ena = 2;
int in1 = 30;
int in2 = 31;
//MotorB ซ้าย
int enb = 3;
int in3 = 32;
int in4 = 33;


int normal_speed = 180;
int fast_speed  = 200;


//Servo1 Arm
int pin_arm = 40;
//Servo2 Gripper
int pin_gripper1 = 41;
//Servo3 Gripper
int pin_gripper2 = 42;
//Servo4 Gripper
int pin_gripper3 = 43;

void setup() {
  Serial.begin(115200);
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  arm.attach(pin_arm);
  gripper1.attach(pin_gripper1);
  gripper2.attach(pin_gripper2);
  gripper3.attach(pin_gripper3);
  arm.write(90);
  gripper1.write(45);
  gripper2.write(45);
  gripper3.write(45);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
  int c = 0;
  Usb.Task();
 
   if (JoyEvents.X == 0xC0 and JoyEvents.Z1 == 0x00) {
      Serial.println("Func test1111111111 ");
      
  }
}



//========================== robot Function =====================
// func 1 Robot go ปกติ
void func_1_robot_go(int sp) {
  analogWrite(ena, sp);  // Motor ขวาเดิน หน้า
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(enb, sp);  // Motor Left เดิน หน้า
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

}
// func 2 Robot go เร็ว
void func_2_robot_go_fast() {

}
// func 3 Robot go ช้า
void func_3_robot_go_slow() {

}
// func 4 Robot arm analog
void func_4_robot_arm_analog(int inData) {
  arm.write(inData);  // ยกลง
}

// func 7 Robot หนีบ
void func_7_robot_Griper_on() {

}
// func 8 Robot ปล่อย
void func_8_robot_Griper_off() {

}
