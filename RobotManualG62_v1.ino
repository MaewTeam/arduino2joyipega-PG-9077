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

//============= ตัวแปร ==============
float joybnt_x_y_a_b = 0.0;
float joybnt_u_d_l_r = 0.0;
float joy_Analog_click = 0.0;
float joy_AnalogL_axisX = 0.0;
float joy_AnalogL_axisY = 0.0;


int normala = 30;
int normalb = 36;
int normalc = 30;
int normald =  0;
int normale =  0;


float keyX = 36.0;
float keyY = 30.0;
float keyA = 33.0;
float keyB = 31.0;
float keyup = 30.0;
float keydown = 33.0;
float keyleft = 35.0;
float keyright = 31.0;

float keyup_A = 30.0;
float keydown_A = 33.0;
float keyleft_A = 35.0;
float keyright_A = 31.0;

float keyXLB = 43.0;
float keyXLT = 92.0;
float keyXRB = 57.0;
float keyXRT = 140.0;

float keyYLB = 44.0;
float keyYLT = 86.0;
float keyYRB = 58.0;
float keyYRT = 141.0;

float keyALB = 47.0;
float keyALT = 88.0;
float keyARB = 61.0;
float keyART = 143.0;

float keyBLB = 45.0;
float keyBLT = 86.0; //จะซ้ำกับปุ้ม LT+Y
float keyBRB = 59.0;
float keyBRT = 142.0;

float keyLB_RB = 71.0;
float keyLT_RT = 195.0;
float keyLB_LT = 99.0;
float keyRB_RT = 168.0;
float keyLT_RB = 112.0;

int keyLB = 43.0;
int keyLT = 85.0;
int keyRB = 57.0;
int keyRT = 140.0;


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


//==============================================
int check_button_X(int key) {
  if (key == keyX) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_Y(int key) {
  if (key == keyY) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_A(int key) {
  if (key == keyA) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_B(int key) {
  if (key == keyB) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_up(int key) {
  if (key == keyup) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_down(int key) {
  if (key == keydown) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_left(int key) {
  Serial.println(key);
  if (key == keyleft) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_right(int key) {
  Serial.println(key);
  if (key == keyright) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_LB(int key) {
  if (key == keyLB) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_LT(int key) {
  if (key == keyLT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_RB(int key) {
  if (key == keyRB) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_RT(int key) {
  if (key == keyRT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_XLB(int key) {
  if (key == keyXLB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_XLT(int key) {
  if (key == keyXLT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_XRB(int key) {
  if (key == keyXRB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_XRT(int key) {
  if (key == keyXRT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_YLB(int key) {
  if (key == keyYLB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_YLT(int key) {
  if (key == keyYLT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_YRB(int key) {
  if (key == keyYRB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_YRT(int key) {
  if (key == keyYRT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_ALB(int key) {
  if (key == keyALB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_ALT(int key) {
  if (key == keyALT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_ARB(int key) {
  if (key == keyARB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_ART(int key) {
  if (key == keyART) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_BLB(int key) {
  if (key == keyBLB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_BLT(int key) {
  if (key == keyBLT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_BRB(int key) {
  if (key == keyBRB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_BRT(int key) {
  if (key == keyBRT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_RB_RT(int key) {
  if (key == keyRB_RT) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_LB_LT(int key) {
  if (key == keyLB_LT) {
    return 1;
  } else return 0;
}

//==============================================
int check_button_LB_RB(int key) {
  if (key == keyLB_RB) {
    return 1;
  } else return 0;
}
//==============================================
int check_button_LT_RT(int key) {
  if (key == keyLT_RT) {
    return 1;
  } else return 0;
}

int check_button_LT_RB(int key) {
  if (key == keyLT_RB) {
    return 1;
  } else return 0;
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
