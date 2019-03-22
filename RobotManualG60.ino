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
Servo gripper;


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

int low_speed = 110;
int normal_speed = 180;
int fast_speed  = 200;

int hand_data  = 50;

//Servo1 Arm
int pin_arm = 42;
//Servo2 Hand
int pin_hand = 5;
//Servo3 Gripper
int pin_gripper = 22;

//============= ตัวแปร ==============
float joybnt_x_y_a_b = 0.0;
float joybnt_u_d_l_r = 0.0;
float joy_Analog_click = 0.0;
float joy_AnalogL_axisX= 0.0;
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
        pinMode(ena,OUTPUT);
        pinMode(in1,OUTPUT);
        pinMode(in2,OUTPUT);
        pinMode(enb,OUTPUT);
        pinMode(in3,OUTPUT);
        pinMode(in4,OUTPUT);
        
        arm.attach(pin_arm);
        hand.attach(pin_hand);
        gripper.attach(pin_gripper);
        arm.write(90);
        gripper.write(45);
        
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
        float Joybutton_x_y_a_b = map(JoyEvents.X, 0, 0xFF, 30.0f, 250.0f);
        float JoyAnalogL_axisY = map(JoyEvents.Z2, 0, 0xFF, -250.0f, 250.0f);
        float JoyAnalog_click = map(JoyEvents.Y, 0, 0xFF, 30.0f, 250.0f);
        
        float Joybutton_u_d_l_r = map(JoyEvents.Z1, 0, 0xFF, 30.0f, 250.0f);
        float JoyAnalogL_axisX= map(JoyEvents.Rz, 0, 0xFF, -250.0f, 250.0f);

       
       joybnt_x_y_a_b = Joybutton_x_y_a_b; 
       joybnt_u_d_l_r=Joybutton_u_d_l_r;
       joy_Analog_click=JoyAnalog_click;
       joy_AnalogL_axisX=JoyAnalogL_axisX;
       joy_AnalogL_axisY=JoyAnalogL_axisY;
       
       if(c == 1){
           Serial.print(Joybutton_x_y_a_b);     
           Serial.print("   ");
           Serial.print(Joybutton_u_d_l_r);
           Serial.print("   ");
           Serial.print(JoyAnalog_click);     
           Serial.print("   ");
           Serial.print(JoyAnalogL_axisX);
           Serial.print("   ");
          }

          
         if(check_button_up(joybnt_u_d_l_r) and check_button_LT(joybnt_x_y_a_b) ){
             Serial.println("Func robot_go_fast");
             func_1_robot_go(fast_speed);
          }
          else if(check_button_up(joybnt_u_d_l_r) and check_button_LB(joybnt_x_y_a_b) ){
             Serial.println("Func robot_go_slow");
             func_1_robot_go(low_speed);
          }
          else if((joy_AnalogL_axisY>-251.0)and(joy_AnalogL_axisY<0) and check_button_LT(joybnt_x_y_a_b) ){
             Serial.println("Func robot_arm_up_down_analog");
      
              arm.write(0);
          }else if((joy_AnalogL_axisY>0)and(joy_AnalogL_axisY<251.0) and check_button_LT(joybnt_x_y_a_b) ){
             Serial.println("Func robot_arm_up_down_analog");
      
             arm.write(160);
          }
          else if((joy_AnalogL_axisX>-251.0)and(joy_AnalogL_axisY<251.0) and check_button_LT_RT(joybnt_x_y_a_b)){
             Serial.println("Func robot_hand_left_analog");
              hand_data = hand_data + 10;
              delay(100);
              if(hand_data <=0) hand_data = 0;
              if(hand_data >=90) hand_data = 90;
              hand.write(hand_data); 
          }
          else if((joy_AnalogL_axisX>-251.0)and(joy_AnalogL_axisY<251.0) and check_button_LT_RB(joybnt_x_y_a_b)){
             Serial.println("Func robot_hand_right_analog");
              hand_data = hand_data - 10;
              delay(100);
              if(hand_data <=0) hand_data = 0;
              if(hand_data >=90) hand_data = 90;
              hand.write(hand_data);
          }
          else if((joy_AnalogL_axisX>-251.0)and(joy_AnalogL_axisY<251.0) and check_button_YLT(joybnt_x_y_a_b)){
             Serial.println("Func robot_Griper On");
               gripper.write(0);
          }
          else if((joy_AnalogL_axisX>-251.0)and(joy_AnalogL_axisY<251.0) and check_button_XLT(joybnt_x_y_a_b)){
             Serial.println("Func robot_Griper Off");
             gripper.write(45);
          }
         else if(check_button_right(joybnt_u_d_l_r)){
           Serial.println("Func robot_turn_left");
           analogWrite(enb,normal_speed);  // Motor Left  ถอยหลัง
           digitalWrite(in3,HIGH);
           digitalWrite(in4,LOW);
           analogWrite(ena,normal_speed);   // Motor ขวาเดิน หน้า
           digitalWrite(in1,LOW);
           digitalWrite(in2,HIGH);
          }
          else if(check_button_left(joybnt_u_d_l_r)){
             Serial.println("Func robot_turn_right");
             analogWrite(ena,normal_speed);  // Motor ขวา ถอยหลัง
             digitalWrite(in1,HIGH);
             digitalWrite(in2,LOW);
             analogWrite(enb,normal_speed);   // Motor Left เดิน หน้า
             digitalWrite(in3,LOW);
             digitalWrite(in4,HIGH);
          }
          else if(check_button_down(joybnt_u_d_l_r)){
             Serial.println("Func robot_back");
             analogWrite(ena,normal_speed);  // Motor ขวา ถอยหลัง
             digitalWrite(in1,HIGH);
             digitalWrite(in2,LOW);
             analogWrite(enb,normal_speed);  // Motor Left  ถอยหลัง
             digitalWrite(in3,HIGH);
             digitalWrite(in4,LOW);
           
          }else if(joybnt_u_d_l_r == 36.0){
             Serial.println("Func robot stop");
            analogWrite(ena,0);
            analogWrite(enb,0);
          
          }else if(check_button_up(joybnt_u_d_l_r)){
           Serial.println("Func robot_go ");
           func_1_robot_go(normal_speed);
         }
              
    
}


//==============================================
int check_button_X(int key){
    if(key == keyX){
      return 1;
    }else return 0;
}

//==============================================
int check_button_Y(int key){
    if(key == keyY){
      return 1;
    }else return 0;
}

//==============================================
int check_button_A(int key){
    if(key == keyA){
      return 1;
    }else return 0;
}

//==============================================
int check_button_B(int key){
    if(key == keyB){
      return 1;
    }else return 0;
}

//==============================================
int check_button_up(int key){
    if(key == keyup){
      return 1;
    }else return 0;
}

//==============================================
int check_button_down(int key){
    if(key == keydown){
      return 1;
    }else return 0;
}

//==============================================
int check_button_left(int key){
    Serial.println(key);
    if(key == keyleft){
      return 1;
    }else return 0;
}

//==============================================
int check_button_right(int key){
    Serial.println(key);
    if(key == keyright){
      return 1;
    }else return 0;
}

//==============================================
int check_button_LB(int key){
    if(key == keyLB){
      return 1;
    }else return 0;
}

//==============================================
int check_button_LT(int key){
    if(key == keyLT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_RB(int key){
    if(key == keyRB){
      return 1;
    }else return 0;
}

//==============================================
int check_button_RT(int key){
    if(key == keyRT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_XLB(int key){
    if(key == keyXLB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_XLT(int key){
    if(key == keyXLT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_XRB(int key){
    if(key == keyXRB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_XRT(int key){
    if(key == keyXRT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_YLB(int key){
    if(key == keyYLB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_YLT(int key){
    if(key == keyYLT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_YRB(int key){
    if(key == keyYRB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_YRT(int key){
    if(key == keyYRT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_ALB(int key){
    if(key == keyALB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_ALT(int key){
    if(key == keyALT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_ARB(int key){
    if(key == keyARB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_ART(int key){
    if(key == keyART){
      return 1;
    }else return 0;
}

//==============================================
int check_button_BLB(int key){
    if(key == keyBLB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_BLT(int key){
    if(key == keyBLT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_BRB(int key){
    if(key == keyBRB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_BRT(int key){
    if(key == keyBRT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_RB_RT(int key){
    if(key == keyRB_RT){
      return 1;
    }else return 0;
}
//==============================================
int check_button_LB_LT(int key){
    if(key == keyLB_LT){
      return 1;
    }else return 0;
}

//==============================================
int check_button_LB_RB(int key){
    if(key == keyLB_RB){
      return 1;
    }else return 0;
}
//==============================================
int check_button_LT_RT(int key){
    if(key == keyLT_RT){
      return 1;
    }else return 0;
}

int check_button_LT_RB(int key){
    if(key == keyLT_RB){
      return 1;
    }else return 0;
}

//========================== robot Function =====================
// func 1 Robot go ปกติ
void func_1_robot_go(int sp){
   analogWrite(ena,sp);   // Motor ขวาเดิน หน้า
   digitalWrite(in1,LOW);
   digitalWrite(in2,HIGH);
   
   analogWrite(enb,sp);   // Motor Left เดิน หน้า
   digitalWrite(in3,LOW);
   digitalWrite(in4,HIGH);
   
}
// func 2 Robot go เร็ว
void func_2_robot_go_fast(){
  
}
// func 3 Robot go ช้า
void func_3_robot_go_slow(){
  
}
// func 4 Robot arm analog
void func_4_robot_arm_analog(int inData){
  arm.write(inData);  // ยกลง
}
// func 5 Robot hand left 
void func_5_robot_hand_left(){
  
}
// func 6 Robot hand right 
void func_6_robot_hand_right(){
  
}
// func 7 Robot หนีบ
void func_7_robot_Griper_on(){
  
}
// func 8 Robot ปล่อย
void func_8_robot_Griper_off(){
  
}
