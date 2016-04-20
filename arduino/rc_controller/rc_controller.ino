//*********************************************************************************
// www.Lynxmotion.com
// Basic code for 2WD rover using continuous rotation servos, controlled via PS2
// Right now, the library does NOT support hot pluggable controllers, meaning 
// you must always either restart your Arduino after you connect the controller, 
// or call config_gamepad(pins) again after connecting the controller.
//*********************************************************************************

#include <PS2X_lib.h>  //for v1.6
#include <Servo.h> 

// create PS2 Controller Class
PS2X ps2x; 
int error = 0; 
byte vibrate = 0;

// create servo objects to control the servos
Servo servoleft; // left servo
Servo servoright; // right servo


// 0 < write_value < 90 -> foward
// 90 < write_value < 180 -> backward
// write_value = 90 -> stop
#define Deadzone 10 //PS2 analog joystick Deadzone
#define BACKWARD 118
#define FORWARD 65
#define STOP 90

void setup(){
 Serial.begin(9600);

 // Attaches servos to digital pins 3 and 4 on the BotBoarduino
 servoleft.attach(3);
 servoright.attach(4); 
 
 pinMode(7, INPUT);
 
 error = ps2x.config_gamepad(9,7,8,6, true, true);   
}


void loop(){
  
  ps2x.read_gamepad();          //read controller 

      // This code uses the colored buttons on the right side of the joystick
      // Go forward
      if(ps2x.Button(PSB_GREEN)) {
       servoleft.write(FORWARD);
       servoright.write(FORWARD);

      }
      // Go backward
      else if(ps2x.Button(PSB_BLUE)){
       servoleft.write(BACKWARD);
       servoright.write(BACKWARD);

      }
      // Go left
      else if(ps2x.Button(PSB_PINK)) {
       servoleft.write(BACKWARD);
       servoright.write(FORWARD);
      }
      // Go right
      else if(ps2x.Button(PSB_RED)){
       servoleft.write(FORWARD);
       servoright.write(BACKWARD);

      }   
      else {
       servoleft.write(STOP);    // Adjust these values if the servos still move slightly
       servoright.write(STOP);
      }
      // To do debugging
      // Serial.println();
 delay(50);
     
}
