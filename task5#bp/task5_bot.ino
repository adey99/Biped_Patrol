 /*
 * Team Id: eYRC-BP#2557
 * Author List: Arunava Dey,Dipanjan Maity,Amiya Ghosh,Anick Bhattacharya
 * Filename: eYRC-BP#2557_Task5_Code.ino
 * Theme: Biped Patrol
 * Functions: void setup(),void loop(),void read_receiver(),void read_gyro()
 * Global Variables: JOYSTICK_X_MID,JOYSTICK_Y_MID,LM_PWM,RM_PWM,LM_A,LM_B,LM_ENCA,LM_ENCB,LM_OFFSET,RM_A,RM_B,RM_ENCA,RM_ENCB,RM_OFFSET,buzzer,magnet,gyro_address,gyro_x,acc_x,acc_y,acc_z,temp,gyro_x_cal,
 *                    cal_counter,first_angle,gyro_angle,accel_angle,digital_input,joystick_x,joystick_y,receive_byte,pid_p_gain,pid_i_gain,pid_d_gain,turning_speed,max_target_speed,self_balance_pid_setpoint,
 *                    pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error,pid_output_left,pid_output_right,left_motor,throttle_left_motor,throttle_counter_left_motor,throttle_left_motor_memory,
 *                    right_motor,throttle_right_motor,throttle_counter_right_motor,throttle_right_motor_memory,loop_timer,loop_counter
 */ 
#include<Wire.h>

//joystick middle offsets
#define JOYSTICK_X_MID 417
#define JOYSTICK_Y_MID 398

//pin configuration for left motor
#define LM_PWM 45
#define LM_A 11      //A high B low reverse
#define LM_B 12
#define LM_ENCA
#define LM_ENCB
#define LM_OFFSET 88

//pin configuration for right motor
#define RM_PWM 46
#define RM_A 8      //A high B low reverse
#define RM_B 7
#define RM_ENCA
#define RM_ENCB
#define RM_OFFSET 85

//buzzer and magnet pin configuration
#define buzzer 27
#define magnet 26

// Speed values
#define TILT_ANGLE_OFFSET -0.85
#define FULL_SPEED 60        // Position increment rate
#define FORWARD_SPEED 150
#define REVERSE_SPEED 160
#define SLOPE_SPEED 155
#define TURN_SPEED 150

#define COAST  0
#define FORWARD  1
#define BACK  2
#define LEFT  3
#define RIGHT 4
#define BRAKE 5

// Rotation Proportional Gains
#define LEFT_GAIN 5
#define RIGHT_GAIN 5

////////////////////////////////
//////encoder variables/////////
////////////////////////////////
volatile int encoderLpos = 0;
volatile int encoderRpos = 0;
byte lastLA,lastLB,lastRA,lastRB;

////////////////////////////////
//////////gyro variables////////
////////////////////////////////
int gyro_address=0x68;
int gyro_x,acc_x,acc_y,acc_z,temp;
double gyro_x_cal;
int cal_counter = 0;
bool first_angle = false;
float gyro_angle,accel_angle;
///////////////////////////////////////////
////////////receiver variables/////////////
///////////////////////////////////////////
bool digital_input;
int joystick_x=0,joystick_y=0;
byte receive_byte = 0b00000000;

/////////////////////////////////////////////
///////////loop time variables///////////////
/////////////////////////////////////////////
uint32_t loop_timer,last_task_time=0;

/////////////////////////////////////////////
//////// PID Structure Definition////////////
/////////////////////////////////////////////
typedef struct PID
{
  volatile float con_KP;      // Conservative proportional gain
  volatile float con_KI;      // Conservative integral gain
  volatile float con_KD;      // Conservative derivative gain
  
  volatile float agr_KP;      // Aggressive proportional gain
  volatile float agr_KI;      // Aggressive integral gain
  volatile float agr_KD;      // Aggressive derivative gain
  
  volatile float set_point;   // Set point value
  volatile float error;     // Error value
  volatile float current_position;    // Current position
  volatile float last_position; // Previous position
  
  volatile float integral;    // Integral sum
  volatile float derivative;    // Derivative term
  volatile float output;      // PID output 
  volatile int controller_direction;     // Controller direction
};
/////////////////////////////////////////////////
//////// Structure Initializations///////////////
/////////////////////////////////////////////////
PID angle    = {25, 3.2, 35, 30, 4, 40};
PID velocity = {15, 0, 4, 5, 0, 0, 0};
PID encoder  = {1.5, 0, 0, 8.2, 0, 0, 0};

////////////////////////////////////////////////////
/////////////////output variables///////////////////
////////////////////////////////////////////////////
float rotation_left=0, rotation_right=0;

float move_offset=0, max_angle_vel=4, max_angle_enc=2;
float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;

///////////////////flags/////////////////
bool STOP_FLAG = true;
bool ROTATION_FLAG = false;

/*
 * Function Name:setup
 * Input: NONE
 * Output: NONE
 * Logic: to initialize variables, pin modes, start using libraries.
 * Example Call: setup()
 */ 

void setup() {
  pinMode(buzzer,OUTPUT);
  digitalWrite(buzzer, HIGH);
  Serial.begin(57600);
  Serial2.begin(9600);
  Wire.begin();
  TWBR = 12;
  
  //Gyro configuration setting
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);    //+/-250 deg/sec 131
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);   //+/-4g 8192
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  while(Wire.available()<1);
  if(Wire.read() != 0x00){
    Serial.println("Gyro configuration failed.");
    while(1){
      digitalWrite(buzzer, LOW);
      delay(10);
    }
  }
  else{
    Serial.println("Gyro configuration successful.");
  }
  
  for(cal_counter=0;cal_counter<2000;cal_counter++){
    if(cal_counter % 137 == 0){
      Serial.print(".");
      //digitalWrite(buzzer, !digitalRead(buzzer));
    }
    Wire.beginTransmission(gyro_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 2);
    while(Wire.available()<2);
    gyro_x_cal += Wire.read()<<8|Wire.read();
    delayMicroseconds(3700);
  }
  if(cal_counter == 2000){
    gyro_x_cal /= 2000;
    Serial.print("Gyro x offset :");
    Serial.println(gyro_x_cal);
  }
  //delay(1000);
  digitalWrite(buzzer, LOW);
  pinMode(LM_A, OUTPUT);
  pinMode(LM_B, OUTPUT);
  pinMode(RM_A, OUTPUT);
  pinMode(RM_B, OUTPUT);
  pinMode(magnet, OUTPUT);

  digitalWrite(LM_A, LOW);
  digitalWrite(RM_A, LOW);
  digitalWrite(LM_B, LOW);
  digitalWrite(RM_B, LOW);
  digitalWrite(magnet, LOW);
  delay(1000);
  digitalWrite(buzzer, HIGH);

  angle.controller_direction = 1;
  velocity.controller_direction = 1;
  encoder.controller_direction = 1;
  cli();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  sei();
  loop_timer = micros() + 10000;  //simulate a loop @100Hz refresh rate
  last_task_time = millis();
}

/*
▪ * Function Name: loop
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function repeats the code inside it repeatedly till the arduino board has power supply.
▪ * Example Call: loop()
▪ */ 
void loop(){
  read_receiver();
  read_gyro();
  if(millis()-last_task_time >= 20){
    last_task_time = millis();
    volatile float left_current_count = encoderLpos;
    volatile float right_current_count = encoderRpos;
    left_RPM = (float)(((left_current_count - left_prev_count) * 60)/(0.02*270));
    right_RPM = (float)(((right_current_count - right_prev_count) * 60)/(0.02*270));
    left_prev_count = left_current_count;
    right_prev_count = right_current_count;

    if(receive_byte & B00000001){
      rotation_left  = -TURN_SPEED;
      rotation_right = TURN_SPEED;
      ROTATION_FLAG = true;
    }
    if(receive_byte & 0B00000010){
      rotation_left  = TURN_SPEED;
      rotation_right = -TURN_SPEED;
      ROTATION_FLAG = true;
    }
    
    if(receive_byte & 0B00001100 || !(receive_byte & 0B00000000)){
      rotation_left = 0;
      rotation_right = 0;
      ROTATION_FLAG = false;
    }
    if(receive_byte & B00000100){    ///backward motion
      encoder.set_point -= 0.1*FULL_SPEED;
      velocity.set_point = -REVERSE_SPEED;
      move_offset = -0.35;
      STOP_FLAG = false;
    }

    if(receive_byte & B00001000){    //forward motion
      encoder.set_point  += 0.1*FULL_SPEED;
      velocity.set_point  = FORWARD_SPEED;
      move_offset = 0.07;
      STOP_FLAG = false;
    }

    if(!(receive_byte & B00001111)){
      encoderLpos = encoderRpos = 0;
      encoder.set_point = encoder_count();
      velocity.set_point = 0;
      move_offset = 0;
      STOP_FLAG = true;
    }
    
    float encoder_KP = 0;
    encoder.current_position = encoder_count();
    encoder.error = encoder.set_point - encoder.current_position;
    if(!STOP_FLAG){
      encoder_KP = encoder.con_KP;
      max_angle_enc = 2;
    }
    else{
      encoder_KP = encoder.agr_KP;
      max_angle_enc = 2;
    }
    encoder.output = (encoder_KP*encoder.error*0.0001);
    encoder.output = encoder.controller_direction*constrain(encoder.output, -max_angle_enc,max_angle_enc);

    float velocity_KP = 0;
    velocity.current_position = (left_RPM + right_RPM)/2.0;
    velocity.error = velocity.set_point - velocity.current_position;
    if(!STOP_FLAG){
      velocity_KP = velocity.con_KP;
      if(abs(velocity.error) < 60)max_angle_vel = 2.5;
      if(abs(velocity.error) >= 60)max_angle_vel = 4;
    }

    else{
      velocity_KP = velocity.con_KP;
      max_angle_vel = 2;
    }

    velocity.derivative = velocity.current_position - velocity.last_position;
    velocity.integral += velocity.con_KI*0.001*velocity.error;
    velocity.integral = constrain(velocity.integral, -max_angle_vel,max_angle_vel);

    velocity.output = (velocity_KP*0.001*velocity.error) + velocity.integral - (velocity.con_KD*0.001*velocity.derivative);
    velocity.output = velocity.controller_direction * constrain(velocity.output, -max_angle_vel, max_angle_vel);

    velocity.last_position = velocity.current_position;

    //angle pid calcultion
    float angle_KP = 0,angle_KI = 0, angle_KD = 0;
    float angle_position;
    angle_position = angle.current_position;
    angle.set_point = TILT_ANGLE_OFFSET + move_offset + encoder.output + velocity.output;

    angle.error = angle.set_point - angle_position;
    if (abs(angle.error) >= 75)
    {
      angle.output = 0;
      return;
    }
    if (abs(angle.error) < 3.0)
    {
      angle_KP = angle.con_KP;
      angle_KI = angle.con_KI;
      angle_KD = angle.con_KD;
    }
    else
    {
      angle_KP = angle.agr_KP;
      angle_KI = angle.agr_KI;
      angle_KD = angle.agr_KD;
    }
    angle.derivative = angle_position - angle.last_position;
    angle.integral += angle_KI*angle.error;
    angle.integral = constrain(angle.integral, -255, 255);
    angle.output = (angle_KP*angle.error) + (angle.integral) - (angle_KD*angle.derivative);
    angle.output = constrain(angle.output, -255, 255);
    angle.last_position = angle_position;
    if(ROTATION_FLAG){
      encoderLpos = encoderRpos;
    }
    rotation_left = -1*LEFT_GAIN * (encoderLpos - encoderRpos)*0.05;
    rotation_right = RIGHT_GAIN * (encoderLpos - encoderRpos)*0.05;
    update_motors(angle.output, rotation_left, rotation_right);
    //Serial.println("inside 50Hz loop");
  }
  digitalWrite(magnet,digital_input);
  if(micros()>loop_timer){
    Serial.println("Loop time exceeded");
    while(1){
      delay(10);
    }
  }
  while(loop_timer>micros());
  loop_timer = micros()+10000;
}


/*
▪ * Function Name: read_receiver
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function reads the received values from the receiver XBee and processes them for further use
▪ * Example Call: read_receiver()
▪ */ 
void read_receiver(){
  int sum = 131;
  byte byte_discard, checksum;
  byte digital_data = 0;
  byte AD0[2] = {0,0}, AD1[2] = {0,0};
  if(Serial2.available() >= 18){
    if(Serial2.read() == 0x7E){
      for(int i=0;i<2;i++){
        byte_discard = Serial2.read();
      }
      if(Serial2.read() != 0x83){
        return;
      }
      for(int i=0;i<8;i++){
        sum += Serial2.read();
      }
      digital_data = Serial2.read();
      sum += digital_data;

      AD0[1] = Serial2.read();
      AD0[0] = Serial2.read();
      AD1[1] = Serial2.read();
      AD1[0] = Serial2.read();

      sum = sum+AD0[0]+AD0[1]+AD1[0]+AD1[1];
      checksum = 0xFF - (0xFF & (byte)sum);
      byte_discard = Serial2.read();
      if(byte_discard != checksum) return;

      if((digital_data & 0x08) == 0x08) digital_input = HIGH;
      else digital_input = LOW;

      joystick_x = AD1[1]<<8|AD1[0];
      joystick_y = AD0[1]<<8|AD0[0];

      if((joystick_y < (JOYSTICK_Y_MID + 20) && joystick_y > (JOYSTICK_Y_MID - 20)) && (joystick_x > (JOYSTICK_X_MID + 100))){
        receive_byte = 0b00000001;                   ///right motion(1)
      }
      else if((joystick_y < (JOYSTICK_Y_MID + 20) && joystick_y > (JOYSTICK_Y_MID - 20)) && (joystick_x < (JOYSTICK_X_MID - 100))){
        receive_byte = 0b00000010;                   ///left motion(2)
      }
      else if((joystick_x < (JOYSTICK_X_MID + 20) && joystick_x > (JOYSTICK_X_MID - 20)) && (joystick_y < (JOYSTICK_Y_MID - 100))){
        receive_byte = 0b00001000;                   ///forward motion(8)
      }
      else if((joystick_x < (JOYSTICK_X_MID + 20) && joystick_x > (JOYSTICK_X_MID - 20)) && (joystick_y > (JOYSTICK_Y_MID + 100))){
        receive_byte = 0b00000100;                   ///backward motion(4)
      }
      else{
        receive_byte = 0b00000000;                   ///no input(0)
      }
    }
  }
}

/*
▪ * Function Name: read_gyro
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function reads the values from MPU6050 and processes them for further use
▪ * Example Call: read_gyro()
▪ */ 

void read_gyro(){
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address,10);
  while(Wire.available()<10);
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  temp = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_x -= gyro_x_cal;

  if(abs(acc_y)<acc_z){
    accel_angle = asin((float)acc_y/acc_z) * 57.296;
    accel_angle -= 0.00;
  }
  if(!first_angle){
    gyro_angle = accel_angle;
    first_angle = true;
  }
  else{
    gyro_angle += gyro_x * 0.000076;
    gyro_angle = gyro_angle * 0.98 + accel_angle * 0.02;
  }
  angle.current_position = gyro_angle;
}

/*
▪ * Function Name: encoder_count
▪ * Input: NONE
▪ * Output: total number of encoder pulses from left and right encoder
▪ * Logic: This function returns the total number pulses from encoder counted by the Interrupt Service Routine
▪ * Example Call: float enc_count = encoder_count()
▪ */ 
float encoder_count(){
  return (encoderLpos + encoderRpos);
}

/*
▪ * Function Name: update_motors
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function sets PWM values and updates rotation offsets
▪ * Example Call: update_motors()
▪ */ 
void update_motors(float PID_output, float left_offset, float right_offset)
{
  float left_PWM=0, right_PWM=0;
  
  // Add rotation offsets and constrain the output
  left_PWM = constrain(PID_output+left_offset, -255, 255);
  right_PWM = constrain(PID_output+right_offset, -255, 255);
  
  // Drive the motors
  drive_motor(LEFT, left_PWM, LM_OFFSET);
  drive_motor(RIGHT, right_PWM, RM_OFFSET);
}

/*
▪ * Function Name: drive_motor
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function drives the motors from the PWM values set by update_motors() 
▪ * Example Call: drive_motor()
▪ */ 
void drive_motor(int motor, float PWM_value, float min_value)
{
  // Coast the motors
  if (PWM_value == 0)
  {
    set_motor_PWM(motor, 0);
    set_motor_mode(motor, COAST);
  }
  
  // Move the robot forward
  else if (PWM_value > 0)
  {
    PWM_value = map(PWM_value, 0, 255, min_value, 255);   // Map the PWM values
    set_motor_PWM(motor, (unsigned char)PWM_value);     // Set motor speed
    set_motor_mode(motor, FORWARD);             // Set motor direction
  }
  
  // Move the robot back
  else if (PWM_value < 0)
  {
    PWM_value = map(PWM_value, 0, -255, min_value, 255);  // Map the PWM values
    set_motor_PWM(motor, (unsigned char)PWM_value);     // Set motor speed
    set_motor_mode(motor, BACK);              // Set motor direction
  }
}

/*
▪ * Function Name: set_motor_PWM
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function sets the PWM values for motors for left and right turn
▪ * Example Call: set_motor_PWM()
▪ */ 
void set_motor_PWM(int motor, unsigned char motor_speed)
{
  if (motor==LEFT)  analogWrite(LM_PWM, motor_speed);
  if (motor==RIGHT) analogWrite(RM_PWM, motor_speed);
}

/*
▪ * Function Name: set_motor_mode
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function sets the modes for different motions of the bot i.e (forward,backward,left,right,brake,coast)
▪ * Example Call: set_motor_mode()
▪ */ 
void set_motor_mode(int motor, int mode)
{
  if (mode==FORWARD)  set_motor_pin(motor, LOW, HIGH);
  if (mode==BACK)   set_motor_pin(motor, HIGH, LOW);
  if (mode==COAST)  set_motor_pin(motor, LOW, LOW);
  if (mode==BRAKE)  set_motor_pin(motor, HIGH, HIGH);
  if (mode==LEFT){
    if(motor == LEFT){
      set_motor_pin(motor, HIGH,LOW);
    }
    if(motor == RIGHT){
      set_motor_pin(motor, LOW,HIGH);
    }
  }
  if(mode==RIGHT){
    if(motor == LEFT){
      set_motor_pin(motor,LOW,HIGH);
    }
    if(motor == RIGHT){
      set_motor_pin(motor,HIGH,LOW);
    }
  }
}
/*
▪ * Function Name: set_motor_pin
▪ * Input: NONE
▪ * Output: NONE
▪ * Logic: This function writes digital values(HIGH or LOW) to the left and right motors' pins
▪ * Example Call: set_motor_pin()
▪ */ 
void set_motor_pin(int motor, bool pin1, bool pin2)
{
  if (motor==LEFT){
    digitalWrite(LM_A,pin2);
    digitalWrite(LM_B,pin1);
  }
  if (motor==RIGHT){
    digitalWrite(RM_A,pin2);
    digitalWrite(RM_B,pin1);
  }
}

/*  Function Name: Interrupt Service Routine
 * Input: Pin Change Interrupt 0 (PCINT0)
 * Output: None
 * Logic: It counts the encoder pulses from both the right and left encoders
 * Example Call: ISR(PCINT0_vect) 
 *                It is called when a Pin change Interrupt Request 0 occurs.
 */

ISR(PCINT0_vect){
  if(lastLA == 0 && (PINB & B00000001)){          // for the detection of a CCW rotation of left motor
    lastLA = 1;                                   // CW rotation will be considered when signal of A 
    if(lastLB == 0 && !(PINB & B00000010)){       // will go 1 from 0 where B will remain at 0
      lastLB = 0;
      encoderLpos -= 1;
    }
  }
  else if(lastLA == 1 && !(PINB & B00000001)){    // for the detection of a CW rotation of left motor
    lastLA = 0;                                   // CCW rotation will be considered when signal of A
    if(lastLB == 0 && !(PINB & B00000010)){       // will go 0 from 1 where B will remain at 0
      lastLB = 0;
      encoderLpos += 1;
    }
  }
  if(lastRA == 0 && (PINB & B00000100)){          // for the detection of a CW rotation of right motor
    lastRA = 1;                                   // CW rotation will be considered when signal of A 
    if(lastRB == 0 && !(PINB & B00001000)){       // will go 1 from 0 where B will remain at 0
      lastRB = 0;
      encoderRpos += 1;
    }
  }
  else if(lastRA == 1 && !(PINB & B00000100)){    // for the detection of a CCW rotation of right motor
    lastRA = 0;                                   // CCW rotation will be considered when signal of A
    if(lastRB == 0 && !(PINB & B00001000)){       // will go 0 from 1 where B will remain at 0
      lastRB = 0;
      encoderRpos -= 1;
    }
  }
}
