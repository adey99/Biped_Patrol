/*THIS CODE EXECUTES TASK 3.2 AND ALSO THE RECEIVED VALUES BY THE XBEE CAN BE OBSERVED IN SERIAL MONITOR IF REQUIRED*/
//Defining Pin names for programming ease
#define lm_pwm 8   // lm_pwm= left motor pwm 
#define lm_a 22     //lm_a & lm_b = left_motor control
#define lm_b 23
#define rm_pwm 9    //rm_pwm= right motor pwm
#define rm_a 24      //rm_a & rm_b= right motor control
#define rm_b 25
#define magnet 26     // magnet= pin for electromagnet control
int packet[20],datalength,joystick_x,joystick_y,i,sw_val,val_x,val_y;   //these variables store and handle the packets received by the Xbee module
void setup() {  
  pinMode(lm_pwm,OUTPUT) ;                  //Setting up the pin modes for respective pins
  pinMode(lm_a,OUTPUT) ; 
  pinMode(lm_b,OUTPUT) ;
  pinMode(rm_pwm,OUTPUT) ;  
  pinMode(rm_a,OUTPUT) ; 
  pinMode(rm_b,OUTPUT) ;
  pinMode(magnet,OUTPUT) ;
  Serial.begin(115200);                     //terminal baud rate
  Serial1.begin(9600);                      //XBee baud rate
}

void loop() {                               //This function holds the code that controls the motors and electromagnet according to the packet received by Xbee
  if(Serial1.available()>0){                //if block only entered if some value is received by the Xbee
    int b = Serial1.read();
    if(b == 0x7E){                          //This if block reads the packets received by the Xbee( 0x7E is the starting byte of the transmitted packet from transmitter XBee) 
      packet[0] = b;
      packet[1] = readByte();     
      packet[2] = readByte();
      datalength = (packet[1]<<8)|packet[2];
      for(i=1;i<=datalength;i++){
        packet[2+i] = readByte();
      }
      packet[3+datalength] = readByte();      //checksum
      val_y = (packet[13]<<8)|packet[14];     // parsing the received values
      val_x = (packet[15]<<8)|packet[16];
      sw_val = (packet[12]>>3)&1;
    }
  }
  joystick_y = map(val_y,5,972,255,0);      //Mapping the joystick x coordinate values
  joystick_x = map(val_x,0,968,0,255);      //Mapping the joystick y coordinate values
  Serial.print(" Joystick Y : ");
  Serial.print(joystick_y);                 //Printing the received y coordinate value in serial monitor(for testing purpose)
  Serial.print(" Joystick X : ");
  Serial.print(joystick_x);                 //Printing the received x coordinate value in serial monitor(for testing purpose)
  Serial.print(" Magnet Switch : ");
  Serial.println(sw_val);                   //Printing the received digital value from the toggle switch(for testing purpose)
  if(joystick_y>200 && (joystick_x>110 && joystick_x<200)){
    forward();                              //This if block filters the values and calls the function that makes motors execute forward motion
  }
  else if(joystick_y<100 && (joystick_x>110 && joystick_x<200)){
    reverse();                              //This if block filters the values and calls the function that makes motors execute backward motion
  }
  else if((joystick_y<170 && joystick_y>130) && joystick_x<100){
    left();                                 //This if block filters the values and calls the function that makes motors execute left turn
  }
  else if((joystick_y<170 && joystick_y>130) && joystick_x>200){
    right();                                //This if block filters the values and calls the function that makes motors execute right turn
  }
  else{
    stopcar();                              //if none of the block matches, no movement occurs
  }
  magnet_op(sw_val);                        //Toggles the electromagnet on or off based on digital value sw_val
}

int readByte() {                            //Function for reading the packets received by Xbee
  while (true) {
    if (Serial1.available() > 0) {
      return Serial1.read();
    }
  }
}

void forward(){                             //Function for executing forward  motion
  digitalWrite(lm_a,HIGH) ;
  digitalWrite(lm_b,LOW) ;
  digitalWrite(rm_a,HIGH) ;
  digitalWrite(rm_b,LOW) ;
  analogWrite(lm_pwm,128) ;
  analogWrite(rm_pwm,128) ;
}

void reverse(){                             //Function for executing backward motion
  digitalWrite(lm_a,LOW) ;
  digitalWrite(lm_b,HIGH) ;
  digitalWrite(rm_a,LOW) ;
  digitalWrite(rm_b,HIGH) ;
  analogWrite(lm_pwm,128);
  analogWrite(rm_pwm,128);
}

void right(){                              //Function for executing right turn
  digitalWrite(lm_a,LOW) ;
  digitalWrite(lm_b,HIGH) ;
  digitalWrite(rm_a,HIGH) ;
  digitalWrite(rm_b,LOW) ;
  analogWrite(lm_pwm,128);
  analogWrite(rm_pwm,128);
}

void left(){                                //Function for executing left turn
  digitalWrite(lm_a,HIGH) ;
  digitalWrite(lm_b,LOW) ;
  digitalWrite(rm_a,LOW) ;
  digitalWrite(rm_b,HIGH) ;
  analogWrite(lm_pwm,128);
  analogWrite(rm_pwm,128);
}

void stopcar(){                               //Function for executing no operation of motors(i.e motors at rest)
  digitalWrite(lm_a,HIGH) ;
  digitalWrite(lm_b,HIGH) ;
  digitalWrite(rm_a,HIGH) ;
  digitalWrite(rm_b,HIGH) ;
}

void magnet_op(int val){                      //Function for toggling the electromagnet on or off based on the received digital value
  if(val == 1){
    digitalWrite(magnet,HIGH);          //Magnet On
  }
  else{
    digitalWrite(magnet,LOW);           //Magnet Off
  }
}
