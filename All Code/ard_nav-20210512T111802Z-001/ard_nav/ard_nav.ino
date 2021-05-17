/* 
  This program is the interpretation routine of standard output protocol of TFmini product on Arduino.
  For details, refer to Product Specifications.
  For Arduino boards with only one serial port like UNO board, the function of software visual serial port is to be used. 
*/ 
int verb = 0;
const int pwm = 5 ; //initializing pin 2 as pwm
const int pwm2 = 6;
const int in_1 = A0 ;
const int in_2 = A1 ;
const int in_3 = A2 ;
const int in_4 = A3 ;
int encA, encB;
int x;

#include <Wire.h>
#include "VL53L1X.h"

VL53L1X front;
VL53L1X back;

/* For Arduinoboards with multiple serial ports like DUEboard, interpret above two pieces of code and directly use Serial1 serial port*/


void set_tof() {
  pinMode(4, OUTPUT); // back
  pinMode(12, OUTPUT); //side
  
  digitalWrite(4, LOW);
  digitalWrite(12, LOW);

 // Initalisiert I2C
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);
  Serial.begin (115200);
  Serial.setTimeout(1);

  digitalWrite(4,HIGH);
  delay(150);
  back.init();
  //Serial.println("01");
  delay(100);
  back.setAddress(0x33);
  //Serial.println("02");


  digitalWrite(12, HIGH);
  delay(150);
  //Serial.println("09");
  front.init();
  //Serial.println("10");
  delay(100);


  

  front.setDistanceMode(VL53L1X::Short);
  front.setMeasurementTimingBudget(50000);
  front.startContinuous(50);
  front.setTimeout(100);

  back.setDistanceMode(VL53L1X::Medium);
  back.setMeasurementTimingBudget(50000);
  back.startContinuous(50);
  back.setTimeout(100);
  
  
  delay(150);
  //Serial.println("addresses set");

  //Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      if(verb){
        Serial.print ("Found address: ");
        Serial.print (i, DEC);
        Serial.print (" (0x");
        Serial.print (i, HEX);
        Serial.println (")");
      }
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  if(verb){
    Serial.println ("Done.");
    Serial.print ("Found ");
    Serial.print (count, DEC);
    Serial.println (" device(s).");
  }
}

int read_front_tof() {
  front.read();
  int distance_from_wall = front.ranging_data.range_mm;
  return distance_from_wall;
}

int read_back_tof() {
  back.read();
  int distance_to_target = back.ranging_data.range_mm;
  return distance_to_target;
}

void extInterruptInit(){
  DDRD &= ~(0x0C);     // Clear the PD2 and PD3 pin, PD2 & PD3 (PCINT0 & 1 pins) are now inputs
  //PORTD &= ~(0x04);    // turn On the Pull-up, PD2 is now an input with pull-up enabled
  EICRA |= 0x05; // rising edge detection on INT0 & INT1
}

void extInterruptEnable(){
  EIMSK |= (1 << INT0) | (1 << INT1); // Enable INT0 interrupt
}

void extInterruptDisable(){
  EIMSK &= ~(0x03); // Disable INT0 interrupt
}

void setup() {
   extInterruptInit();
   extInterruptEnable();
   set_tof();
   encA = 0;
   encB = 0;
   DDRB |= 0x20;
   PORTB |= 0x20;
   
   pinMode(pwm,OUTPUT) ; //we have to set PWM pin as output
   pinMode(pwm2,OUTPUT);
   
   pinMode(in_1,OUTPUT) ; //Logic pins are also set as output
   pinMode(in_2,OUTPUT) ;
   pinMode(in_3, OUTPUT) ;
   pinMode(in_4, OUTPUT) ;

   pinMode(2, INPUT_PULLUP);
   pinMode(3, INPUT_PULLUP);

   pinMode(LED_BUILTIN, OUTPUT);

}


void brake() {
   digitalWrite(in_1,HIGH) ;
   digitalWrite(in_2,HIGH) ;
   digitalWrite(in_3,HIGH) ;
   digitalWrite(in_4, HIGH) ;
}

void forward() {
   digitalWrite(in_1,LOW) ;
   digitalWrite(in_2,HIGH) ;
   digitalWrite(in_3,HIGH) ;
   digitalWrite(in_4, LOW) ;
}

void rotate_right() {
   digitalWrite(in_1,HIGH) ;
   digitalWrite(in_2,LOW) ;
   digitalWrite(in_3,HIGH) ;
   digitalWrite(in_4, LOW) ;
}

int target_distance = 0;

void distance_map(char station) {

  switch(station) {
    case 'A': 
      target_distance = 1320;
      break;
    case 'B':
      target_distance = 970;
      break;
    case 'C':
      target_distance = 645;
      break;
    case 'D': 
      target_distance = 310;
      break;
    case 'E':
      target_distance = 125;
      break;
    case 'F':
      target_distance = 100;
      break;
    case 'G': 
      target_distance = 250;
      break;
    case 'H':
      target_distance = 400;
      break;
    default:
      target_distance = read_front_tof();
      brake();
  }
}

int count = 0;
char input;
char prev_input = 'A';
float enc_weight = 1.28;
float adjustment = 1.25;
int wheel_speed = 150;

void navigation_loop_one(char input, int tof_distance) { //if flag = 1, tof_front | if flag = 2, tof_back
  
    distance_map(input);
    
    while((tof_distance - target_distance) > 20) {
       if(count%100 == 0) {
        encA = 1;
        encB = 1;
        count = 0;
       }
       
       count++;
       forward();
       float enc_ratio = (enc_weight*encA)/(1.0*encB);
       
       if(enc_ratio > .95 && enc_ratio < 1.05) {
          analogWrite(pwm2,wheel_speed) ;
          analogWrite(pwm, wheel_speed * adjustment);
       }else if(enc_ratio > 1.05) {
          //Serial.print("Correnting A > B\n");
          analogWrite(pwm2,0) ;
          analogWrite(pwm, wheel_speed);
       }else if(enc_ratio < .95) {
          //Serial.print("Correnting A <  B\n");
          analogWrite(pwm2, wheel_speed);
          analogWrite(pwm, 0);
       }

       tof_distance = read_front_tof();

     if(verb){ 
         Serial.print("Distance to cover: ");
         Serial.print(tof_distance - target_distance);
         Serial.print("\t");
         Serial.print(encA);
         Serial.print(", ");
         Serial.print(encB);
         Serial.print("\n");
     }
    }
    encA = 1;
    encB = 1;
    prev_input = input;
}

void navigation_loop_two(char input, int tof_distance) { //if flag = 1, tof_front | if flag = 2, tof_back
  
    //distance_map(input);
    
    while(target_distance - tof_distance > 20) {
       if(count%100 == 0) {
        encA = 1;
        encB = 1;
        count = 0;
       }
       
       count++;
       forward();
       float enc_ratio = (enc_weight*encA)/(1.0*encB);
       
       if(enc_ratio > .95 && enc_ratio < 1.05) {
          analogWrite(pwm2,wheel_speed) ;
          analogWrite(pwm, wheel_speed * adjustment);
       }else if(enc_ratio > 1.05) {
          //Serial.print("Correnting A > B\n");
          analogWrite(pwm2,0) ;
          analogWrite(pwm, wheel_speed);
       }else if(enc_ratio < .95) {
          //Serial.print("Correnting A <  B\n");
          analogWrite(pwm2, wheel_speed);
          analogWrite(pwm, 0);
       }


     tof_distance = read_back_tof();

     if(verb){ 
         Serial.print("Distance to cover: ");
         Serial.print(tof_distance - target_distance);
         Serial.print("\t");
         Serial.print(encA);
         Serial.print(", ");
         Serial.print(encB);
         Serial.print("\n");
     }
    }
    encA = 1;
    encB = 1;
    prev_input = input;
}

void corner_condition() {
   rotate_right();
   analogWrite(pwm2, 200);
   analogWrite(pwm, 200);
   delay(1300);
}
/*
void loop() { 

  while (!Serial.available());
  x = Serial.readString().toInt();
  Serial.print(x+1);
}
*/

void loop() { 
  Serial.flush();
  if(Serial.available()) {
    input = (char)Serial.read();
    //Serial.print(input);
    //distance_map(input);
  }else{
    brake();
  }

  int front_dist = read_front_tof();
  int back_dist = read_back_tof();

if(verb){
    Serial.print("Front = ");
    Serial.print(front_dist);
    Serial.print(", Back = ");
    Serial.print(back_dist);
    Serial.print("\n");
}
  enc_weight = .9;
  adjustment = 1.1;
  wheel_speed = 185;
  
  
  if(input != prev_input) 
  { 
     if(input == 'A') {
      //digitalWrite(LED_BUILTIN, HIGH);
        prev_input = input;
        Serial.print("d");
     }
     
     else if(input <= 'E') 
     {
      //digitalWrite(LED_BUILTIN, HIGH);
      
       // Serial.print("navigating to station");
       // Serial.print(input);
       // Serial.print("\n");
        
        navigation_loop_one(input, front_dist);
    
     } else { //if(input > 'E')
    
       if(prev_input <= 'E') 
       {
      
          navigation_loop_one('E', front_dist);
          delay(5); 
          corner_condition();
          distance_map(input);
          delay(1);
       }
       back_dist = read_back_tof();
       navigation_loop_two(input, back_dist);

     }
     Serial.print("d");
  }
}


ISR(INT0_vect)
{
  encA++;
};

ISR(INT1_vect)
{
  encB++;
};
/*
Download the program into Arduino board and power on monitor for serial port. Then real-time distance values measured by LiDAR and corresponding signal strength can be viewed.
In addition, data curve can be viewed in curve plotter for serial port, provided, however, that the above code regarding printing for serial port should be modified:
//  Serial.print("dist = ");
Serial.print(dist);//output measure distance value of LiDAR
Serial.print(' ');
//  Serial.print("strength = ");        
Serial.print(strength);//output signal strength value
Serial.print('\n');
Reinterpret and download the program into Arduino board and power on the curve plotter. Then two curves representing dist and strength can be viewed.
*/
