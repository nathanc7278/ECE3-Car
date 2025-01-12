#include <ECE3.h>

uint16_t sensor_values[8];


const float min_array[] = {919, 708, 709, 709, 686, 640, 731, 732};
const float max_array[] = {1581, 1792, 1791, 1791, 1814, 1860, 1769, 1768};
const float weights[] = {-15, -14, -12, -8, 8, 12, 14, 15};


const int left_nslp_pin = 31;       // not sleep
const int left_dir_pin  = 29;       // rotation directions
const int left_pwm_pin  = 40;       // pulse width modulation (controls motor speed)

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const float Kp = 0.04;      
const float Ki = 0;
const float Kd = 0.004;      

const int base_speed = 30;             // (error margin)(Base speed) = Kp (maxErrorValue)

int current_right_speed;
int current_left_speed;

int sensor_max_reading;
bool has_turned = false;

float p_value;
float i_value = 0;
float d_value;
float error;
float previous_error = 0;
float pid_value;


void setup(){
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
  
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  
  
  ECE3_Init();
  Serial.begin(9600);
  resetEncoderCount_left();
  resetEncoderCount_right();
 
  delay(2000);

}


int average_encoder_count() {
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}


void loop(){
  error = 0;
  sensor_max_reading = 0;
  ECE3_read_IR(sensor_values);  
  for(int i = 0; i < 8; i++) {                     
    if(sensor_values[i] == 2500) {
      sensor_max_reading++;
    }
    sensor_values[i] -= min_array[i];
    sensor_values[i] = sensor_values[i] * 1000 / max_array[i];
    error += sensor_values[i] * weights[i];
  }
  if (average_encoder_count() > 6500 && average_encoder_count() < 7000 && !has_turned) { 
    if (sensor_max_reading == 8) {
      digitalWrite(left_dir_pin,LOW);
      digitalWrite(right_dir_pin,HIGH);
      analogWrite(right_pwm_pin, 50);
      analogWrite(left_pwm_pin, 50);
      delay(1500);
      digitalWrite(left_dir_pin,LOW);
      digitalWrite(right_dir_pin,LOW);
      has_turned = true;
    }
   
  }
  if (average_encoder_count() > 13500 && average_encoder_count() < 15000) { 
    if (sensor_max_reading == 8) {
      digitalWrite(left_nslp_pin,LOW);
      digitalWrite(right_nslp_pin,LOW);
    }
  }
  
  error /= 8.0;
  p_value = error * Kp;
  i_value += error * Ki;
  d_value = (error - previous_error) * Kd;
  pid_value = p_value + i_value + d_value;
  if (pid_value > base_speed * 2) {   //sharp left
    int saved = average_encoder_count();
    while (average_encoder_count() - saved < 4) { 
      digitalWrite(left_dir_pin, HIGH);
      analogWrite(right_pwm_pin, 0.25 * (base_speed + pid_value));
      analogWrite(left_pwm_pin, 0.25 * (base_speed + pid_value));
    }
    digitalWrite(left_dir_pin,LOW);
  } else if (pid_value < -1 * base_speed * 2) { // sharp right        
    int saved = average_encoder_count();
    while (average_encoder_count() - saved < 4) { 
      digitalWrite(right_dir_pin, HIGH);
      analogWrite(right_pwm_pin, 0.25 * (base_speed - pid_value));
      analogWrite(left_pwm_pin, 0.25 * (base_speed - pid_value));
    }
    digitalWrite(right_dir_pin, LOW);
  } else {    // straight line
    analogWrite(right_pwm_pin, base_speed + pid_value);
    analogWrite(left_pwm_pin, base_speed - pid_value);
  }
  previous_error = error;
  
}






//
//
//void do_turn(const int& left_speed, const int& right_speed, const int& num_encoder_ticks, int& current_left_speed, int& current_right_speed) {
//  int saved = average_encoder_count();
//  while(average_encoder_count() - saved < num_encoder_ticks){
//      current_left_speed = left_speed;
//      current_right_speed = right_speed; 
//      analogWrite(right_pwm_pin, current_right_speed);
//      analogWrite(left_pwm_pin, current_left_speed);
//    }
//}

//void do_u_turn(const int& left_speed, const int& right_speed, const int& num_encoder_ticks, int& current_left_speed, int& current_right_speed) {      // right sided u-turn
//  int saved = average_encoder_count();
//  while(average_encoder_count() - saved < num_encoder_ticks){
//      current_left_speed = left_speed;
//      current_right_speed = right_speed; 
//      digitalWrite(left_dir_pin,LOW);
//      digitalWrite(right_dir_pin,HIGH);
//      analogWrite(right_pwm_pin, current_right_speed);
//      analogWrite(left_pwm_pin, current_left_speed);
//    }
//    digitalWrite(left_dir_pin,LOW);
//    digitalWrite(right_dir_pin,LOW);
//}
//


//
//void do_u_turn2(const int& left_speed, const int& right_speed, const int& num_encoder_ticks, int& current_left_speed, int& current_right_speed) {      // right sided u-turn
//  int saved = average_encoder_count();
//  while(average_encoder_count() - saved < num_encoder_ticks){
//      current_left_speed = left_speed;
//      current_right_speed = right_speed; 
//      digitalWrite(left_dir_pin,HIGH);
//      digitalWrite(right_dir_pin,LOW);
//      analogWrite(right_pwm_pin, current_right_speed);
//      analogWrite(left_pwm_pin, current_left_speed);
//    }
//    digitalWrite(left_dir_pin,LOW);
//    digitalWrite(right_dir_pin,LOW);
//}
//











//
//  if (starting_encoder_count + average_encoder_count() == 500) {
//    do_turn(30, 0, 500, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 1700) {
//    do_u_turn2(20, 30, 200, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 3600) {
//    do_u_turn(30, 20, 350, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 6300) {
//    do_turn(0, 30, 100, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 6500) {
//    do_u_turn(30,30, 375, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 6950) {
//    do_turn(30, 0, 50, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 9400) {
//    do_u_turn2(20, 30, 350, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 11400) {
//    do_u_turn(30, 20, 200, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 12000) {
//    do_turn(0, 30, 100, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 12100) {
//    do_turn(30, 30, 100, current_left_speed, current_right_speed);
//  }
//  if (starting_encoder_count + average_encoder_count() == 12200) {
//    do_turn(0, 30, 600, current_left_speed, current_right_speed);
//  }
//  if(starting_encoder_count + average_encoder_count() == 13700) {
//    while (true){ 
//      analogWrite(right_pwm_pin, 0);
//      analogWrite(left_pwm_pin, 0);
//    }
//  }



//  if(sensor_max_reading == 8 && currently_in_crosspiece == false) {
//    currently_in_crosspiece = true;
//  } else if (currently_in_crosspiece == true && sensor_max_reading != 8) {
//    crosspiece_count++;
//    currently_in_crosspiece = false;
//  }
//  if (crosspiece_count == 3) {
//    digitalWrite(left_dir_pin,LOW);
//    digitalWrite(right_dir_pin,HIGH);
//    analogWrite(right_pwm_pin, 50);
//    analogWrite(left_pwm_pin, 50);
//    delay(1500);
//    crosspiece_count++;
//    digitalWrite(left_dir_pin,LOW);
//    digitalWrite(right_dir_pin,LOW);
//  } 

//  if (crosspiece_count == 7) {
//    digitalWrite(left_nslp_pin,LOW);
//    digitalWrite(right_nslp_pin,LOW);
//  }
