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

const float Kp = 0.01;
const float Ki = 0;
const float Kd = 0.01;

int right_speed = 30;
int left_speed = 30;

float p_value;
float i_value;
float d_value;
float error;
float previous_error = 0;
float pid_value;

const int LED_RF = 41;

void setup(){
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  
  ECE3_Init();
  Serial.begin(9600);
  i_value = 0;
  delay(2000);

}



void loop(){
  error = 0;
  ECE3_read_IR(sensor_values);
  for(int i = 0; i < 8; i++) {                     // this block computes the error value and stores it in sensorValues[i]
    sensor_values[i] -= min_array[i];
    sensor_values[i] = sensor_values[i] * 1000 / max_array[i];
    error += sensor_values[i] * weights[i];
  }
  
  for (int i = 0; i < 8; i++) {
    
  }
  error /= 8.0;
  p_value = error * Kp;
  i_value += error * Ki;
  d_value = (error - previous_error) * Kd;
  pid_value = p_value + i_value + d_value;
  
  int current_right_speed = right_speed + pid_value;
  int current_left_speed = left_speed - pid_value;
  if (error > 2000) {
    current_right_speed = right_speed;
    current_left_speed = 0;
  }
  if (error < -2000) {
    current_right_speed = 0;
    current_left_speed = left_speed;
  }
  if (right_speed > 255) {
    right_speed = 255;
  }
  if (right_speed < 0) {
    right_speed = 0;
  }
  if (left_speed > 255) {
    left_speed = 255;
  }
  if (left_speed < 0) {
    left_speed = 0;
  }

  analogWrite(right_pwm_pin, current_right_speed);
  analogWrite(left_pwm_pin, current_left_speed);

  previous_error = error;
  
}
