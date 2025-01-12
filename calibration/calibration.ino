#include <ECE3.h>

uint16_t sensorValues[8];

int current_position = -40;
int increment_position = 4;

int number_samples = 5;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
}


void loop()
{
  int summed_values[8] = {0};

  // Take the average of 5 consecutive values for each sensor
  for (int j = 0; j < number_samples; j++){
    // Read raw sensor values
    ECE3_read_IR(sensorValues);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++)
    {
      summed_values[i] += sensorValues[i];
    }
  }

  // Print current position at start of each column
  Serial.print(current_position);
  Serial.print("\t");

  // Print average values (average value = summed_values / number_samples
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(summed_values[i] / number_samples);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();

  // Increment current position by the increment value
  current_position += increment_position;

  // Add a delay to give yourself time to move the paper
  delay(5000);
}
