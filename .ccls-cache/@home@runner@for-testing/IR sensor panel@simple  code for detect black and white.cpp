#define NUM_SENSORS 8 // number of sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // analog input pins for the sensors
int threshold = 400; // threshold value to distinguish between black and white

void setup() {
  Serial.begin(9600); // initialize serial communication
}

void loop() {
  int sensorValues[NUM_SENSORS];
 
  // read the sensor values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // output 0 for black, 1 for white
  Serial.print("Sensor values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    
    if (sensorValues[i] < threshold) {
      Serial.print("white ");
    } else {
      Serial.print("black ");
    }
  }
  Serial.println();

  delay(100);
}