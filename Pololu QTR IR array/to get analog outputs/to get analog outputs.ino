int sensorPins[] = {A3, A4, A5, A6, A7, A8, A9, A10}; // Change these to the pin numbers where you connected the sensor OUT pins
int numSensors = 8; // Change this to the number of sensors you're using

void setup() {
  Serial.begin(9600); // Initialize serial communication
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT); // Set the sensor pins as inputs
  }
}

void loop() {
  int sensorValues[numSensors]; // Create an array to store the sensor values
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]); // Read the sensor values
  }
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]); // Print the sensor values to the serial monitor
    Serial.print('\t'); // Add a tab character for formatting
  }
  Serial.println(); // Add a line break
  delay(100); // Wait for a short time before reading the sensor again
}