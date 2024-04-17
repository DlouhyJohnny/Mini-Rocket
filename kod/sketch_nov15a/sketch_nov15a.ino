#include <Arduino.h>
#include <EEPROM.h>

// Sample values for the variables
int SPEED;
float ODO = 0;
float TRIP = 0;
float VOLT = 0;
float POWER = 0;
float CONSUMPTION = 0;
float RANGE = 0;
String BLINKERL = "N";
String BLINKERR = "N";

const int hallSensor1 = 17;  

volatile unsigned long pulseCount = 0;
unsigned long prevTime = 0;

const int powerSensor = A0;  // Analog input pin
const int voltageSensor = A1;
int calibrationConstant;

const int numReadings = 20;        // Number of readings to average over (1 minute at 500ms intervals)

// Variables
unsigned long previousMillis = 0;   // Stores the last time update occurred
float speedReadings[numReadings];   // Array to store recent speed readings
float powerReadings[numReadings];   // Array to store recent power readings
int numSavedReadings = 0;           // Number of readings saved
float totalDistance = 0;            // Total distance traveled
float totalPower = 0;               // Total power produced

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(4));  // Seed the random number generator with an analog value
  pinMode(powerSensor, INPUT);
  pinMode(hallSensor1, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  EEPROM.get(0, ODO);
  calibrationConstant = analogRead(powerSensor);

  attachInterrupt(digitalPinToInterrupt(hallSensor1), countPulse, FALLING);
}

void loop() {
  // Measure time interval
unsigned long currentTime = millis();
unsigned long elapsedTime = currentTime - prevTime;

delay(1000);
// Calculate RPM
float rpm = (pulseCount * 60000.0) / elapsedTime;

// Calculate wheel circumference in inches
float speedKmH = (3.6 * PI * 0.3175 * rpm) / 60;   //float speedKmH = (2 * 3.6 * PI * 0.3175 * rpm) / 60;  
SPEED = round(speedKmH);

  // Measure voltage from A0
  int sensorValue = analogRead(powerSensor) - calibrationConstant;
  VOLT = (analogRead(voltageSensor) * (3.3 / 4095.0)) * 29.2;
  POWER = ((sensorValue * (3.3 / 4095.0)) / 0.032) * VOLT ; // Assuming 3.3V reference
  int batteryPercentage = convertToPercentage(VOLT);
  RANGE = (13 * batteryPercentage) / CONSUMPTION; // Km

  SpeedPowerReading(SPEED, POWER);

  if(digitalRead(26)){
    BLINKERL = "YE";
  }else {
    BLINKERL = "N";
  }

  if(digitalRead(25)){
    BLINKERR = "YE";
  }else {
    BLINKERR = "N";
  }

  // Construct the response string with variable names and values
  String responseData = "SPEED = " + String(SPEED) +
                        ", ODO = " + String(ODO, 1) +
                        ", TRIP = " + String(TRIP, 2) +
                        ", VOLT = " + String(VOLT, 2) +
                        ", POWER = " + String(POWER, 2) +
                        ", CONSUMPTION = " + String(CONSUMPTION, 2) +
                        ", RANGE = " + String(RANGE, 2) +
                        ", BLINKERL = " + BLINKERL +
                        ", BLINKERR = " + BLINKERR;

  Serial.println(responseData);

  EEPROM.put(0, ODO);
  pulseCount = 0;
  prevTime = currentTime;
}

void countPulse() {
  noInterrupts(); 
  pulseCount++;
  interrupts();   
}

// Function to handle new speed and power reading
void SpeedPowerReading(float speedKmh, float power) {
  // Save the speed reading
  speedReadings[numSavedReadings] = speedKmh;
  // Save the power reading
  powerReadings[numSavedReadings] = power;
  numSavedReadings++;

  // If we have saved enough readings, calculate the average speed and power
  if (numSavedReadings >= numReadings) {
    // Calculate average speed
    float sumSpeed = 0;
    for (int i = 0; i < numReadings; i++) {
      sumSpeed += speedReadings[i];
    }
    float averageSpeed = sumSpeed / numReadings;

    // Calculate average power
    float sumPower = 0;
    for (int i = 0; i < numReadings; i++) {
      sumPower += powerReadings[i];
    }
    float averagePower = sumPower / numReadings;

    // Update total distance using the average speed
    TRIP += averageSpeed / 60 ;
    ODO += averageSpeed / 60 ;
    // Update total power produced
    CONSUMPTION = (averagePower / 60) / (averageSpeed / 60); // Wh/Km

    // Reset saved readings count
    numSavedReadings = 0;
  }
}

int convertToPercentage(float voltage) {
  // Ensure voltage is within the valid range
  if (voltage < 61.5) {
    voltage = 61.5;
  } else if (voltage > 84) {
    voltage = 84;
  }
  
  // Map the voltage to percentage using linear interpolation
  float percentage = ((voltage - 61.5) / (84 - 61.5)) * 100.0;
  
  // Convert to integer percentage
  return (int)percentage;
}






