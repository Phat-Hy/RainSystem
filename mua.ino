#define BLYNK_TEMPLATE_ID "TMPL6WdAAAPhO"
#define BLYNK_TEMPLATE_NAME "Rain"
#define BLYNK_AUTH_TOKEN "VZJEvImOByecFsRt_H0hPCLnM4CHLSCn"

#include <Servo.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#define BLYNK_PRINT Serial

char auth[] = "VZJEvImOByecFsRt_H0hPCLnM4CHLSCn";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "FPTU_Student";
char pass[] = "12345678";

const int sensorPin = D0;           // Rain sensor pin
const int ultrasonicTrigPin = D2;   // Ultrasonic sensor trigger pin
const int ultrasonicEchoPin = D3;   // Ultrasonic sensor echo pin
int previousSensorData = HIGH;      // Previous rain sensor state
int servoPin = D4;                  // Servo control pin
Servo myServo;                      // Servo object
bool servoActive = true;            // Flag to control servo activation
bool rainSensorActive = true;       // Flag to control rain sensor activation

const int numReadings = 5;          // Number of readings to average
int readings[numReadings];          // Array to store readings
int idx = 0;                        // Index of the current reading
int total = 0;                      // Total sum of readings

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);  
  myServo.attach(servoPin);  // Attach servo to pin D4
  Serial.println("Setup complete");

  // Initialize the array with initial readings
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  Blynk.run();
  int sensorData = digitalRead(sensorPin);
  int sensorDataConverted = (sensorData == LOW) ? 0 : 1; // Convert LOW to 0 and HIGH to 1

  // Send rain sensor data to virtual pin V2
  Blynk.virtualWrite(V2, sensorDataConverted);

  if (rainSensorActive) {
    // Rain sensor is active
    if (sensorData == LOW) {
      // Rain detected
      Serial.println("Rain detected! Activating servo...");

      if (servoActive) {
        // Spin the servo while rain is detected and servo is active
        unsigned long startTime = millis();
        while (millis() - startTime < 30000) {
          myServo.write(0);   // Start spinning (assuming 0 starts the spin)
          delay(100);         // Short delay to allow continuous spin

          // Check if the sensor state has changed to no rain during spinning
          sensorData = digitalRead(sensorPin);
          if (sensorData == HIGH) {
            Serial.println("No rain detected. Stopping servo.");
            break; // Exit the spinning loop
          }

          // Check distance from ultrasonic sensor
          if (isDoorClose()) {
            Serial.println("Door close detected. Stopping servo.");
            myServo.write(90); // Stop the servo
            servoActive = false; // Deactivate servo
            break; // Exit the spinning loop
          }
        }

        // Stop the servo after 30 seconds if it hasn't already stopped
        myServo.write(90); // Stop the servo
      }
    } else {
      // No rain detected
      Serial.println("No rain detected.");

      if (!servoActive) {
        // Activate servo if it was deactivated
        Serial.println("Reactivating servo.");
        servoActive = true;
      }

      myServo.write(90); // Stop the servo
    }
  } else {
    // Rain sensor is inactive
    Serial.println("Rain sensor inactive. Waiting for object to move away...");

    // Check if object (door) is far enough to reactivate rain sensor
    if (!isDoorClose()) {
      Serial.println("Object moved away. Activating rain sensor.");
      rainSensorActive = true;
    }
  }

  // Read distance from ultrasonic sensor
  int distance = getDistance();

  // Add the new reading to the total
  total = total - readings[idx] + distance;
  // Store the new reading in the array
  readings[idx] = distance;
  // Increment idx for the next reading
  idx = (idx + 1) % numReadings;

  // Calculate the average
  float averageDistance = total / (float)numReadings;

  // Print the smoothed distance
  Serial.print("Smoothed Distance (cm): ");
  Serial.println(averageDistance);

  // Check conditions to turn on the system
  if (!isDoorClose() || averageDistance > 5) {
    // Perform actions to turn on the system
    Serial.println("Door reopened or average distance exceeded 5 cm. Turning on the system.");
    // Add your code to turn on the system here
    // For example, you can activate a relay or perform any other action.
    // Replace the following line with your actual code to turn on the system.
    // Example: digitalWrite(systemPin, HIGH);
  }

  // Send door status to virtual pin V3
  int doorStatus = isDoorClose() ? 1 : 0; // Convert boolean to 1 or 0
  Blynk.virtualWrite(V3, doorStatus);

  delay(100); // Delay between sensor readings
}

bool isDoorClose() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ultrasonicEchoPin, HIGH);

  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;

  // Print for debugging
  Serial.print("Pulse duration: ");
  Serial.print(duration);
  Serial.print(" microseconds. Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 5) {  // Adjust threshold as needed
    return true;  // Object (door) is close
  } else {
    return false; // Object (door) is far away
  }
}

int getDistance() {
  // Your code to read distance from the ultrasonic sensor
  // Ensure to implement the pulse and duration measurement
  // Return the measured distance
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ultrasonicEchoPin, HIGH);

  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;

  return distance;
}

// Blynk Write function to handle virtual button press
BLYNK_WRITE(V1) {
  Serial.println("Button pressed");
  if (param.asInt() == 1) { // Button pressed
    if (isDoorClose()) {
      Blynk.logEvent("notification", "The door is closed");
    } else {
      // Close the door until it is closed
      Serial.println("Door is not closed. Closing the door...");
      while (!isDoorClose()) {
        myServo.write(0);  // Spin to close the door
        delay(100);  // Delay to allow servo to move
      }
      myServo.write(90);  // Stop the servo
      Blynk.logEvent("notification", "The door is now closed.");
      // The deep sleep is removed to keep the system running
    }
  }
}

// Blynk Sync function to update V1 based on V3
BLYNK_WRITE(V3) {
  int doorStatus = param.asInt();
  if (doorStatus == 1) { // Door is closed
    Blynk.virtualWrite(V1, 0); // Turn off button
  } else { // Door is open
    Blynk.virtualWrite(V1, 1); // Turn on button
  }
}
