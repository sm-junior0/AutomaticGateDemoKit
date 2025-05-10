#include <Servo.h>

// Pin definitions
const int triggerPin = 2;       // Ultrasonic sensor trigger pin
const int echoPin = 3;          // Ultrasonic sensor echo pin
const int redLedAnodePin = 4;   // Red LED anode pin (adjust if connected to Vcc)
const int redLedCathodePin = 8; // Red LED cathode pin (acting as ground)
const int blueLedCathodePin = 7;
const int blueLedPin = 5; // Blue LED pin
const int servoPin = 6;   // Servo motor pin
const int buzzerPin = 12; // Buzzer pin

// Constants
const float thresholdDistance = 10.0;  // Detection distance in cm
const int closedAngle = 0;             // Servo angle when barrier is closed
const int openAngle = 90;              // Servo angle when barrier is open
const unsigned long closeDelay = 5000; // 5 seconds delay before closing

Servo myServo;                       // Servo object
bool isOpen = false;                 // Barrier state: false = closed, true = open
unsigned long lastDetectionTime = 0; // Time when object was last detected

void setup()
{
  // Initialize pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLedAnodePin, OUTPUT); // Set anode pin as output
  pinMode(redLedCathodePin, OUTPUT);
  pinMode(blueLedCathodePin, OUTPUT);
  digitalWrite(blueLedCathodePin, LOW); // Set cathode pin as output
  digitalWrite(redLedCathodePin, LOW);  // Set pin 8 LOW to act as ground
  pinMode(blueLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Attach servo and set initial position
  myServo.attach(servoPin);
  myServo.write(closedAngle);         // Barrier closed
  digitalWrite(redLedAnodePin, HIGH); // Red LED on (anode high, cathode low)
  digitalWrite(blueLedPin, LOW);      // Blue LED off
  digitalWrite(buzzerPin, LOW);       // Buzzer off
}

// Function to measure distance using ultrasonic sensor
float getDistance()
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 58.0; // Convert to cm
}

unsigned long lastBeepTime = 0;
bool buzzerState = false;

void loop()
{
  float distance = getDistance();
  bool objectDetected = (distance < thresholdDistance);

  if (objectDetected)
  {
    if (!isOpen)
    {
      // Open barrier and update indicators
      myServo.write(openAngle);
      isOpen = true;
      digitalWrite(blueLedPin, HIGH);
      digitalWrite(redLedAnodePin, LOW); // Turn off red LED
    }
    lastDetectionTime = millis(); // Update detection time
  }
  else
  {
    if (isOpen && (millis() - lastDetectionTime > closeDelay))
    {
      // Close barrier after 5 seconds of no detection
      myServo.write(closedAngle);
      isOpen = false;
      digitalWrite(redLedAnodePin, HIGH); // Turn red LED ON
      digitalWrite(blueLedPin, LOW);
      digitalWrite(buzzerPin, LOW); // Turn off buzzer
      buzzerState = false;          // Reset buzzer state
    }
  }

  // Beep logic when gate is open
  if (isOpen)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastBeepTime >= 500)
    {                                       // Change 500 to speed up/down the beep rate
      buzzerState = !buzzerState;           // Toggle buzzer state
      digitalWrite(buzzerPin, buzzerState); // Turn buzzer ON or OFF
      lastBeepTime = currentTime;
    }
  }

  delay(50); // Small delay for stability
}
