// Pin definitions
#define TRIG_PIN1 5  // GPIO5 (D1 on NodeMCU) - Inside sensor
#define ECHO_PIN1 4  // GPIO4 (D2 on NodeMCU)
#define TRIG_PIN2 0  // GPIO0 (D3 on NodeMCU) - Outside sensor
#define ECHO_PIN2 2  // GPIO2 (D4 on NodeMCU)
#define LED_PIN 16   // GPIO16 (D0 on NodeMCU) - LED pin for alert
#define FLAME_SENSOR_PIN 14  // GPIO14 (D5 on NodeMCU) - Flame sensor pin
#define BUZZER_PIN 12  // GPIO5 (D6 on NodeMCU) - Buzzer pin

// Constants
const int MAX_DISTANCE = 400;     // Maximum measurable distance in cm
const int MOTION_THRESHOLD = 7;   // Threshold for detecting motion (in cm)
const int TIME_WINDOW = 3000;     // Time window for sequence matching (in ms)
const int DEBOUNCE_TIME = 500;    // Debounce time to ignore noise (in ms)
const int MAX_PEOPLE = 39;        // Threshold for LED activation

// Variables
int peopleCount = 0;              // Occupancy count
unsigned long lastMotionTime1 = 0; // Last motion time for Sensor 1 (Inside)
unsigned long lastMotionTime2 = 0; // Last motion time for Sensor 2 (Outside)
bool motionInside = false;        // Motion detected inside
bool motionOutside = false;       // Motion detected outside

// Sequence timing variables
unsigned long lastInsideTime = 0;  // Last valid motion time for inside
unsigned long lastOutsideTime = 0; // Last valid motion time for outside

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
  digitalWrite(BUZZER_PIN, LOW); // Ensure Buzzer is off initially

  Serial.println("Occupancy Detection System Ready");
  Serial.print("People in Room: ");
  Serial.println(peopleCount);
}

void loop() {
  unsigned long currentTime = millis();

  float distance1 = measureDistance(TRIG_PIN1, ECHO_PIN1);
  float distance2 = measureDistance(TRIG_PIN2, ECHO_PIN2);

  // Motion detection logic
  if (distance1 > 0 && distance1 <= MOTION_THRESHOLD) {
    if (!motionInside && (currentTime - lastMotionTime1 > DEBOUNCE_TIME)) {
      motionInside = true;
      lastMotionTime1 = currentTime;
      Serial.println("Motion detected by Sensor 1 (Inside)");
      processMotionSequence(true); // Process motion as "inside"
    }
  } else {
    motionInside = false;
  }

  if (distance2 > 0 && distance2 <= MOTION_THRESHOLD) {
    if (!motionOutside && (currentTime - lastMotionTime2 > DEBOUNCE_TIME)) {
      motionOutside = true;
      lastMotionTime2 = currentTime;
      Serial.println("Motion detected by Sensor 2 (Outside)");
      processMotionSequence(false); // Process motion as "outside"
    }
  } else {
    motionOutside = false;
  }

  // LED control based on people count
  if (peopleCount > MAX_PEOPLE) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off LED
  }

  // Flame sensor logic: Check if flame is detected
  int flameStatus = digitalRead(FLAME_SENSOR_PIN);
  if (flameStatus == HIGH) {  // Flame detected
    digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
    Serial.println("Flame detected! Buzzer activated.");
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
  }

  delay(100); // Short delay to prevent excessive polling
}

// Function to measure distance using HC-SR04
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  float distance = (duration / 2.0) * 0.0343;   // Convert to cm

  if (duration == 0 || distance > MAX_DISTANCE) {
    return -1; // Return -1 if out of range
  }
  return distance;
}

// Function to process motion sequence
void processMotionSequence(bool isInside) {
  unsigned long currentTime = millis();

  if (isInside) {
    lastInsideTime = currentTime;

    // Check if "Outside → Inside" sequence occurred
    if (lastOutsideTime > 0 && (currentTime - lastOutsideTime <= TIME_WINDOW)) {
      peopleCount++;
      Serial.println("Person entered the room");
      Serial.print("People in Room: ");
      Serial.println(peopleCount);
      resetSequence();
    }
  } else {
    lastOutsideTime = currentTime;

    // Check if "Inside → Outside" sequence occurred
    if (lastInsideTime > 0 && (currentTime - lastInsideTime <= TIME_WINDOW)) {
      peopleCount = max(0, peopleCount - 1);
      Serial.println("Person exited the room");
      Serial.print("People in Room: ");
      Serial.println(peopleCount);
      resetSequence();
    }
  }
}

// Function to reset the sequence tracking
void resetSequence() {
  lastInsideTime = 0;
  lastOutsideTime = 0;
}
