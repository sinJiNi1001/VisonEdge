// Define pin connections for ESP32
#define TRIG_PIN 5    // GPIO5 for Trigger
#define ECHO_PIN 18   // GPIO18 for Echo
#define LED_PIN 2     // GPIO2 (built-in LED)
#define BUZZER_PIN 4  // GPIO4 for Buzzer

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  
  // Test buzzer at startup
  Serial.println("Testing buzzer...");
  tone(BUZZER_PIN, 1000);  // 1kHz tone
  delay(500);              // Buzz for 0.5 seconds
  noTone(BUZZER_PIN);
  delay(500);
  
  Serial.println("System initialized");
}

void loop() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time with timeout and debug info
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Handle alert based on distance
  if (distance > 0 && distance < 30) {
    // Object detected within 30cm
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000);  // Higher frequency for better alerting
    Serial.println("ALERT: Object detected! BUZZER ON");
  } else {
    // No object detected or out of range
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }

  delay(100);  // Shorter delay for more responsive alerts
}