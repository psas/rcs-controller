unsigned long led1_time, led2_time;

 // requires passing times by reference, rather than value, in order to properly write new times
void timed_LED(int pin, unsigned long *last_time, int timeon, int timeoff) {
  static int states[14] = {LOW}; // Initialized once, persists between calls
  unsigned long current_time = millis();

  if (states[pin] == LOW) {
    if (current_time - *last_time >= timeoff) {
      digitalWrite(pin, HIGH);
      states[pin] = HIGH;
      *last_time = current_time;
    }
  } else {
    if (current_time - *last_time >= timeon) {
      digitalWrite(pin, LOW);
      states[pin] = LOW;
      *last_time = current_time;
    }
  }
}

void setup() {
   pinMode(12, OUTPUT);
   pinMode(13, OUTPUT);
   led1_time = millis();
   led2_time = millis();
}

void loop() {
  timed_LED(12, &led1_time, 100, 100);
  timed_LED(13, &led2_time, 200, 200);
}