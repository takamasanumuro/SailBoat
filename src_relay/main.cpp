#include <Arduino.h>
 

constexpr int relay_one_pin = 53;
constexpr int relay_two_pin = 52;
constexpr int relay_three_pin = 51;
constexpr int relay_four_pin = 50;

constexpr int interval_ms = 3000;


void setup() {
    pinMode(relay_one_pin, OUTPUT); digitalWrite(relay_one_pin, HIGH);
    pinMode(relay_two_pin, OUTPUT); digitalWrite(relay_two_pin, HIGH);
    pinMode(relay_three_pin, OUTPUT); digitalWrite(relay_three_pin, HIGH);
    pinMode(relay_four_pin, OUTPUT); digitalWrite(relay_four_pin, HIGH);
}

void loop() {
    //Toggle each pin sequentially
    digitalWrite(relay_one_pin, LOW);
    delay(interval_ms);
    digitalWrite(relay_one_pin, HIGH);

    digitalWrite(relay_two_pin, LOW);
    delay(interval_ms);
    digitalWrite(relay_two_pin, HIGH);

    digitalWrite(relay_three_pin, LOW);
    delay(interval_ms);
    digitalWrite(relay_three_pin, HIGH);

    digitalWrite(relay_four_pin, LOW);
    delay(interval_ms);
    digitalWrite(relay_four_pin, HIGH);

}