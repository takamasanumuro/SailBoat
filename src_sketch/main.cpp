#include <Arduino.h>

#ifdef SKETCH_MODE
    #define SKETCH_NAME ""
#endif

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    #ifdef SKETCH_MODE
        Serial.println("====================================");
        Serial.print("Starting: "); Serial.println(SKETCH_NAME);
        Serial.println("Environment: SKETCH");
        Serial.println("====================================");
    #endif
    
}

void loop() {
    delay(10);
}