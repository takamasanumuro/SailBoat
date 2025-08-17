#include <Arduino.h>

#ifdef SKETCH_MODE
    #define SKETCH_VERSION "1.0.0"
    #define SKETCH_NAME "SailBoat Sketch Environment"
#endif

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    #ifdef SKETCH_MODE
        Serial.println("===================================");
        Serial.print("Starting: "); Serial.println(SKETCH_NAME);
        Serial.print("Version: "); Serial.println(SKETCH_VERSION);
        Serial.println("Environment: SKETCH");
        Serial.println("===================================");
    #endif
    
    Serial.println("Sketch environment ready for testing!");
    Serial.println("Add your experimental code here...");
}

void loop() {
    static uint32_t heartbeat_timer = 0;
    
    if (millis() - heartbeat_timer > 5000) {
        heartbeat_timer = millis();
        Serial.print("Sketch running... uptime: ");
        Serial.print(millis() / 1000);
        Serial.println(" seconds");
    }
    
    delay(100);
}