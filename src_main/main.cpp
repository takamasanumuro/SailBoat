#include <Arduino.h>
#include "config.hpp"
#include "logger.hpp"
#include "timing_manager.hpp"

void setup() {
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    
    // Initialize logging system first
    Logger::init();
	Logger::setLogLevel(Logger::DEBUG);
    
    #ifdef ENVIRONMENT_MAIN
		//Log system start info
    #endif
    
	delay(1000);

    // Initialize timing manager
    TimingManager::init();

	Logger::log(Logger::INFO, "System initialization complete");
}

void loop() {

}



 