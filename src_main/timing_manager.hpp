#pragma once
#include <Arduino.h>
#include "config.hpp"

namespace TimingManager {
    
    // Timer class to handle individual timing operations
    class Timer {
    private:
        uint32_t interval_ms;
        uint32_t last_trigger_time;
        bool enabled;
        
    public:
        Timer(uint32_t interval = 1000);
        
        // Check if timer is ready to trigger
        bool isReady();
        
        // Reset timer to current time
        void reset();
        
        // Set new interval
        void setInterval(uint32_t new_interval_ms);
        
        // Get current interval
        uint32_t getInterval() const;
        
        // Enable/disable timer
        void setEnabled(bool enable);
        bool isEnabled() const;
        
        // Get time since last trigger
        uint32_t getElapsedTime() const;
        
        // Get time remaining until next trigger
        uint32_t getRemainingTime() const;
    };
    
    // Timer IDs for system timers
    enum SystemTimerID {
        PID_LOG_TIMER = 0,
        MAVLINK_PUBLISH_TIMER,
        THROTTLE_LOG_TIMER,
        RUDDER_READ_TIMER,
        SAIL_READ_TIMER,
        HEARTBEAT_TIMER,
        SYSTEM_TIMER_COUNT  // Must be last
    };
    
    // Initialize timing manager
    void init();
    
    // Get reference to specific system timer
    Timer& getTimer(SystemTimerID timer_id);
    
    // Convenience functions for common timers
    Timer& getPIDLogTimer();
    Timer& getMAVLinkTimer();
    Timer& getThrottleLogTimer();
    Timer& getRudderReadTimer();
    Timer& getSailReadTimer();
    Timer& getHeartbeatTimer();
    
    // Utility functions
    uint32_t getSystemUptime();           // System uptime in milliseconds
    uint32_t getSystemUptimeSeconds();    // System uptime in seconds
    
    // Safe millis() wrapper that handles overflow
    uint32_t getTimeMS();
    
    // Calculate time difference handling overflow
    uint32_t getTimeDifference(uint32_t start_time, uint32_t end_time);
    
    // High-precision timing for critical operations
    class PrecisionTimer {
    private:
        uint32_t start_time;
        
    public:
        PrecisionTimer();
        void start();
        uint32_t getElapsed() const;
        void logElapsed(const char* operation_name) const;
    };
    
    // Watchdog timer functionality
    namespace Watchdog {
        void init(uint32_t timeout_ms = 30000);  // 30 second default
        void feed();                              // Reset watchdog
        void disable();
        bool isEnabled();
        uint32_t getTimeRemaining();
    }
    
}