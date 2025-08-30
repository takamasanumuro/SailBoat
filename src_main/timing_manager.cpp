#include "timing_manager.hpp"
#include "logger.hpp"

namespace TimingManager {
    
    // System timer instances
    static Timer system_timers[SYSTEM_TIMER_COUNT];
    static bool timing_manager_initialized = false;
    static uint32_t system_start_time = 0;
    
    // Timer class implementation
    Timer::Timer(uint32_t interval) 
        : interval_ms(interval), last_trigger_time(0), enabled(true) {
        reset();
    }
    
    bool Timer::isReady() {
        if (!enabled) return false;
        
        uint32_t current_time = millis();
        uint32_t elapsed = getTimeDifference(last_trigger_time, current_time);
        
        if (elapsed >= interval_ms) {
            last_trigger_time = current_time;
            return true;
        }
        return false;
    }
    
    void Timer::reset() {
        last_trigger_time = millis();
    }
    
    void Timer::setInterval(uint32_t new_interval_ms) {
        interval_ms = new_interval_ms;
    }
    
    uint32_t Timer::getInterval() const {
        return interval_ms;
    }
    
    void Timer::setEnabled(bool enable) {
        enabled = enable;
        if (enable) {
            reset(); // Reset when enabling to avoid immediate trigger
        }
    }
    
    bool Timer::isEnabled() const {
        return enabled;
    }
    
    uint32_t Timer::getElapsedTime() const {
        return getTimeDifference(last_trigger_time, millis());
    }
    
    uint32_t Timer::getRemainingTime() const {
        uint32_t elapsed = getElapsedTime();
        return (elapsed >= interval_ms) ? 0 : (interval_ms - elapsed);
    }
    
    // Timing Manager implementation
    void init() {
        system_start_time = millis();
        
        // Initialize system timers with their respective intervals from config
        system_timers[MOTOR_LOG_TIMER] = Timer(Config::Timing::THROTTLE_LOG_INTERVAL_MS);
        system_timers[RUDDER_READ_TIMER] = Timer(Config::Timing::RUDDER_READ_INTERVAL_MS);
        
        timing_manager_initialized = true;
        Logger::log(Logger::INFO, "Timing manager initialized");
    }
    
    Timer& getTimer(SystemTimerID timer_id) {
        if (timer_id >= SYSTEM_TIMER_COUNT) {
            Logger::logError("Invalid timer ID requested");
            return system_timers[0]; // Return default timer
        }
        return system_timers[timer_id];
    }
    
    Timer& getThrottleLogTimer() {
        return getTimer(MOTOR_LOG_TIMER);
    }
    
    Timer& getRudderReadTimer() {
        return getTimer(RUDDER_READ_TIMER);
    }
    
    uint32_t getSystemUptime() {
        if (!timing_manager_initialized) return 0;
        return getTimeDifference(system_start_time, millis());
    }
    
    uint32_t getSystemUptimeSeconds() {
        return getSystemUptime() / 1000;
    }
    
    uint32_t getTimeMS() {
        return millis();
    }
    
    uint32_t getTimeDifference(uint32_t start_time, uint32_t end_time) {
        // Handle millis() overflow (occurs every ~49.7 days)
        if (end_time >= start_time) {
            return end_time - start_time;
        } else {
            // Overflow occurred
            return (0xFFFFFFFF - start_time) + end_time + 1;
        }
    }
    
    // Precision Timer implementation
    PrecisionTimer::PrecisionTimer() : start_time(0) {
        start();
    }
    
    void PrecisionTimer::start() {
        start_time = millis();
    }
    
    uint32_t PrecisionTimer::getElapsed() const {
        return getTimeDifference(start_time, millis());
    }
    
    void PrecisionTimer::logElapsed(const char* operation_name) const {
        Logger::logTiming(operation_name, getElapsed());
    }
    
    // Watchdog implementation
    namespace Watchdog {
        static bool watchdog_enabled = false;
        static uint32_t watchdog_timeout_ms = 30000;
        static uint32_t last_feed_time = 0;
        
        void init(uint32_t timeout_ms) {
            watchdog_timeout_ms = timeout_ms;
            feed(); // Initialize feed time
            watchdog_enabled = true;
            Logger::log(Logger::INFO, "Watchdog timer initialized with timeout: ", (int)timeout_ms);
        }
        
        void feed() {
            last_feed_time = millis();
            if (watchdog_enabled) {
                Logger::log(Logger::DEBUG, "Watchdog fed");
            }
        }
        
        void disable() {
            watchdog_enabled = false;
            Logger::log(Logger::INFO, "Watchdog timer disabled");
        }
        
        bool isEnabled() {
            return watchdog_enabled;
        }
        
        uint32_t getTimeRemaining() {
            if (!watchdog_enabled) return 0;
            
            uint32_t elapsed = getTimeDifference(last_feed_time, millis());
            return (elapsed >= watchdog_timeout_ms) ? 0 : (watchdog_timeout_ms - elapsed);
        }
        
        // This function should be called periodically to check watchdog status
        bool checkWatchdog() {
            if (!watchdog_enabled) return true;
            
            uint32_t time_remaining = getTimeRemaining();
            if (time_remaining == 0) {
                Logger::logCritical("Watchdog timeout detected!");
                return false;
            }
            
            if (time_remaining < (watchdog_timeout_ms / 4)) {
                Logger::logError("Watchdog timeout approaching");
            }
            
            return true;
        }
    }
    
}