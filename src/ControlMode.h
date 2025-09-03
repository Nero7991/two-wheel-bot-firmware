/**
 * Control Mode Selector for Two-Wheel Balancing Robot
 * 
 * Allows switching between different control algorithms:
 * - PID with Kalman filtering
 * - DQN (Deep Q-Network) 
 * - Manual control
 */

#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

enum class ControlMode {
    PID_MODE,     // PID controller with Kalman filter (default)
    DQN_MODE,     // Deep Q-Network trained model
    MANUAL_MODE,  // Direct manual control
    DISABLED      // Motors disabled for safety
};

class ControlModeManager {
private:
    ControlMode current_mode;
    ControlMode previous_mode;
    bool mode_changed;
    
public:
    ControlModeManager() {
        current_mode = ControlMode::PID_MODE;  // Default to PID as requested
        previous_mode = ControlMode::PID_MODE;
        mode_changed = false;
    }
    
    /**
     * Set control mode
     */
    void setMode(ControlMode mode) {
        if (mode != current_mode) {
            previous_mode = current_mode;
            current_mode = mode;
            mode_changed = true;
        }
    }
    
    /**
     * Get current control mode
     */
    ControlMode getMode() const {
        return current_mode;
    }
    
    /**
     * Get previous control mode
     */
    ControlMode getPreviousMode() const {
        return previous_mode;
    }
    
    /**
     * Check if mode has changed since last check
     */
    bool hasChanged() {
        bool changed = mode_changed;
        mode_changed = false;
        return changed;
    }
    
    /**
     * Get mode name as string for logging
     */
    const char* getModeName() const {
        switch (current_mode) {
            case ControlMode::PID_MODE:
                return "PID";
            case ControlMode::DQN_MODE:
                return "DQN";
            case ControlMode::MANUAL_MODE:
                return "MANUAL";
            case ControlMode::DISABLED:
                return "DISABLED";
            default:
                return "UNKNOWN";
        }
    }
    
    /**
     * Toggle between PID and DQN modes
     */
    void toggleMode() {
        if (current_mode == ControlMode::PID_MODE) {
            setMode(ControlMode::DQN_MODE);
        } else if (current_mode == ControlMode::DQN_MODE) {
            setMode(ControlMode::PID_MODE);
        }
    }
    
    /**
     * Emergency stop - disable motors
     */
    void emergencyStop() {
        setMode(ControlMode::DISABLED);
    }
    
    /**
     * Resume from emergency stop
     */
    void resume() {
        if (current_mode == ControlMode::DISABLED) {
            setMode(previous_mode);
        }
    }
    
    /**
     * Check if motors should be active
     */
    bool isMotorControlActive() const {
        return current_mode != ControlMode::DISABLED;
    }
};

#endif // CONTROL_MODE_H