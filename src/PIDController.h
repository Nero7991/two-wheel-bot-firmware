/**
 * PID Controller for Two-Wheel Balancing Robot
 * 
 * Features:
 * - Standard PID control with separate gains
 * - Anti-windup for integral term
 * - Output limiting
 * - Derivative filtering option
 * - Reset capability
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>

class PIDController {
private:
    // PID gains
    float Kp;  // Proportional gain
    float Ki;  // Integral gain  
    float Kd;  // Derivative gain
    
    // State variables
    float integral;
    float prev_error;
    float prev_derivative;
    
    // Anti-windup and limits
    float integral_limit;
    float output_min;
    float output_max;
    
    // Derivative filter coefficient (0 = no filter, 0.9 = heavy filter)
    float derivative_filter_coeff;
    
    // Sample time
    float dt;
    
    // Helper function to constrain value
    float constrain(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
public:
    /**
     * Constructor
     * @param sample_time Time between controller updates in seconds
     */
    PIDController(float sample_time = 0.02f) : dt(sample_time) {
        // Default gains (will need tuning)
        Kp = 50.0f;   // Proportional to angle error
        Ki = 0.0f;    // Start with no integral
        Kd = 2.0f;    // Derivative for damping
        
        // Initialize state
        integral = 0.0f;
        prev_error = 0.0f;
        prev_derivative = 0.0f;
        
        // Set default limits
        integral_limit = 100.0f;  // Prevent integral windup
        output_min = -1.0f;       // Motor torque limits
        output_max = 1.0f;
        
        // Derivative filter (0.2 = light filtering)
        derivative_filter_coeff = 0.2f;
    }
    
    /**
     * Set PID gains
     */
    void setGains(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
    
    /**
     * Get current gains (for tuning/debugging)
     */
    void getGains(float& p, float& i, float& d) const {
        p = Kp;
        i = Ki;
        d = Kd;
    }
    
    /**
     * Set output limits
     */
    void setOutputLimits(float min, float max) {
        output_min = min;
        output_max = max;
    }
    
    /**
     * Set integral windup limit
     */
    void setIntegralLimit(float limit) {
        integral_limit = limit;
    }
    
    /**
     * Set derivative filter coefficient (0-1, higher = more filtering)
     */
    void setDerivativeFilter(float coeff) {
        derivative_filter_coeff = constrain(coeff, 0.0f, 0.99f);
    }
    
    /**
     * Calculate PID output for balancing control
     * @param angle Current tilt angle (radians)
     * @param angular_velocity Current angular velocity (rad/s)
     * @param angle_setpoint Desired angle (usually 0 for upright)
     * @param delta_time Time since last update (seconds), 0 to use default dt
     * @return Motor torque command (-1 to 1)
     */
    float calculate(float angle, float angular_velocity, float angle_setpoint = 0.0f, float delta_time = 0) {
        // Use provided delta_time or default
        float dt_update = (delta_time > 0) ? delta_time : dt;
        
        // Calculate error
        float error = angle_setpoint - angle;
        
        // Proportional term
        float P = Kp * error;
        
        // Integral term with anti-windup
        integral += error * dt_update;
        integral = constrain(integral, -integral_limit, integral_limit);
        float I = Ki * integral;
        
        // Derivative term (using angular velocity directly for better response)
        // This is more responsive than calculating d(error)/dt
        float D = -Kd * angular_velocity;  // Negative because we want to oppose motion
        
        // Alternative: Use error derivative with filtering
        // float derivative = (error - prev_error) / dt_update;
        // derivative = derivative_filter_coeff * prev_derivative + 
        //              (1 - derivative_filter_coeff) * derivative;
        // float D = Kd * derivative;
        // prev_derivative = derivative;
        
        // Calculate total output
        float output = P + I + D;
        
        // Apply output limits
        output = constrain(output, output_min, output_max);
        
        // Store error for next iteration
        prev_error = error;
        
        return output;
    }
    
    /**
     * Alternative calculate method for cascaded control
     * Uses separate angle and velocity control
     */
    float calculateCascaded(float angle, float angular_velocity, 
                           float angle_setpoint = 0.0f,
                           float velocity_setpoint = 0.0f,
                           float delta_time = 0) {
        // Use provided delta_time or default
        float dt_update = (delta_time > 0) ? delta_time : dt;
        
        // Outer loop: angle control
        float angle_error = angle_setpoint - angle;
        
        // The angle controller output becomes the velocity setpoint
        float desired_velocity = Kp * angle_error + velocity_setpoint;
        
        // Inner loop: velocity control
        float velocity_error = desired_velocity - angular_velocity;
        
        // Integral term for steady-state error
        integral += angle_error * dt_update;
        integral = constrain(integral, -integral_limit, integral_limit);
        
        // Combined output
        float output = desired_velocity + Ki * integral + Kd * velocity_error;
        
        // Apply output limits
        output = constrain(output, output_min, output_max);
        
        prev_error = angle_error;
        
        return output;
    }
    
    /**
     * Reset the controller state
     */
    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
        prev_derivative = 0.0f;
    }
    
    /**
     * Get integral term value (for debugging)
     */
    float getIntegral() const {
        return integral;
    }
    
    /**
     * Set integral term value (for bumpless transfer)
     */
    void setIntegral(float value) {
        integral = constrain(value, -integral_limit, integral_limit);
    }
    
    /**
     * Get individual PID components for analysis
     */
    void getComponents(float angle, float angular_velocity, float angle_setpoint,
                      float& p_term, float& i_term, float& d_term) {
        float error = angle_setpoint - angle;
        p_term = Kp * error;
        i_term = Ki * integral;
        d_term = -Kd * angular_velocity;
    }
};

#endif // PID_CONTROLLER_H