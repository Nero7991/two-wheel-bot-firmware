/**
 * Kalman Filter for Two-Wheel Balancing Robot
 * 
 * This filter fuses accelerometer and gyroscope data to estimate:
 * - Tilt angle (theta)
 * - Angular velocity (theta_dot)
 * - Gyroscope bias
 * 
 * The filter uses a simple linear model of the robot dynamics
 * and provides optimal state estimation in the presence of noise.
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <math.h>

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

#ifndef isfinite
#define isfinite(x) (!(isnan(x)) && !((x) == INFINITY) && !((x) == -INFINITY))
#endif

class KalmanFilter {
private:
    // State vector: [angle, angular_velocity, gyro_bias]
    float x[3];  // State estimate
    
    // Error covariance matrix (3x3, stored as 1D array for efficiency)
    float P[9];  // P[i*3+j] = P[i][j]
    
    // Process noise covariance
    float Q_angle;      // Process noise for angle
    float Q_velocity;   // Process noise for angular velocity  
    float Q_bias;       // Process noise for gyro bias
    
    // Measurement noise covariance
    float R_angle;      // Measurement noise for accelerometer angle
    
    // Sample time
    float dt;
    
    // Filter health monitoring
    bool filter_healthy;
    int consecutive_nan_count;
    static const int MAX_NAN_COUNT = 5;
    
    // Helper function for matrix operations with overflow protection
    void multiplyMatrix3x3(const float* A, const float* B, float* C) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C[i*3 + j] = 0;
                for (int k = 0; k < 3; k++) {
                    float product = A[i*3 + k] * B[k*3 + j];
                    
                    // Check for overflow or NaN
                    if (isnan(product) || fabsf(product) > 1e6f) {
                        C[i*3 + j] = 0;  // Set to safe value
                        continue;
                    }
                    
                    C[i*3 + j] += product;
                    
                    // Check result for overflow
                    if (isnan(C[i*3 + j]) || fabsf(C[i*3 + j]) > 1e6f) {
                        C[i*3 + j] = 0;  // Reset to safe value
                    }
                }
            }
        }
    }
    
public:
    /**
     * Constructor
     * @param sample_time Time between filter updates in seconds
     */
    KalmanFilter(float sample_time = 0.02f) : dt(sample_time) {
        // Initialize state
        x[0] = 0.0f;  // angle
        x[1] = 0.0f;  // angular velocity
        x[2] = 0.0f;  // gyro bias
        
        // Initialize error covariance matrix (diagonal)
        for (int i = 0; i < 9; i++) {
            P[i] = 0.0f;
        }
        P[0] = 1.0f;   // Initial angle uncertainty
        P[4] = 1.0f;   // Initial velocity uncertainty  
        P[8] = 0.01f;  // Initial bias uncertainty
        
        // Set default noise parameters (tunable)
        Q_angle = 0.001f;     // Low process noise for angle
        Q_velocity = 0.003f;  // Moderate process noise for velocity
        Q_bias = 0.00003f;    // Very low process noise for bias (slow drift)
        
        R_angle = 0.03f;      // Accelerometer measurement noise
        
        // Initialize health monitoring
        filter_healthy = true;
        consecutive_nan_count = 0;
    }
    
    /**
     * Set process noise parameters
     */
    void setProcessNoise(float angle_noise, float velocity_noise, float bias_noise) {
        Q_angle = angle_noise;
        Q_velocity = velocity_noise;
        Q_bias = bias_noise;
    }
    
    /**
     * Set measurement noise parameter
     */
    void setMeasurementNoise(float angle_noise) {
        R_angle = angle_noise;
    }
    
    /**
     * Check if filter state contains NaN values
     */
    bool hasNaN() const {
        for (int i = 0; i < 3; i++) {
            if (isnan(x[i])) return true;
        }
        for (int i = 0; i < 9; i++) {
            if (isnan(P[i])) return true;
        }
        return false;
    }
    
    /**
     * Validate and sanitize input values
     */
    bool validateInputs(float accel_angle, float gyro_rate) {
        // Check for NaN inputs
        if (isnan(accel_angle) || isnan(gyro_rate)) {
            return false;
        }
        
        // Check for extremely large values that could cause overflow
        if (fabsf(accel_angle) > 10.0f || fabsf(gyro_rate) > 100.0f) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Reset filter to safe initial state
     */
    void resetToSafeState() {
        // Reset state
        x[0] = 0.0f;
        x[1] = 0.0f;
        x[2] = 0.0f;
        
        // Reset error covariance to reasonable values
        for (int i = 0; i < 9; i++) {
            P[i] = 0.0f;
        }
        P[0] = 1.0f;   // Initial angle uncertainty
        P[4] = 1.0f;   // Initial velocity uncertainty  
        P[8] = 0.01f;  // Initial bias uncertainty
        
        // Reset health monitoring
        filter_healthy = true;
        consecutive_nan_count = 0;
    }
    
    /**
     * Update the filter with new sensor readings
     * @param accel_angle Angle calculated from accelerometer (radians)
     * @param gyro_rate Raw gyroscope angular velocity (rad/s)
     * @param delta_time Time since last update (seconds), 0 to use default dt
     */
    void update(float accel_angle, float gyro_rate, float delta_time = 0) {
        // Validate inputs first
        if (!validateInputs(accel_angle, gyro_rate)) {
            consecutive_nan_count++;
            if (consecutive_nan_count >= MAX_NAN_COUNT) {
                resetToSafeState();
            }
            return;
        }
        
        // Check for existing NaN state
        if (hasNaN()) {
            resetToSafeState();
            filter_healthy = false;
        }
        
        // Use provided delta_time or default
        float dt_update = (delta_time > 0) ? delta_time : dt;
        
        // Clamp dt to reasonable range
        if (dt_update <= 0 || dt_update > 1.0f) {
            dt_update = dt;
        }
        
        // --- PREDICTION STEP ---
        
        // State transition matrix F
        // [1  dt  -dt]
        // [0  1   -1 ]  
        // [0  0   1  ]
        
        // Predict state: x_k|k-1 = F * x_k-1|k-1
        float x_pred[3];
        x_pred[0] = x[0] + dt_update * x[1] - dt_update * x[2];  // angle += velocity * dt - bias * dt
        x_pred[1] = x[1] - x[2];                                  // velocity -= bias (bias affects gyro reading)
        x_pred[2] = x[2];                                          // bias stays constant
        
        // Use gyro rate directly for velocity prediction
        x_pred[1] = gyro_rate - x[2];  // Corrected velocity = gyro - bias
        
        // Predict error covariance: P_k|k-1 = F * P_k-1|k-1 * F' + Q
        float F[9] = {
            1, dt_update, -dt_update,
            0, 1, -1,
            0, 0, 1
        };
        
        // P = F * P * F' + Q with numerical stability checks
        float temp[9];
        float P_pred[9];
        
        // temp = F * P
        multiplyMatrix3x3(F, P, temp);
        
        // Check for NaN in intermediate result
        for (int i = 0; i < 9; i++) {
            if (isnan(temp[i])) {
                resetToSafeState();
                return;
            }
        }
        
        // P_pred = temp * F' (F' is F transposed)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P_pred[i*3 + j] = 0;
                for (int k = 0; k < 3; k++) {
                    P_pred[i*3 + j] += temp[i*3 + k] * F[j*3 + k];  // F transposed
                }
                
                // Check for NaN during multiplication
                if (isnan(P_pred[i*3 + j])) {
                    resetToSafeState();
                    return;
                }
            }
        }
        
        // Add process noise Q with bounds checking
        P_pred[0] += Q_angle;
        P_pred[4] += Q_velocity;
        P_pred[8] += Q_bias;
        
        // Ensure diagonal elements remain positive and bounded
        const float MIN_COVARIANCE = 1e-8f;
        const float MAX_COVARIANCE_PRED = 10.0f;
        
        if (P_pred[0] <= MIN_COVARIANCE) P_pred[0] = MIN_COVARIANCE;
        if (P_pred[4] <= MIN_COVARIANCE) P_pred[4] = MIN_COVARIANCE;
        if (P_pred[8] <= MIN_COVARIANCE) P_pred[8] = MIN_COVARIANCE;
        
        if (P_pred[0] > MAX_COVARIANCE_PRED) P_pred[0] = MAX_COVARIANCE_PRED;
        if (P_pred[4] > MAX_COVARIANCE_PRED) P_pred[4] = MAX_COVARIANCE_PRED;
        if (P_pred[8] > MAX_COVARIANCE_PRED) P_pred[8] = MAX_COVARIANCE_PRED;
        
        // --- UPDATE STEP ---
        
        // Measurement model: H = [1, 0, 0] (we only measure angle)
        
        // Innovation: y = z - H * x_pred
        float y = accel_angle - x_pred[0];
        
        // Innovation covariance: S = H * P_pred * H' + R
        float S = P_pred[0] + R_angle;  // Since H = [1, 0, 0]
        
        // Ensure S is positive and well-conditioned
        const float MIN_INNOVATION_COVARIANCE = 1e-6f;
        if (S <= MIN_INNOVATION_COVARIANCE || isnan(S)) {
            S = MIN_INNOVATION_COVARIANCE;
        }
        
        // Kalman gain: K = P_pred * H' * S^-1 with numerical conditioning
        float K[3];
        K[0] = P_pred[0] / S;  // First column of P_pred divided by S
        K[1] = P_pred[3] / S;
        K[2] = P_pred[6] / S;
        
        // Validate Kalman gains
        const float MAX_GAIN = 10.0f;
        for (int i = 0; i < 3; i++) {
            if (isnan(K[i]) || fabsf(K[i]) > MAX_GAIN) {
                resetToSafeState();
                return;
            }
        }
        
        // Update state: x = x_pred + K * y with bounds checking
        x[0] = x_pred[0] + K[0] * y;
        x[1] = x_pred[1] + K[1] * y;
        x[2] = x_pred[2] + K[2] * y;
        
        // Clamp state values to reasonable ranges
        const float MAX_ANGLE = M_PI;  // ±180 degrees
        const float MAX_VELOCITY = 50.0f;  // ±50 rad/s
        const float MAX_BIAS = 5.0f;  // ±5 rad/s bias
        
        if (x[0] > MAX_ANGLE) x[0] = MAX_ANGLE;
        if (x[0] < -MAX_ANGLE) x[0] = -MAX_ANGLE;
        if (x[1] > MAX_VELOCITY) x[1] = MAX_VELOCITY;
        if (x[1] < -MAX_VELOCITY) x[1] = -MAX_VELOCITY;
        if (x[2] > MAX_BIAS) x[2] = MAX_BIAS;
        if (x[2] < -MAX_BIAS) x[2] = -MAX_BIAS;
        
        // Update error covariance: P = (I - K * H) * P_pred
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (j == 0) {
                    P[i*3 + j] = P_pred[i*3 + j] * (1 - K[i]);
                } else {
                    P[i*3 + j] = P_pred[i*3 + j];
                }
            }
        }
        
        // Final NaN check and validation
        if (hasNaN()) {
            resetToSafeState();
            filter_healthy = false;
            consecutive_nan_count++;
        } else {
            // Filter is healthy
            filter_healthy = true;
            consecutive_nan_count = 0;
            
            // Clamp covariance values to prevent runaway
            const float MAX_COVARIANCE = 100.0f;
            for (int i = 0; i < 9; i++) {
                if (P[i] > MAX_COVARIANCE) P[i] = MAX_COVARIANCE;
                if (P[i] < -MAX_COVARIANCE) P[i] = -MAX_COVARIANCE;
            }
            
            // Ensure diagonal elements stay positive
            if (P[0] <= 0) P[0] = 0.01f;
            if (P[4] <= 0) P[4] = 0.01f;
            if (P[8] <= 0) P[8] = 0.001f;
        }
    }
    
    /**
     * Get the filtered angle estimate
     */
    float getAngle() const {
        return x[0];
    }
    
    /**
     * Get the filtered angular velocity estimate
     */
    float getAngularVelocity() const {
        return x[1];
    }
    
    /**
     * Get the estimated gyroscope bias
     */
    float getGyroBias() const {
        return x[2];
    }
    
    /**
     * Get all state estimates
     */
    void getState(float& angle, float& velocity, float& bias) const {
        angle = x[0];
        velocity = x[1];
        bias = x[2];
    }
    
    /**
     * Check if filter is healthy
     */
    bool isHealthy() const {
        return filter_healthy && !hasNaN();
    }
    
    /**
     * Get consecutive NaN count for diagnostics
     */
    int getNaNCount() const {
        return consecutive_nan_count;
    }
    
    /**
     * Reset the filter to initial conditions
     */
    void reset() {
        resetToSafeState();
    }
    
    /**
     * Set initial angle (useful for calibration)
     */
    void setInitialAngle(float angle) {
        x[0] = angle;
    }
};

#endif // KALMAN_FILTER_H