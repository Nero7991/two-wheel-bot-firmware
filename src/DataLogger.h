/**
 * Serial Data Logger for Two-Wheel Balancing Robot
 * 
 * Outputs data in Simple Plot Language (SPL) format for real-time plotting
 * Supports plot configuration and tagged data streaming
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <zephyr/kernel.h>
#include <stdio.h>

class DataLogger {
private:
    // Logging configuration
    bool enabled;
    uint32_t log_interval_ms;
    uint32_t last_log_time;
    uint32_t sequence_number;
    
    // Data decimation for reducing output rate
    int decimation_factor;
    int decimation_counter;
    
    // Plot configuration state
    bool plots_configured;
    
public:
    DataLogger() {
        enabled = true;
        log_interval_ms = 20;  // 50Hz by default
        last_log_time = 0;
        sequence_number = 0;
        decimation_factor = 1;
        decimation_counter = 0;
        plots_configured = false;
    }
    
    /**
     * Enable/disable logging
     */
    void setEnabled(bool enable) {
        enabled = enable;
        if (enable && !plots_configured) {
            configurePredefinedPlots();
        }
    }
    
    /**
     * Set logging interval in milliseconds
     */
    void setInterval(uint32_t interval_ms) {
        log_interval_ms = interval_ms;
    }
    
    /**
     * Set decimation factor (log every Nth sample)
     */
    void setDecimation(int factor) {
        decimation_factor = (factor > 0) ? factor : 1;
    }
    
    // ===== SPL Plot Configuration API =====
    
    /**
     * Begin defining a new plot configuration
     */
    void beginPlotConfig(const char* plot_name) {
        if (!enabled) return;
        printf("<plot:%s>\n", plot_name);
    }
    
    /**
     * Set plot title
     */
    void setPlotTitle(const char* title) {
        if (!enabled) return;
        printf("title=%s\n", title);
    }
    
    /**
     * Set plot axis labels
     */
    void setPlotAxis(const char* xlabel, const char* ylabel) {
        if (!enabled) return;
        if (xlabel) printf("xlabel=%s\n", xlabel);
        if (ylabel) printf("ylabel=%s\n", ylabel);
    }
    
    /**
     * Set axis label (ylabel only - xlabel defaults to "Time")
     */
    void setPlotAxis(const char* ylabel) {
        if (!enabled) return;
        printf("ylabel=%s\n", ylabel);
    }
    
    /**
     * Set Y-axis range
     */
    void setPlotYRange(float ymin, float ymax) {
        if (!enabled) return;
        printf("ymin=%.3f\n", (double)ymin);
        printf("ymax=%.3f\n", (double)ymax);
    }
    
    /**
     * Set plot height
     */
    void setPlotHeight(int height) {
        if (!enabled) return;
        printf("height=%d\n", height);
    }
    
    /**
     * Define column names for the plot
     */
    void setPlotColumns(const char* column_names) {
        if (!enabled) return;
        printf("columns=%s\n", column_names);
    }
    
    /**
     * Add a data series to the plot
     */
    void addPlotSeries(int column_index, const char* label, const char* color, int width = 2) {
        if (!enabled) return;
        printf("series=%d,%s,%s,%d\n", column_index, label, color, width);
    }
    
    /**
     * End plot configuration
     */
    void endPlotConfig() {
        if (!enabled) return;
        printf("</plot:\n");  // Close tag without name for simplicity
    }
    
    // ===== SPL Data Streaming API =====
    
    /**
     * Begin data stream for a specific plot
     */
    void beginDataStream(const char* plot_name) {
        if (!enabled) return;
        printf("<data:%s>\n", plot_name);
    }
    
    /**
     * Stream a data point (timestamp + values)
     */
    void streamData(uint32_t timestamp, float value1) {
        if (!enabled) return;
        printf("%u,%.4f\n", timestamp, (double)value1);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f\n", timestamp, (double)value1, (double)value2);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2, float value3) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f,%.4f\n", timestamp, (double)value1, (double)value2, (double)value3);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2, float value3, float value4) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f,%.4f,%.4f\n", timestamp, 
               (double)value1, (double)value2, (double)value3, (double)value4);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2, float value3, 
                   float value4, float value5) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f\n", timestamp,
               (double)value1, (double)value2, (double)value3, (double)value4, (double)value5);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2, float value3,
                   float value4, float value5, float value6) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", timestamp,
               (double)value1, (double)value2, (double)value3, (double)value4, 
               (double)value5, (double)value6);
    }
    
    void streamData(uint32_t timestamp, float value1, float value2, float value3,
                   float value4, float value5, float value6, float value7, float value8) {
        if (!enabled) return;
        printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", timestamp,
               (double)value1, (double)value2, (double)value3, (double)value4, 
               (double)value5, (double)value6, (double)value7, (double)value8);
    }
    
    /**
     * Stream motor data (timestamp + motor values)
     */
    void streamMotorData(uint32_t timestamp, int16_t left_motor, int16_t right_motor) {
        if (!enabled) return;
        printf("%u,%d,%d\n", timestamp, left_motor, right_motor);
    }
    
    /**
     * End data stream
     */
    void endDataStream() {
        if (!enabled) return;
        printf("</data:\n");  // Close tag without name for simplicity
    }
    
    // ===== Legacy API Compatibility =====
    
    /**
     * Log data point (legacy interface - converts to SPL format)
     */
    void log(uint32_t timestamp_ms,
            const char* control_mode,
            float raw_angle,
            float raw_gyro,
            float kalman_angle,
            float kalman_velocity,
            float kalman_bias,
            float pid_output,
            float pid_p,
            float pid_i,
            float pid_d,
            int16_t motor_left,
            int16_t motor_right,
            float angle_setpoint = 0.0f,
            bool has_fallen = false) {
        
        if (!enabled) return;
        
        // Check decimation
        decimation_counter++;
        if (decimation_counter < decimation_factor) {
            return;
        }
        decimation_counter = 0;
        
        // Check minimum interval
        if (timestamp_ms - last_log_time < log_interval_ms) {
            return;
        }
        last_log_time = timestamp_ms;
        
        // Stream data to different plots
        streamToAnglesPlot(timestamp_ms, raw_angle, kalman_angle);
        streamToVelocityPlot(timestamp_ms, raw_gyro, kalman_velocity);
        streamToPIDPlot(timestamp_ms, pid_output, pid_p, pid_i, pid_d);
        streamToMotorPlot(timestamp_ms, motor_left, motor_right);
        
        // Stream status data
        streamToStatusPlot(timestamp_ms, control_mode, has_fallen, kalman_bias);
        
        // Stream comprehensive plot data (average of both motors for single output signal)
        int16_t motor_avg = (motor_left + motor_right) / 2;
        streamToComprehensivePlot(timestamp_ms, raw_angle, kalman_angle, 
                                 kalman_velocity, pid_output, pid_p, pid_i, pid_d, motor_avg);
        
        sequence_number++;
    }
    
private:
    /**
     * Configure predefined plots for robot data
     */
    void configurePredefinedPlots() {
        if (plots_configured) return;
        
        // Angles plot
        beginPlotConfig("angles");
        setPlotTitle("Robot Angle Tracking");
        setPlotAxis("Angle (radians)");
        setPlotYRange(-1.0f, 1.0f);
        setPlotHeight(200);
        setPlotColumns("timestamp_ms,raw_angle,kalman_angle");
        addPlotSeries(1, "Raw Angle", "#ff6b6b", 1);
        addPlotSeries(2, "Kalman Filtered", "#4ecdc4", 2);
        endPlotConfig();
        
        // Velocity plot
        beginPlotConfig("velocity");
        setPlotTitle("Angular Velocity");
        setPlotAxis("Velocity (rad/s)");
        setPlotYRange(-10.0f, 10.0f);
        setPlotHeight(200);
        setPlotColumns("timestamp_ms,raw_gyro,kalman_velocity");
        addPlotSeries(1, "Raw Gyro", "#ff9f43", 1);
        addPlotSeries(2, "Kalman Velocity", "#10ac84", 2);
        endPlotConfig();
        
        // PID plot
        beginPlotConfig("pid");
        setPlotTitle("PID Control Output");
        setPlotAxis("Control Signal");
        setPlotYRange(-1.2f, 1.2f);
        setPlotHeight(200);
        setPlotColumns("timestamp_ms,pid_output,pid_p,pid_i,pid_d");
        addPlotSeries(1, "PID Output", "#f7dc6f", 3);
        addPlotSeries(2, "P Term", "#ff9ff3", 1);
        addPlotSeries(3, "I Term", "#54a0ff", 1);
        addPlotSeries(4, "D Term", "#5f27cd", 1);
        endPlotConfig();
        
        // Motor plot
        beginPlotConfig("motors");
        setPlotTitle("Motor Control Signals");
        setPlotAxis("Motor Speed (PWM)");
        setPlotYRange(-800.0f, 800.0f);
        setPlotHeight(200);
        setPlotColumns("timestamp_ms,left_motor,right_motor");
        addPlotSeries(1, "Left Motor", "#45b7d1", 2);
        addPlotSeries(2, "Right Motor", "#f7dc6f", 2);
        endPlotConfig();
        
        // Status plot (for bias and other status info)
        beginPlotConfig("status");
        setPlotTitle("System Status");
        setPlotAxis("Bias (rad/s)");
        setPlotYRange(-1.0f, 1.0f);
        setPlotHeight(200);
        setPlotColumns("timestamp_ms,kalman_bias");
        addPlotSeries(1, "Gyro Bias", "#a55eea", 2);
        endPlotConfig();
        
        // Comprehensive plot with all key signals including PID components
        beginPlotConfig("comprehensive");
        setPlotTitle("Complete System Overview with PID Components");
        setPlotAxis("Multiple Signals (scaled)");
        setPlotYRange(-2.0f, 2.0f);
        setPlotHeight(300);
        setPlotColumns("timestamp_ms,raw_angle,kalman_angle,kalman_velocity,pid_output,pid_p,pid_i,pid_d,motor_normalized");
        addPlotSeries(1, "Raw Angle", "#ff6b6b", 1);
        addPlotSeries(2, "Kalman Angle", "#4ecdc4", 2);
        addPlotSeries(3, "Angular Velocity (×0.1)", "#10ac84", 2);
        addPlotSeries(4, "PID Output", "#f7dc6f", 3);
        addPlotSeries(5, "P Term", "#ff9ff3", 1);
        addPlotSeries(6, "I Term", "#54a0ff", 1);
        addPlotSeries(7, "D Term", "#5f27cd", 1);
        addPlotSeries(8, "Motor Output (×0.002)", "#45b7d1", 2);
        endPlotConfig();
        
        plots_configured = true;
    }
    
    void streamToAnglesPlot(uint32_t timestamp, float raw_angle, float kalman_angle) {
        beginDataStream("angles");
        streamData(timestamp, raw_angle, kalman_angle);
        endDataStream();
    }
    
    void streamToVelocityPlot(uint32_t timestamp, float raw_gyro, float kalman_velocity) {
        beginDataStream("velocity");
        streamData(timestamp, raw_gyro, kalman_velocity);
        endDataStream();
    }
    
    void streamToPIDPlot(uint32_t timestamp, float pid_output, float pid_p, 
                        float pid_i, float pid_d) {
        beginDataStream("pid");
        streamData(timestamp, pid_output, pid_p, pid_i, pid_d);
        endDataStream();
    }
    
    void streamToMotorPlot(uint32_t timestamp, int16_t motor_left, int16_t motor_right) {
        beginDataStream("motors");
        streamMotorData(timestamp, motor_left, motor_right);
        endDataStream();
    }
    
    void streamToStatusPlot(uint32_t timestamp, const char* mode, bool fallen, float bias) {
        beginDataStream("status");
        streamData(timestamp, bias);
        endDataStream();
        
        // Log events as comments for debugging
        static const char* last_mode = "";
        static bool last_fallen = false;
        
        if (strcmp(mode, last_mode) != 0) {
            printf("# MODE_CHANGE: %s -> %s at %u ms\n", last_mode, mode, timestamp);
            last_mode = mode;
        }
        
        if (fallen != last_fallen) {
            printf("# FALLEN: %s at %u ms\n", fallen ? "true" : "false", timestamp);
            last_fallen = fallen;
        }
    }
    
    void streamToComprehensivePlot(uint32_t timestamp, float raw_angle, float kalman_angle,
                                   float kalman_velocity, float pid_output, float pid_p,
                                   float pid_i, float pid_d, int16_t motor_avg) {
        beginDataStream("comprehensive");
        // Scale angular velocity and motor output for better visualization
        float scaled_velocity = kalman_velocity * 0.1f;  // Scale down by 10x
        float scaled_motor = motor_avg * 0.002f;  // Scale down by 500x (assuming max ~800)
        streamData(timestamp, raw_angle, kalman_angle, scaled_velocity, pid_output, 
                  pid_p, pid_i, pid_d, scaled_motor);
        endDataStream();
    }
    
public:
    /**
     * Log system event
     */
    void logEvent(const char* event) {
        if (enabled) {
            printf("# EVENT: %s at %u ms\n", event, k_uptime_get_32());
        }
    }
    
    /**
     * Log PID tuning parameters
     */
    void logTuning(float kp, float ki, float kd) {
        if (enabled) {
            printf("# PID_TUNING: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", 
                   (double)kp, (double)ki, (double)kd);
        }
    }
    
    /**
     * Log Kalman filter tuning
     */
    void logKalmanTuning(float q_angle, float q_velocity, float q_bias, float r_angle) {
        if (enabled) {
            printf("# KALMAN_TUNING: Q_angle=%.6f, Q_vel=%.6f, Q_bias=%.6f, R_angle=%.6f\n",
                   (double)q_angle, (double)q_velocity, (double)q_bias, (double)r_angle);
        }
    }
    
    /**
     * Log mode change
     */
    void logModeChange(const char* old_mode, const char* new_mode) {
        if (enabled) {
            printf("# MODE_CHANGE: %s -> %s at %u ms\n", 
                   old_mode, new_mode, k_uptime_get_32());
        }
    }
};

#endif // DATA_LOGGER_H