# PID Control with Kalman Filter for Two-Wheel Balancing Robot

## Overview
This implementation adds PID control with Kalman filtering to your two-wheel balancing robot, alongside the existing DQN control mode. The system now defaults to PID mode and includes real-time serial data output for visualization and tuning.

## Key Components

### 1. Kalman Filter (`KalmanFilter.h`)
The Kalman filter optimally fuses accelerometer and gyroscope data to estimate:
- **True tilt angle (θ)** - Filtered angle with reduced noise
- **True angular velocity (θ̇)** - Filtered angular rate
- **Gyroscope bias** - Compensates for sensor drift over time

**How it works:**
- **Prediction step**: Uses a motion model to predict the next state
- **Update step**: Corrects predictions using accelerometer measurements
- **Optimal fusion**: Balances trust between prediction and measurement based on noise characteristics

### 2. PID Controller (`PIDController.h`)
Standard PID controller with enhancements:
- **P (Proportional)**: Responds to current angle error
- **I (Integral)**: Eliminates steady-state error
- **D (Derivative)**: Provides damping using angular velocity
- **Anti-windup**: Prevents integral term saturation
- **Output limiting**: Ensures motor commands stay within bounds

### 3. Control Modes (`ControlMode.h`)
- **PID_MODE** (default): Uses PID with Kalman filtering
- **DQN_MODE**: Uses pre-trained neural network
- **MANUAL_MODE**: Direct keyboard control
- **DISABLED**: Emergency stop

### 4. Data Logger (`DataLogger.h`)
Outputs CSV-formatted data via serial for real-time plotting:
```csv
timestamp,seq,mode,raw_angle,raw_gyro,kalman_angle,kalman_velocity,kalman_bias,pid_output,motor_left,motor_right
```

## Control Commands

Press these keys during operation:

| Key | Function |
|-----|----------|
| **A/D** or **←/→** | Manual left/right control |
| **S** or **Space** | Brake |
| **M** | Toggle between PID/DQN modes |
| **L** | Toggle data logging on/off |
| **P/O** | Increase/decrease Kp by 5 |
| **I/U** | Increase/decrease Ki by 0.5 |
| **R** | Resume automatic control |
| **H/?** | Show help |

## Building and Flashing

```bash
# Build the project (defaults to inference mode with PID)
west build -b nucleo_wl55jc

# Flash to board
west flash
```

## Real-Time Data Visualization

### Web-Based Plotter (Recommended)
```bash
# One-time setup
cd web-plotter && ./setup.sh

# Start web server
./start-web-plotter.sh

# Open browser to http://localhost:3001
# Select serial port from dropdown and connect
```

### Features
- **Web interface** - Access from any device with browser
- **Port selection** - Choose serial port directly in the web UI
- **Real-time charts** - 4 synchronized plots with 60fps updates
- **Interactive controls** - Connect/disconnect without command line
- **Multiple viewers** - Share the dashboard with team members
- **Responsive design** - Works on desktop, tablet, and mobile
- **Auto-reconnection** - Handles disconnections gracefully

### Using Serial Monitor (Alternative)
```bash
# View raw data stream
minicom -D /dev/ttyUSB0 -b 115200

# Or using screen
screen /dev/ttyUSB0 115200
```

## PID Tuning Guide

### Starting Values
```c
Kp = 50.0f   // Proportional gain
Ki = 0.0f    // Integral gain (start with 0)
Kd = 2.0f    // Derivative gain
```

### Tuning Process
1. **Start with P only** (Ki=0, Kd=0)
   - Increase Kp until robot responds quickly but oscillates
   - Reduce Kp to about 70% of oscillation point

2. **Add D for damping**
   - Increase Kd to reduce oscillations
   - Too much Kd causes jerky motion

3. **Add I if needed**
   - Small Ki eliminates steady-state error
   - Too much Ki causes instability

### Live Tuning
Use keyboard commands while robot is running:
- Press **P** to increase Kp by 5
- Press **O** to decrease Kp by 5
- Press **I** to increase Ki by 0.5
- Press **U** to decrease Ki by 0.5

## Kalman Filter Parameters

Located in `main_inference.cpp`:
```c
#define KALMAN_Q_ANGLE 0.001f      // Process noise for angle
#define KALMAN_Q_VELOCITY 0.003f   // Process noise for velocity
#define KALMAN_Q_BIAS 0.00003f     // Process noise for bias
#define KALMAN_R_ANGLE 0.03f       // Measurement noise
```

**Tuning tips:**
- Increase R_ANGLE if accelerometer is noisy
- Decrease Q values for smoother but slower response
- Increase Q values for faster but noisier response

## Data Analysis

The CSV output can be imported into:
- **Excel/LibreOffice**: For offline analysis
- **MATLAB**: For advanced control analysis
- **Python**: Using pandas for data processing

Example Python analysis:
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('robot_data.csv', comment='#')

# Plot angle tracking
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp']/1000, df['raw_angle'], alpha=0.3, label='Raw')
plt.plot(df['timestamp']/1000, df['kalman_angle'], label='Filtered')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.show()

# Calculate RMS error
rms_error = np.sqrt(np.mean(df['kalman_angle']**2))
print(f"RMS angle error: {rms_error:.4f} rad")
```

## Understanding the Control Loop

```
MPU6050 Sensor → Raw Data → Kalman Filter → Filtered State
                                  ↓
                            PID Controller
                                  ↓
                            Motor Command → Motors
                                  ↓
                            Serial Output → PC
```

### Loop Timing
- Runs at 50Hz (20ms period)
- Kalman filter updates with each sensor reading
- PID calculates control output
- Data logged for analysis

## Troubleshooting

### Robot falls immediately
- Reduce Kp significantly
- Check angle calibration offset
- Verify motor direction

### Oscillations
- Reduce Kp
- Increase Kd
- Check for mechanical play

### Slow response
- Increase Kp
- Reduce Kd if overdamped

### Drift over time
- Add small Ki (start with 0.5)
- Check Kalman bias estimation

## Safety Features
- Automatic fall detection and recovery
- Motor disable on excessive tilt (>30°)
- Emergency stop mode
- Consecutive fall counter

## Next Steps
1. Tune PID gains for your specific robot
2. Experiment with different Kalman filter parameters
3. Compare PID vs DQN performance
4. Implement adaptive control or gain scheduling
5. Add disturbance rejection testing