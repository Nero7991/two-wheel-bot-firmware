# Using Pre-trained Models from Web Simulator

This guide explains how to use models trained in the web simulator on your STM32 microcontroller.

## Quick Start

1. **Train a model** using the web simulator at https://botsim.orenslab.com or locally

2. **Export the model** as C++ code using the "Export C++" button

3. **Copy the exported file** to the STM32 project:
   ```bash
   # Copy directly as .cpp (works fine)
   cp ~/Downloads/two_wheel_bot_dqn_*.cpp src/model.cpp
   
   # Or rename to .h if preferred
   cp ~/Downloads/two_wheel_bot_dqn_*.cpp src/model.h
   ```

4. **Update the include** in `src/main_inference.cpp`:
   ```cpp
   // Change this line to point to your model file:
   #include "model.cpp"  // or #include "model.h"
   ```

5. **Build and flash** the firmware:
   ```bash
   west build -b nucleo_wl55jc
   west flash
   ```

## File Structure

- `main_inference.cpp` - Main inference program (no training on device)
- `main.cpp` - Original training program (trains on device)
- `model.cpp` or `model.h` - Your exported model from web simulator

## Using Pre-trained Models

The `models/` directory in the web simulator contains pre-trained models:
- `good_two_wheel_bot_dqn_2025-08-17T19-28-58.cpp` - Well-trained model
- `okayish_two_wheel_bot_dqn_2025-08-17T17-26-44.cpp` - Partially trained model

To use these:
```bash
# Copy pre-trained model to STM32 project
cp ../twowheelbot-rl-web/models/good_two_wheel_bot_dqn_*.cpp src/model.cpp
```

## Key Parameters to Adjust

In `main_inference.cpp`, you may need to adjust:

- `ANGLE_OFFSET` - Calibration offset for your specific robot (default: 0.115f)
- `MAX_MOTOR_SPEED` - Maximum PWM value for motors (default: 1000)
- `MAX_TILT_ANGLE` - Safety limit in radians (default: 0.5f ≈ 30°)
- `ROLLING_AVERAGE_SIZE` - Sensor smoothing (default: 4)

## Sensor Mapping

The model expects:
- **Input 0**: Tilt angle in radians (normalized to ±π/3)
- **Input 1**: Angular velocity in rad/s (normalized to ±10)

The MPU6050 mapping:
- Tilt angle: Calculated from Y-axis accelerometer
- Angular velocity: X-axis gyroscope (pitch rate)

## Motor Control

The model outputs three actions:
- Action 0: Move backward (torque = -1.0)
- Action 1: Brake (torque = 0.0)  
- Action 2: Move forward (torque = 1.0)

Both wheels move together for balance control.

## Debugging

Enable debug output by adding to `prj.conf`:
```
CONFIG_LOG=y
CONFIG_APP_LOG_LEVEL_DBG=y
```

Or compile with debug flag:
```bash
west build -b nucleo_wl55jc -- -DDEBUG_INFERENCE=1
```

## Safety Features

- Automatic motor stop if robot falls repeatedly
- Continuous fall detection and recovery attempts
- Status reporting every second
- Calibration routine on startup

## Simulation Mode

If the MPU6050 sensor or motor driver is not available, the system automatically enters **simulation mode** to benchmark inference performance. This is useful for:

- Testing the model on the board without hardware
- Benchmarking inference speed
- Verifying the model loads correctly

### Simulation Mode Features:

1. **Inference Benchmark**: Tests various robot states and measures inference time
2. **Throughput Test**: Runs 1000 iterations to measure average performance
3. **50Hz Control Loop Test**: Simulates 10 seconds of real-time control to check if the system can maintain timing
4. **Performance Metrics**:
   - Inference time per state (microseconds)
   - Maximum theoretical frequency
   - Control loop deadline tracking
   - Action distribution analysis

### Expected Performance:

On STM32WL55 (48MHz Cortex-M4):
- Single inference: ~50-200 microseconds (depending on model size)
- With sensor/motor overhead: ~600 microseconds total
- Theoretical max frequency: >1000 Hz
- 50Hz control loop: Should achieve 0% missed deadlines

## Switching Between Training and Inference

- Use `main.cpp` for on-device training (experimental)
- Use `main_inference.cpp` for running pre-trained models (recommended)

Simply update your CMakeLists.txt to choose which main file to compile.