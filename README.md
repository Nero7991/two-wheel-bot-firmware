# Two-Wheel Balancing Robot Firmware

STM32 firmware for a self-balancing two-wheel robot using reinforcement learning.

## Hardware

- **MCU**: STM32WL55 (Nucleo-WL55JC board)
- **IMU**: MPU6050 (I2C)
- **Motors**: DC motors with PWM control
- **Framework**: Zephyr RTOS

## Build Modes

The firmware supports two modes:

1. **Inference Mode** (default): Runs pre-trained neural network models
2. **Training Mode**: Experimental on-device Q-learning

## Quick Start

### Prerequisites

- Zephyr SDK and toolchain
- `west` build tool
- Pre-trained model from [web simulator](https://github.com/Nero7991/two-wheel-bot-sim-rl)

### Building

```bash
# Clone repository
git clone <repo-url>
cd adeept-2wheelbot

# Build with inference mode (default)
west build -b nucleo_wl55jc

# Or build with training mode
west build -b nucleo_wl55jc -- -DUSE_INFERENCE_MODE=OFF

# Flash to board
west flash
```

## Using Pre-trained Models

1. Train a model using the web simulator
2. Export as C++ code
3. Copy to `src/model.cpp`
4. Build and flash

Example models are available in the web simulator's `models/` directory.

## Project Structure

```
src/
├── main.cpp              # On-device training (experimental)
├── main_inference.cpp    # Inference with pre-trained models
├── model.cpp            # Your exported neural network
├── Motor.cpp/h          # Motor control
├── DQNAgent.cpp/h       # Q-learning implementation
└── TwoWheelBotDQN.h     # Model interface template
```

## Configuration

Key parameters in `main_inference.cpp`:

- `CONTROL_LOOP_PERIOD_MS`: Control frequency (default: 20ms = 50Hz)
- `MAX_MOTOR_SPEED`: PWM limit (default: 300)
- `ANGLE_OFFSET`: IMU calibration offset
- `MAX_TILT_ANGLE`: Fall detection threshold

## Simulation Mode

If hardware is unavailable, the firmware automatically enters simulation mode for performance benchmarking. This tests:

- Inference speed across different states
- Control loop timing at 50Hz
- Memory usage and throughput

## Performance

On STM32WL55 (48MHz Cortex-M4):
- Flash usage: ~57KB of 256KB
- RAM usage: ~8.6KB of 64KB
- Inference time: <200μs per decision
- Control rate: 50Hz sustained

## Troubleshooting

**MPU6050 not detected**: Check I2C connections and device tree overlay

**Motor not responding**: Verify PWM pins match your hardware configuration

**Model import fails**: Ensure C++ file includes proper class definition and M_PI macro

## License

MIT License - See LICENSE file for details