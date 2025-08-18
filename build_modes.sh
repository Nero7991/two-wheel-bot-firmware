#!/bin/bash

# Script to build STM32 firmware in different modes

echo "STM32 Two-Wheel Bot Build Script"
echo "================================="
echo ""
echo "Select build mode:"
echo "1) Inference mode (use pre-trained model)"
echo "2) Training mode (train on device)"
echo ""
read -p "Enter choice [1-2]: " choice

case $choice in
    1)
        echo "Building in INFERENCE mode..."
        west build -b nucleo_wl55jc -- -DUSE_INFERENCE_MODE=ON
        ;;
    2)
        echo "Building in TRAINING mode..."
        west build -b nucleo_wl55jc -- -DUSE_INFERENCE_MODE=OFF
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo "To flash: west flash"
else
    echo ""
    echo "Build failed!"
    exit 1
fi