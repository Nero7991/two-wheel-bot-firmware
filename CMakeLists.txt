
set(BOARD nucleo_wl55jc)
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(adeept_2wheelbot_zephyr)

# Option to choose between training mode and inference mode
option(USE_INFERENCE_MODE "Use pre-trained model inference instead of training" ON)

if(USE_INFERENCE_MODE)
    # Inference mode - uses pre-trained model
    message(STATUS "Building in INFERENCE mode - using pre-trained model")
    target_sources(app PRIVATE 
        src/main_inference.cpp 
        src/Motor.cpp
    )
else()
    # Training mode - trains on device
    message(STATUS "Building in TRAINING mode - trains on device")
    target_sources(app PRIVATE 
        src/main.cpp 
        src/DQNAgent.cpp 
        src/Motor.cpp
    )
endif()

# If you have any header files to include as well, 
# and they are not in your include path, you can use:
target_include_directories(app PRIVATE src/)

# Add path to the web simulator models directory for easy inclusion
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../twowheelbot-rl-web/models/)