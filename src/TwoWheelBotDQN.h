/**
 * Two-Wheel Balancing Robot DQN Model Interface
 * 
 * This file provides the trained neural network for balancing control.
 * Replace the weights arrays with those from your exported C++ model.
 */

#ifndef TWO_WHEEL_BOT_DQN_H
#define TWO_WHEEL_BOT_DQN_H

#include <math.h>

class TwoWheelBotDQN {
private:
    static const int INPUT_SIZE = 2;
    static const int HIDDEN_SIZE = 64;  // Adjust based on your model
    static const int OUTPUT_SIZE = 3;
    
    // Network weights - copy these from your exported model
    // These are placeholder values - replace with actual trained weights
    const float weightsInputHidden[INPUT_SIZE * HIDDEN_SIZE] = {
        // Insert weights from exported model here
        // Format: weightsInputHidden[INPUT_SIZE * HIDDEN_SIZE]
    };
    
    const float biasHidden[HIDDEN_SIZE] = {
        // Insert hidden biases from exported model here
    };
    
    const float weightsHiddenOutput[HIDDEN_SIZE * OUTPUT_SIZE] = {
        // Insert output weights from exported model here
    };
    
    const float biasOutput[OUTPUT_SIZE] = {
        // Insert output biases from exported model here
    };
    
    // Activation function (ReLU)
    float relu(float x) {
        return x > 0 ? x : 0;
    }
    
    float constrain(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    
public:
    /**
     * Get action from current state
     * @param angle Robot angle in radians
     * @param angularVelocity Angular velocity in rad/s
     * @return Action index (0=left, 1=brake, 2=right)
     */
    int getAction(float angle, float angularVelocity) {
        // Normalize inputs (same as in training)
        float input[INPUT_SIZE];
        input[0] = constrain(angle / (M_PI / 3), -1.0, 1.0);
        //input[0] = constrain(angle * 3, -1.0, 1.0);

        input[1] = constrain(-angularVelocity / 10.0, -1.0, 1.0);
        //input[1] = constrain(angularVelocity, -1.0, 1.0);

        
        // Hidden layer computation
        float hidden[HIDDEN_SIZE];
        for (int h = 0; h < HIDDEN_SIZE; h++) {
            hidden[h] = biasHidden[h];
            for (int i = 0; i < INPUT_SIZE; i++) {
                hidden[h] += input[i] * weightsInputHidden[i * HIDDEN_SIZE + h];
            }
            hidden[h] = relu(hidden[h]);
        }
        
        // Output layer computation
        float output[OUTPUT_SIZE];
        float maxValue = -1e10;
        int bestAction = 0;
        
        for (int o = 0; o < OUTPUT_SIZE; o++) {
            output[o] = biasOutput[o];
            for (int h = 0; h < HIDDEN_SIZE; h++) {
                output[o] += hidden[h] * weightsHiddenOutput[h * OUTPUT_SIZE + o];
            }
            
            // Track best action
            if (output[o] > maxValue) {
                maxValue = output[o];
                bestAction = o;
            }
        }
        
        return bestAction;
    }
    
    /**
     * Get all Q-values for debugging
     * @param angle Robot angle in radians
     * @param angularVelocity Angular velocity in rad/s
     * @param qValues Output array for Q-values
     */
    void getQValues(float angle, float angularVelocity, float qValues[OUTPUT_SIZE]) {
        // Normalize inputs
        float input[INPUT_SIZE];
        input[0] = constrain(angle / (M_PI / 3), -1.0, 1.0);
        input[1] = constrain(angularVelocity / 10.0, -1.0, 1.0);
        
        // Hidden layer computation
        float hidden[HIDDEN_SIZE];
        for (int h = 0; h < HIDDEN_SIZE; h++) {
            hidden[h] = biasHidden[h];
            for (int i = 0; i < INPUT_SIZE; i++) {
                hidden[h] += input[i] * weightsInputHidden[i * HIDDEN_SIZE + h];
            }
            hidden[h] = relu(hidden[h]);
        }
        
        // Output layer computation
        for (int o = 0; o < OUTPUT_SIZE; o++) {
            qValues[o] = biasOutput[o];
            for (int h = 0; h < HIDDEN_SIZE; h++) {
                qValues[o] += hidden[h] * weightsHiddenOutput[h * OUTPUT_SIZE + o];
            }
        }
    }
    
    /**
     * Get motor torque for action
     * @param action Action index
     * @return Motor torque (-1.0 to 1.0)
     */
    float getMotorTorque(int action) {
        const float actions[3] = {-1.0, 0.0, 1.0};
        return actions[action];
    }
};

#endif // TWO_WHEEL_BOT_DQN_H