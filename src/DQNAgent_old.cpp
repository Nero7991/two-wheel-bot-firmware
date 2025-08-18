// DQNAgent.cpp
#include "DQNAgent.h"
#include "math.h"
#include <stdio.h>

float DQNAgent::random() {
    // Implementation of a random number generator
    return 2.5;
}

DQNAgent::DQNAgent(float lr, float b1, float b2, float eps)
    : learning_rate(lr), beta1(b1), beta2(b2), epsilon(eps) {
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                weights[i][j][k] = 0.55;
            }
        }
    }
    //print weights for debugging
    #ifdef DEBUG_PRINT
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                printf(" W%d%d%d %f", i, j, k, weights[i][j][k]);
            }
            printf("\n");
        }
    }
    #endif

}

void DQNAgent::initialize(float lr, float b1, float b2, float eps) {
    learning_rate = lr;
    beta1 = b1;
    beta2 = b2;
    epsilon = eps;
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                weights[i][j][k] = 0.50;
            }
        }
    }
}


#ifdef STATS_PRINT
/* Function to print weights and biases */
void DQNAgent::print_stats() {
    
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                printf(" W%d%d%d %f", i, j, k, weights[i][j][k]);
            }
            printf("\n");
        }
    }
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int k = 0; k < output_size; ++k) {
            printf(" B%d%d %f", i, k, biases[i][k]);
        }
        printf("\n");
    }
   
}
 #endif

float DQNAgent::forward(const float inputs[NUM_INPUTS], float outputs[NUM_OUTPUTS]) {
    float activations[NUM_LAYERS + 1][NEURONS_PER_LAYER];
    for (int i = 0; i < NUM_INPUTS; ++i) {
        activations[0][i] = inputs[i];
    }

    for (int i = 0; i < NUM_LAYERS; ++i) {
        // Determine the size of input and output for each layer.
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int k = 0; k < output_size; ++k) {
            activations[i + 1][k] = 0.0; // Initialize the current neuron's activation to zero.
            for (int j = 0; j < input_size; ++j) {
                // Sum the weighted activations from the previous layer.
                //print weights for debugging
                #ifdef DEBUG_PRINT
                printf(" W%d%d%d %f", i, j, k, weights[i][j][k]);
                #endif
                activations[i + 1][k] += activations[i][j] * weights[i][j][k];
            }
            //print new line for debugging
            #ifdef DEBUG_PRINT
            printf("\n");
            #endif

            activations[i + 1][k] += biases[i][k]; // Add bias to the current neuron.
            // Apply the 'tanh' activation function, except for the output layer.
            // print activations for debugging
            #ifdef DEBUG_PRINT
            printf(" A%d%d %f", i, k, activations[i + 1][k]);
            #endif
            activations[i + 1][k] = (i == NUM_LAYERS - 1) ? activations[i + 1][k] : tanh(activations[i + 1][k]);
        }
        #ifdef DEBUG_PRINT
        printf("\n");
        #endif
    }

    for (int i = 0; i < NUM_OUTPUTS; ++i) {
        outputs[i] = activations[NUM_LAYERS][i];
    }

    // Calculate reward based on angular velocity
    float angle = inputs[0];
    float angular_velocity = inputs[1];
    float reward = 0.0;
    // if (abs(angular_velocity) < 0.1) {
    //     reward = 1.0;  // Positive reward for keeping angular velocity minimal
    // } else {
    //     reward = -1.0;  // Negative reward for increasing angular velocity
    // }
    // more positive reward for keeping the tilt angle between 0.11 and 0.12
    if (angle > 0.11 && angle < 0.12) {
        reward += 1.0;
    }
    // more negative reward for exceeding the tilt angle range
    if (angle < 0.10 || angle > 0.13) {
        reward -= 1.0;
    }
    // // Check if the tilt angle is within the defined range
    // float tilt_angle = inputs[0];
    // const float TILT_ANGLE_MIN = -30.0;
    // const float TILT_ANGLE_MAX = 30.0;
    // if (tilt_angle < TILT_ANGLE_MIN || tilt_angle > TILT_ANGLE_MAX) {
    //     reward = -10.0;  // Large negative reward for exceeding tilt angle range
    // }
    //print reward for debugging
    #ifdef DEBUG_PRINT
    printf("Reward: %f\n", reward);
    #endif
    return reward;
}

void DQNAgent::train(const float inputs[NUM_INPUTS], const float targets[NUM_OUTPUTS], float reward) {
    float activations[NUM_LAYERS + 1][NEURONS_PER_LAYER];
    for (int i = 0; i < NUM_INPUTS; ++i) {
        activations[0][i] = inputs[i];
    }

    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int k = 0; k < output_size; ++k) {
            activations[i + 1][k] = 0.0;
            for (int j = 0; j < input_size; ++j) {
                activations[i + 1][k] += activations[i][j] * weights[i][j][k];
            }
            activations[i + 1][k] += biases[i][k];
            activations[i + 1][k] = (i == NUM_LAYERS - 1) ? activations[i + 1][k] : tanh(activations[i + 1][k]);
        }
    }

    float deltas[NUM_LAYERS][NEURONS_PER_LAYER];
    for (int i = NUM_LAYERS - 1; i >= 0; --i) {
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < output_size; ++j) {
            if (i == NUM_LAYERS - 1) {
                deltas[i][j] = (targets[j] - activations[i + 1][j]) * reward;
            } else {
                deltas[i][j] = 0.0;
                for (int k = 0; k < NEURONS_PER_LAYER; ++k) {
                    deltas[i][j] += deltas[i + 1][k] * weights[i + 1][j][k] * (1 - pow(activations[i + 1][j], 2));
                    #ifdef DEBUG_PRINT
                    printf(" D%d%d %f", i, j, deltas[i][j]);
                    #endif
                }
            }
        }
    }

    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                m_weights[i][j][k] = beta1 * m_weights[i][j][k] + (1 - beta1) * deltas[i][k] * activations[i][j];
                v_weights[i][j][k] = beta2 * v_weights[i][j][k] + (1 - beta2) * pow(deltas[i][k] * activations[i][j], 2);
                double lrmw = learning_rate * m_weights[i][j][k];
                double sqvwe = sqrt(v_weights[i][j][k]) + epsilon;
                double temp = lrmw / sqvwe;
                weights[i][j][k] += temp;
                #ifdef DEBUG_PRINT
                printf(" W%d%d%d %f", i, j, k, weights[i][j][k]);
                printf(" Wm%d%d%d %f", i, j, k, m_weights[i][j][k]);
                printf(" Wv%d%d%d %f", i, j, k, v_weights[i][j][k]);
                printf(" Wlrmw%d%d%d %f", i, j, k, lrmw);
                printf(" Wsqvwe%d%d%d %f", i, j, k, sqvwe);
                printf(" Wt%d%d%d %f", i, j, k, temp);
                printf("\n");
                #endif
            }
        }
        for (int k = 0; k < output_size; ++k) {
            m_biases[i][k] = beta1 * m_biases[i][k] + (1 - beta1) * deltas[i][k];
            v_biases[i][k] = beta2 * v_biases[i][k] + (1 - beta2) * pow(deltas[i][k], 2);
            biases[i][k] += learning_rate * m_biases[i][k] / (sqrt(v_biases[i][k]) + epsilon);
        }
    }
}
