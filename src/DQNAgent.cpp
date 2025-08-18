// DQNAgent.cpp
#include "DQNAgent.h"
#include "math.h"
#include <stdio.h>


float DQNAgent::random() {
    // Function to approximate randomness with added fluctuation
    random_value += random_step;
    
    // Generate a pseudo-random value using a simple linear congruential generator
    random_seed = (1103515245 * random_seed + 12345) % 2147483648;
    float random_float = 0.0;
    if (random_seed != 0) {
        random_float = static_cast<float>(random_seed) / 2147483648.0;
    }
    
    // Add a random fluctuation to the step size
    float fluctuation = random_float * random_fluctuation * 2 - random_fluctuation;
    random_value += fluctuation;
    
    // Wrap around if random_value exceeds 1.0 or falls below 0.0
    if (random_value >= 1.0) {
        random_value = random_value - 1.0;
    } else if (random_value < 0.0) {
        random_value = 1.0 + random_value;
    }
    
    return random_value;
}

DQNAgent::DQNAgent(float lr, float b1, float b2, float eps)
    : learning_rate(lr), beta1(b1), beta2(b2), epsilon(eps) {
    // Initialize weights with random values from a normal distribution
    initialize(lr, b1, b2, eps);
}

void DQNAgent::initialize(float lr, float b1, float b2, float eps) {
    learning_rate = lr;
    beta1 = b1;
    beta2 = b2;
    epsilon = eps;
    random_value = 0.1;
    random_step = 0.1;
    random_fluctuation = 0.05;
    random_seed = 1;

    // other initialization
    replay_buffer_size = 0;
    update_counter = 0;
    

    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                weights[i][j][k] = random();
            }
        }
        // Initialize biases to zero
        for (int k = 0; k < output_size; ++k) {
            biases[i][k] = 0.0;
        }
    }
    // Initialize target network weights and biases
    for (int i = 0; i < NUM_LAYERS; ++i) {
        int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
        int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

        for (int j = 0; j < input_size; ++j) {
            for (int k = 0; k < output_size; ++k) {
                target_weights[i][j][k] = weights[i][j][k];
            }
        }
        for (int k = 0; k < output_size; ++k) {
            target_biases[i][k] = biases[i][k];
        }
    }
}

#ifdef STATS_PRINT
/* Function to print weights and biases */
void DQNAgent::print_stats() {
    /* print all variables */
    printf("learning_rate: %f\n", learning_rate);
    printf("beta1: %f\n", beta1);
    printf("beta2: %f\n", beta2);
    printf("epsilon: %f\n", epsilon);
    printf("random_value: %f\n", random_value);
    printf("random_step: %f\n", random_step);
    printf("random_fluctuation: %f\n", random_fluctuation);
    printf("random_seed: %d\n", random_seed);
    printf("update_counter: %d\n", update_counter);
    printf("replay_buffer_size: %d\n", replay_buffer_size);
    printf("REPLAY_BUFFER_CAPACITY: %d\n", REPLAY_BUFFER_CAPACITY);
    printf("MINIBATCH_SIZE: %d\n", MINIBATCH_SIZE);
    printf("TARGET_UPDATE_FREQ: %d\n", TARGET_UPDATE_FREQ);
    printf("EPSILON_DECAY: %f\n", EPSILON_DECAY);
    printf("MIN_EPSILON: %f\n", MIN_EPSILON);
    printf("LEARNING_RATE_DECAY: %f\n", LEARNING_RATE_DECAY);
    printf("MIN_LEARNING_RATE: %f\n", MIN_LEARNING_RATE);

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

    for (int i = 0; i < NUM_OUTPUTS; ++i) {
        outputs[i] = activations[NUM_LAYERS][i];
    }

    const float EPSILON = 1e-8;

    for (int i = 0; i < NUM_OUTPUTS; ++i) {
        // Apply sigmoid activation function to the output
        outputs[i] = 1.0 / (1.0 + exp(-activations[NUM_LAYERS][i]) + EPSILON);
        
        // Scale the output from (0, 1) to (-1, 1)
        outputs[i] = 2.0 * outputs[i] - 1.0;
    }

    return 0;
}

void DQNAgent::train(const float inputs[NUM_INPUTS], const float targets[NUM_OUTPUTS], float reward) {
    // Store current transition in replay buffer
    for (int i = 0; i < NUM_INPUTS; ++i) {
        replay_buffer[replay_buffer_size].inputs[i] = inputs[i];
    }
    for (int i = 0; i < NUM_OUTPUTS; ++i) {
        replay_buffer[replay_buffer_size].targets[i] = targets[i];
    }
    replay_buffer[replay_buffer_size].reward = reward;
    replay_buffer_size++;

    // If replay buffer is full, overwrite the oldest transition
    if (replay_buffer_size >= REPLAY_BUFFER_CAPACITY) {
        replay_buffer_size = 0;
    }

    // Sample a random minibatch of transitions from the replay buffer
    int minibatch_indices[MINIBATCH_SIZE];
    for (int i = 0; i < MINIBATCH_SIZE; ++i) {
        //minibatch_indices[i] = rand() % MIN(replay_buffer_size, REPLAY_BUFFER_CAPACITY);
        /* use recent */
        minibatch_indices[i] = (replay_buffer_size - MINIBATCH_SIZE + i + REPLAY_BUFFER_CAPACITY) % REPLAY_BUFFER_CAPACITY;
    }

    // Perform Q-learning updates on the minibatch
    for (int i = 0; i < MINIBATCH_SIZE; ++i) {
        int idx = minibatch_indices[i];
        float activations[NUM_LAYERS + 1][NEURONS_PER_LAYER];
        for (int j = 0; j < NUM_INPUTS; ++j) {
            activations[0][j] = replay_buffer[idx].inputs[j];
        }

        for (int j = 0; j < NUM_LAYERS; ++j) {
            int input_size = (j == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
            int output_size = (j == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

            for (int k = 0; k < output_size; ++k) {
                activations[j + 1][k] = 0.0;
                for (int m = 0; m < input_size; ++m) {
                    activations[j + 1][k] += activations[j][m] * weights[j][m][k];
                }
                activations[j + 1][k] += biases[j][k];
                activations[j + 1][k] = (j == NUM_LAYERS - 1) ? activations[j + 1][k] : tanh(activations[j + 1][k]);
            }
        }

        float deltas[NUM_LAYERS][NEURONS_PER_LAYER];
        for (int j = NUM_LAYERS - 1; j >= 0; --j) {
            int output_size = (j == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

            for (int k = 0; k < output_size; ++k) {
                if (j == NUM_LAYERS - 1) {
                    // Use target network for Q-value estimation
                    float target_value = 0.0;
                    for (int m = 0; m < NUM_OUTPUTS; ++m) {
                        target_value += replay_buffer[idx].targets[m] * target_weights[j][k][m];
                    }
                    target_value += target_biases[j][k];
                    deltas[j][k] = (target_value - activations[j + 1][k]) * replay_buffer[idx].reward;
                } else {
                    deltas[j][k] = 0.0;
                    for (int m = 0; m < NEURONS_PER_LAYER; ++m) {
                        deltas[j][k] += deltas[j + 1][m] * weights[j + 1][k][m] * (1 - pow(activations[j + 1][k], 2));
                    }
                }
            }
        }

        // Clip gradients to prevent exploding gradients
        for (int j = 0; j < NUM_LAYERS; ++j) {
            int output_size = (j == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;
            for (int k = 0; k < output_size; ++k) {
                deltas[j][k] = MAX(MIN(deltas[j][k], 1.0f), -1.0f);
            }
        }

        // Update weights and biases using Adam optimization
        for (int j = 0; j < NUM_LAYERS; ++j) {
            int input_size = (j == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
            int output_size = (j == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

            for (int k = 0; k < input_size; ++k) {
                for (int m = 0; m < output_size; ++m) {
                    m_weights[j][k][m] = beta1 * m_weights[j][k][m] + (1 - beta1) * deltas[j][m] * activations[j][k];
                    v_weights[j][k][m] = beta2 * v_weights[j][k][m] + (1 - beta2) * pow(deltas[j][m] * activations[j][k], 2);
                    weights[j][k][m] += learning_rate * m_weights[j][k][m] / (sqrt(v_weights[j][k][m] + epsilon) + epsilon);
                }
            }
            for (int m = 0; m < output_size; ++m) {
                m_biases[j][m] = beta1 * m_biases[j][m] + (1 - beta1) * deltas[j][m];
                v_biases[j][m] = beta2 * v_biases[j][m] + (1 - beta2) * pow(deltas[j][m], 2);
                biases[j][m] += learning_rate * m_biases[j][m] / (sqrt(v_biases[j][m] + epsilon) + epsilon);
            }
        }
    }

    // Update target network weights periodically
    if (update_counter % TARGET_UPDATE_FREQ == 0) {
        for (int i = 0; i < NUM_LAYERS; ++i) {
            int input_size = (i == 0) ? NUM_INPUTS : NEURONS_PER_LAYER;
            int output_size = (i == NUM_LAYERS - 1) ? NUM_OUTPUTS : NEURONS_PER_LAYER;

            for (int j = 0; j < input_size; ++j) {
                for (int k = 0; k < output_size; ++k) {
                    target_weights[i][j][k] = weights[i][j][k];
                }
            }
            for (int k = 0; k < output_size; ++k) {
                target_biases[i][k] = biases[i][k];
            }
        }
    }
    update_counter++;

    // Decay epsilon over time to reduce exploration
    epsilon = MAX(epsilon * EPSILON_DECAY, MIN_EPSILON);
    // Decay learning rate over time
    learning_rate = MAX(learning_rate * LEARNING_RATE_DECAY, MIN_LEARNING_RATE);
}