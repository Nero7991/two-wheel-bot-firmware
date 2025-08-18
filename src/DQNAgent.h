// DQNAgent.h
#ifndef DQNAGENT_H
#define DQNAGENT_H

#define NUM_INPUTS 2
#define NUM_OUTPUTS 2
#define NUM_LAYERS 3
#define NEURONS_PER_LAYER 24

#define REPLAY_BUFFER_CAPACITY 2
#define MINIBATCH_SIZE 4
#define TARGET_UPDATE_FREQ 16

// Variables for epsilon-greedy exploration
#define EPSILON_DECAY 0.995
#define MIN_EPSILON 0.3

// Variables for learning rate decay
#define LEARNING_RATE_DECAY  0.99
#define MIN_LEARNING_RATE 0.001

#define STATS_PRINT
// #define DEBUG_PRINT

#include "stdlib.h"
#include <zephyr/sys/util.h>

class DQNAgent {
private:
    float weights[NUM_LAYERS][NEURONS_PER_LAYER][NEURONS_PER_LAYER];
    float biases[NUM_LAYERS][NEURONS_PER_LAYER];
    float m_weights[NUM_LAYERS][NEURONS_PER_LAYER][NEURONS_PER_LAYER];
    float v_weights[NUM_LAYERS][NEURONS_PER_LAYER][NEURONS_PER_LAYER];
    float m_biases[NUM_LAYERS][NEURONS_PER_LAYER];
    float v_biases[NUM_LAYERS][NEURONS_PER_LAYER];
    float learning_rate;
    float beta1;
    float beta2;
    float epsilon;

        // Variables for target network
    float target_weights[NUM_LAYERS][NEURONS_PER_LAYER][NEURONS_PER_LAYER];
    float target_biases[NUM_LAYERS][NEURONS_PER_LAYER];

public:
    // Variables for experience replay
    struct Transition {
        float inputs[NUM_INPUTS];
        float targets[NUM_OUTPUTS];
        float reward;
    };
    Transition replay_buffer[1000];
    int replay_buffer_size = 0;

    
    int update_counter = 0;

    // Variables for random number generation
    float random_value = 0.1;
    float random_step = 0.1;
    float random_fluctuation = 0.05;
    int random_seed = 1;

    DQNAgent(float lr, float b1, float b2, float eps);
    float forward(const float inputs[NUM_INPUTS], float outputs[NUM_OUTPUTS]);
    void train(const float inputs[NUM_INPUTS], const float targets[NUM_OUTPUTS], float reward);
    void initialize(float lr, float b1, float b2, float eps);
    float random();
    #ifdef STATS_PRINT
    void print_stats();
    #endif
};

#endif
