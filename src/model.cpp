/**
 * Two-Wheel Balancing Robot DQN Model
 * Generated: 2025-08-24T19-44-01
 * Architecture: 2-64-3
 * 
 * This file contains the trained neural network weights for deployment
 * on embedded systems (Arduino, ESP32, STM32, etc.)
 */

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class TwoWheelBotDQN {
private:
    static const int INPUT_SIZE = 2;
    static const int HIDDEN_SIZE = 64;
    static const int OUTPUT_SIZE = 3;
    
    // Network weights (stored in program memory to save RAM)
    const float weightsInputHidden[INPUT_SIZE * HIDDEN_SIZE] = {
                6.222715f, -0.318137f, 4.286941f, -3.232339f, 0.295613f, 0.157134f, 1.650019f, 1.200162f,
        -1.553828f, -4.472485f, 0.604582f, 1.154443f, 4.401635f, 0.534327f, 4.747355f, 1.309479f,
        3.139891f, -2.190660f, 0.109789f, 0.879768f, -1.115382f, 5.606535f, -2.441700f, -1.904559f,
        2.053846f, -1.454618f, -1.089492f, 0.904795f, -2.664482f, -3.135923f, -1.891645f, 1.985992f,
        -4.195805f, 6.058016f, 0.979177f, -4.751349f, -2.246619f, -0.759899f, 1.500413f, -0.745927f,
        3.009539f, -2.186470f, -3.079055f, 1.652231f, -1.740592f, -2.361685f, 0.681416f, -3.269698f,
        0.983687f, 3.676093f, -4.701682f, -1.552671f, 1.934866f, 0.762604f, 2.389984f, 0.358032f,
        -3.342948f, -6.709599f, 6.382307f, 3.854782f, 2.821023f, -3.427545f, 4.470200f, 2.269878f,
        2.230539f, 2.133920f, 2.943488f, -5.061087f, -1.436117f, 3.872705f, 1.036283f, -0.668306f,
        0.151802f, -4.851257f, 1.940896f, 3.772658f, 3.299453f, 3.365771f, 5.710608f, -0.990288f,
        1.751844f, -2.669120f, 0.539213f, 0.917723f, -0.520971f, 4.179123f, -2.059248f, -2.365124f,
        1.172247f, -2.530289f, 2.023068f, 1.315187f, 0.504383f, -5.208951f, -1.044872f, 0.836747f,
        -1.676268f, 4.435410f, 1.580129f, -4.372157f, -4.142663f, 0.339476f, 3.063086f, 0.148101f,
        2.462527f, -2.690839f, -4.629966f, 3.041430f, -3.343761f, -2.500673f, 1.292748f, -3.882821f,
        2.167084f, 3.210040f, -5.481805f, -1.873649f, 0.760085f, 1.159282f, 3.492776f, 1.273541f,
        -3.354005f, -5.418931f, 3.576872f, 0.966506f, 0.970869f, 1.528226f, 1.784555f, 1.970077f
    };
    
    const float biasHidden[HIDDEN_SIZE] = {
                9.999868f, 9.999989f, 9.999859f, 9.999860f, 8.460224f, 8.749870f, 8.972045f, 9.999883f,
        8.468541f, 7.016017f, 9.999993f, 8.291431f, 9.518993f, 8.310575f, 8.219358f, 6.755226f,
        9.999852f, -1.034658f, 9.999930f, -0.431626f, 9.999897f, 8.632594f, -1.179572f, -0.935918f,
        9.999887f, 1.513476f, 8.521672f, 7.950207f, 8.155802f, -1.388949f, -0.874964f, 4.361211f,
        0.134857f, -1.376296f, 8.484853f, -1.298243f, -1.081423f, 7.706805f, 8.919501f, 9.999992f,
        7.788467f, -1.073916f, -1.529078f, 6.660520f, 8.670018f, -1.151396f, 8.807898f, 9.999823f,
        9.659616f, 8.351704f, 6.142562f, -0.761652f, 8.201668f, 7.670483f, 8.692678f, 8.090070f,
        -1.501237f, 9.999974f, -1.341383f, 8.265377f, 8.803180f, 9.102405f, 8.456149f, 8.519716f
    };
    
    const float weightsHiddenOutput[HIDDEN_SIZE * OUTPUT_SIZE] = {
                -1.194365f, 0.693859f, -0.609908f, -0.592706f, 0.185939f, 0.517918f, 2.083270f, 1.350751f,
        -1.124856f, -1.019380f, 1.007876f, -0.738012f, -1.410509f, -0.584025f, 0.713185f, 0.050283f,
        -0.914392f, -0.639177f, -0.401894f, -0.901032f, -0.348410f, -2.039094f, 0.730191f, 0.304767f,
        0.898629f, -0.136585f, -0.287173f, 2.515662f, -1.453566f, -0.849370f, -1.003271f, 0.956511f,
        1.657715f, 0.409099f, 0.438902f, 0.626402f, -0.323191f, -0.372430f, 0.240402f, 0.526650f,
        0.066375f, 0.156999f, 0.289346f, -0.335630f, -1.044346f, 0.796426f, -1.429243f, -1.277750f,
        -0.049115f, 0.571124f, -1.595264f, -0.356910f, -1.809747f, -1.324083f, -0.966232f, 0.124769f,
        0.496331f, 10.000000f, 9.878751f, 9.718864f, -1.273098f, 0.469503f, -0.210536f, -0.089975f,
        -0.409755f, 0.377155f, -1.821043f, 0.153059f, -2.831691f, -1.670280f, -1.042745f, -1.424654f,
        -0.882469f, 0.039029f, -0.399174f, -0.964358f, 0.714726f, -2.156274f, 0.862391f, 0.059272f,
        -1.400213f, 0.978418f, 0.122792f, 0.087116f, 0.802574f, 0.567951f, -0.084874f, 0.474541f,
        -1.467037f, -3.062065f, -1.170264f, 0.244769f, -3.226044f, 1.391232f, -3.157122f, 0.204045f,
        -2.730379f, -2.246881f, -2.670165f, -4.571266f, -3.844946f, 1.149924f, 1.861050f, 0.082977f,
        -0.552188f, -0.313213f, -2.206647f, -3.675696f, -0.010873f, -1.007838f, -2.987340f, 0.900961f,
        -0.159421f, 0.022674f, -0.046725f, -0.861038f, -0.599673f, -1.664219f, 0.855422f, 1.683715f,
        -3.691702f, 0.384162f, 1.084237f, -2.838170f, -1.119310f, 0.891948f, -0.741177f, -1.475960f,
        -3.537708f, 0.455548f, -0.888558f, -0.341760f, 0.069014f, 0.174453f, 0.761405f, -2.003209f,
        -1.978797f, -0.605727f, -0.643398f, -0.586302f, 1.213295f, -2.452324f, 0.404782f, 1.164273f,
        -0.552112f, -0.363922f, 0.312328f, 2.067849f, -0.310926f, -0.329316f, 4.025442f, -1.902952f,
        -0.672585f, -1.273117f, -0.861920f, -2.036434f, 0.525059f, 0.244796f, 0.390848f, 1.198777f,
        -0.131555f, 0.000562f, -2.860880f, 1.308783f, 0.982413f, 1.374589f, 0.196381f, -0.085240f,
        -1.469869f, -2.072927f, 1.304597f, -0.297197f, 1.375885f, 0.553922f, -2.277864f, -3.122234f,
        -1.507688f, 1.925275f, 0.027196f, -0.430354f, -2.217910f, -0.657057f, 2.225158f, 0.473231f,
        -0.583069f, -1.194912f, 2.559737f, -0.203895f, -0.565872f, 2.162595f, -0.075215f, -0.553681f
    };
    
    const float biasOutput[OUTPUT_SIZE] = {
                8.276086f, 9.999952f, 8.359546f
    };
    
    // Activation function (ReLU)
    float relu(float x) {
        return x > 0 ? x : 0;
    }
    
public:
    /**
     * Get action from current state
     * @param angle Robot angle in radians
     * @param angularVelocity Angular velocity in rad/s
     * @return Action index (0=left, 1=brake, 2=right)
     */
    int getAction(float angle, float angularVelocity) {
        // Normalize inputs
        float input[INPUT_SIZE];
        input[0] = constrain(angle / (float)(M_PI / 3), -1.0f, 1.0f);
        input[1] = constrain(angularVelocity / 10.0f, -1.0f, 1.0f);
        
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
     * Get motor torque for action
     * @param action Action index
     * @return Motor torque (-1.0 to 1.0)
     */
    float getMotorTorque(int action) {
        const float actions[3] = {-1.0, 0.0, 1.0};
        return actions[action];
    }
    
private:
    float constrain(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

// Usage example:
// TwoWheelBotDQN bot;
// int action = bot.getAction(angle, angularVelocity);
// float torque = bot.getMotorTorque(action);
