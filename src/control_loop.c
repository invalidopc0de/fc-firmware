#include "algorithm.h"

#include <stdint.h>

uint32_t samples[30];
uint32_t* current_sample;

enum RocketState
{
    ROCKETSTATE_PRELAUNCH   = 1,
    ROCKETSTATE_LAUNCH      = 2,
    ROCKETSTATE_MECO        = 3, // Main Engine Cut Off
    ROCKETSTATE_DPLY_DROGE  = 4,
    ROCKETSTATE_DPLY_MAIN   = 5     
};

uint32_t currentRocketState = 0;

// Called on device initialization
void init_algorithm(DeviceSpec* device)
{
    current_sample = samples;
}

// Called on an interval
void eval_algorithm(DeviceSpec* device)
{
    // Read from the sensors
}
