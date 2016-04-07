#include <stdio.h>

#include "libs/AVISIM/src/avisim.h"
#include "libs/AVISIM/src/algorithm.h"

int main()
{
    printf("Running AVISIM test...\n");
    
    AVISIMConfig config = {0};
    config.algorithm_init = init_algorithm;
    config.algorithm_eval = eval_algorithm;
    config.simulationFile = stdin;
    
    avisim_run(&config);    
}
