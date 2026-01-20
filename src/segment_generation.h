#pragma once
#include "segment_queue.h"
#include "stdint.h"

extern float segment_length;

struct planning_data{
        float curr_vel[NUM_AXIS];  //steps/s
        float curr_accel[NUM_AXIS]; //steps/s/s
        float curr_dist[NUM_AXIS]; // therotical num of steps steps the stepper should have traveled (including fractions of steps)
        uint32_t steps_output[NUM_AXIS]; //number of steps sent to stepper motor
};
int generate_accel_segment(struct planning_data *p, uint32_t *steps_to_output);
int generate_decel_segment(struct planning_data *p, uint32_t *steps_to_output);
int generate_uniform_segment(struct planning_data *p, uint32_t *steps_to_output);