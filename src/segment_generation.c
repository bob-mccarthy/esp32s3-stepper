#include "segment_generation.h"

void generate_accel_segment(struct planning_data *p, uint32_t *steps_to_output){

    for(int i = 0; i < NUM_AXIS; i++){
        p->curr_dist[i] += segment_length * (p->curr_vel[i] + 0.5 * p->curr_accel[i] * segment_length);
        p->curr_vel[i] += p->curr_accel[i] * segment_length;
        steps_to_output[i] = (uint32_t) p->curr_dist[i] - p->steps_output[i];
        p->steps_output[i] = (uint32_t) p->curr_dist[i];
    }
    
}

void generate_uniform_segment(struct planning_data *p, uint32_t *steps_to_output){
    for(int i = 0; i < NUM_AXIS; i++){
        p->curr_dist[i] += segment_length * p->curr_vel[i];
        steps_to_output[i] = (uint32_t) p->curr_dist[i] - p->steps_output[i];
        p->steps_output[i] = (uint32_t) p->curr_dist[i];
    }
}

void generate_decel_segment(struct planning_data *p, uint32_t *steps_to_output){
    for(int i = 0; i < NUM_AXIS; i++){
        p->curr_dist[i] += segment_length * (p->curr_vel[i] - 0.5 * p->curr_accel[i] * segment_length);
		p->curr_vel[i] += p->curr_accel[i] * segment_length;
        steps_to_output[i] = (uint32_t) p->curr_dist[i] - p->steps_output[i];
        p->steps_output[i] = (uint32_t) p->curr_dist[i];
    }
}