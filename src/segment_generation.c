#include "segment_generation.h"
#include "esp_log.h"

//returns the number of segment_lengths it needs to output at least one step
int generate_accel_segment(struct planning_data *p, uint32_t *steps_to_output){
    uint8_t max_steps = 0;
    int num_segments = 0;
    do{
        for(int i = 0; i < NUM_AXIS; i++){
            //curr_dist[i] = delta_t * (vel * 1/2*accel*delta_t)
            p->curr_dist[i] += (segment_length * num_segments) * (p->curr_vel[i] + 0.5 * p->curr_accel[i] * (segment_length * num_segments));
            p->curr_vel[i] += p->curr_accel[i] * (segment_length * num_segments);
            steps_to_output[i] = ((uint32_t)p->curr_dist[i]) - p->steps_output[i];
            if (steps_to_output[i] > max_steps){
                max_steps = steps_to_output[i];
            }
            p->steps_output[i] = (uint32_t) p->curr_dist[i];
        }
        num_segments++;
    } while(max_steps == 0);
    return num_segments - 1;
}

int generate_uniform_segment(struct planning_data *p, uint32_t *steps_to_output){
    uint8_t max_steps = 0;
    int num_segments = 0;
    do{
        for(int i = 0; i < NUM_AXIS; i++){
            p->curr_dist[i] += (segment_length * num_segments) * p->curr_vel[i];
            steps_to_output[i] = (uint32_t) p->curr_dist[i] - p->steps_output[i];
            if (steps_to_output[i] > max_steps){
                max_steps = steps_to_output[i];
            }
            p->steps_output[i] = (uint32_t) p->curr_dist[i];
            
        }
        num_segments++;
    }while(max_steps == 0);
    return num_segments - 1; 
}

int generate_decel_segment(struct planning_data *p, uint32_t *steps_to_output){
    uint8_t max_steps = 0;
    int num_segments = 0;
    do{
        for(int i = 0; i < NUM_AXIS; i++){
            p->curr_dist[i] += (segment_length * num_segments) * (p->curr_vel[i] - 0.5 * p->curr_accel[i] * (segment_length * num_segments));
            p->curr_vel[i] -= p->curr_accel[i] * (segment_length * num_segments);
            steps_to_output[i] = (uint32_t) p->curr_dist[i] - p->steps_output[i];
            if (steps_to_output[i] > max_steps){
                max_steps = steps_to_output[i];
            }
            p->steps_output[i] = (uint32_t) p->curr_dist[i];
        }
        num_segments++;
    }while(max_steps == 0);
    return num_segments - 1;
}