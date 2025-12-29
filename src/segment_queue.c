#include "segment_queue.h"


void init_segment_queue(struct segment_queue *new_queue){
    new_queue->head = 0;
    new_queue->tail = 0;
    new_queue->num_elements = 0;
    new_queue->queue = (struct segment_node *) malloc(sizeof(struct segment_node) * BUFFER_SIZE);
}


uint8_t add_segment(struct segment_queue *seg_queue, uint32_t *steps){
    assert(seg_queue->num_elements != BUFFER_SIZE);
    struct segment_node *seg_node = &seg_queue->queue[seg_queue->tail];
    uint32_t max_steps = 0;
    for(int i = 0; i < NUM_AXIS; i++){
        seg_node->steps[i] = steps[i];
        seg_node->counter[i] = 0;
        if (steps[i] > max_steps){
            max_steps = steps[i];
        }
    }
    seg_node->max_steps = max_steps;
    seg_node->steps_left = max_steps;

    seg_queue->num_elements += 1;
    seg_queue->tail = (seg_queue->tail + 1) % BUFFER_SIZE; 
    
    return 1;
}

struct segment_node pop_segment(struct segment_queue *seg_queue){
    assert(seg_queue->num_elements != 0);
    struct segment_node popVal = seg_queue->queue[seg_queue->head];
    seg_queue->head = (seg_queue->head + 1) % BUFFER_SIZE;
    seg_queue->num_elements -= 1;
    return popVal;
}

