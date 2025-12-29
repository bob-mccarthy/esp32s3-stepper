#pragma once
#include "stdint.h"
#include "stdlib.h"
#define BUFFER_SIZE 200
#define NUM_AXIS 2

extern float segment_length;

struct segment_node{
    uint32_t steps[NUM_AXIS];
    uint32_t counter[NUM_AXIS];
    uint32_t max_steps;
    uint32_t steps_left; // how many iterations of the loop do we have left
};

struct segment_queue{
    uint16_t head;
    uint16_t tail; 
    uint16_t num_elements;
    struct segment_node *queue;
};

void init_segment_queue(struct segment_queue *new_queue);
uint8_t add_segment(struct segment_queue *seg_queue, uint32_t *steps);
struct segment_node pop_segment(struct segment_queue *seg_queue);

