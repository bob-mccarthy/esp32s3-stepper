#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include <rom/ets_sys.h>
#include "esp_check.h"
#include "segment_queue.h"
#include "segment_generation.h"
#include "string.h"


static const char *TAG = "STEPPER_MOTOR";

#define DIR_PIN 11
#define STEP_PIN 12

#define TEST_PIN 6

#define BUTTON_PIN 4

#define STEPPER_MOTOR_RESOLUTION 1000000 // 1Mhz
const static uint32_t uniform_speed_hz = 100000;

volatile struct segment_queue *seg_queue = NULL;
volatile struct segment_node *curr_segment = NULL; 
bool refresh_next_segment_flag = false; 

volatile rmt_channel_handle_t motor_chan = NULL;
rmt_encoder_handle_t uniform_motor_encoder = NULL;


float segment_length = 0.002;

rmt_encoder_handle_t *encoder;
rmt_transmit_config_t tx_config = {
    .loop_count = 0,
};

typedef struct{
    //resolution of the tx channel in hz
    uint32_t resolution; 
    // number of interpolation points between start and end
    // min number of points has to be greater than the 
    // difference between the start and end frequency
    uint32_t sample_points; 
    uint32_t start_freq;
    uint32_t end_freq;
} stepper_motor_curve_encoder_config_t;


typedef struct {
    //resolution of the tx channel in hz
    uint32_t resolution;
} stepper_motor_uniform_encoder_config_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    uint32_t resolution;
} rmt_stepper_uniform_encoder_t;

static size_t rmt_encode_stepper_motor_uniform(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state){
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_encoder_handle_t copy_encoder = motor_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    uint32_t target_freq_hz = *(uint32_t *)primary_data;
    uint32_t symbol_duration = motor_encoder->resolution / target_freq_hz / 2;
    rmt_symbol_word_t freq_sample = {
        .level0 = 0,
        .duration0 = symbol_duration,
        .level1 = 1,
        .duration1 = symbol_duration,
    };
    size_t encoded_symbols = copy_encoder->encode(copy_encoder, channel, &freq_sample, sizeof(freq_sample), &session_state);
    *ret_state = session_state;
    return encoded_symbols;
}

static esp_err_t rmt_del_stepper_motor_uniform_encoder(rmt_encoder_t *encoder){
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_del_encoder(motor_encoder->copy_encoder);
    free(motor_encoder);
    return ESP_OK;
}

static esp_err_t rmt_reset_stepper_motor_uniform(rmt_encoder_t *encoder){
    rmt_stepper_uniform_encoder_t *motor_encoder = __containerof(encoder, rmt_stepper_uniform_encoder_t, base);
    rmt_encoder_reset(motor_encoder->copy_encoder);
    return ESP_OK;
}

esp_err_t rmt_new_stepper_motor_uniform_encoder(const stepper_motor_uniform_encoder_config_t *cfg, rmt_encoder_handle_t *ret_encoder){
    esp_err_t ret = ESP_OK;
    rmt_stepper_uniform_encoder_t *step_encoder = NULL;
    ESP_GOTO_ON_FALSE(cfg && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid arguments");
    step_encoder = rmt_alloc_encoder_mem(sizeof(rmt_stepper_uniform_encoder_t));
    ESP_GOTO_ON_FALSE(step_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for stepper uniform encoder");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &step_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    step_encoder->resolution = cfg->resolution;
    step_encoder->base.del = rmt_del_stepper_motor_uniform_encoder;
    step_encoder->base.encode = rmt_encode_stepper_motor_uniform;
    step_encoder->base.reset = rmt_reset_stepper_motor_uniform;
    *ret_encoder = &(step_encoder->base);
    return ESP_OK;
err:
    if (step_encoder) {
        if (step_encoder->copy_encoder) {
            rmt_del_encoder(step_encoder->copy_encoder);
        }
        free(step_encoder);
    }
    return ret;
}

volatile bool led_state = false; 


static bool pulse_interrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
    gpio_set_level(TEST_PIN, led_state);
    led_state = !led_state;
    
    // if(seg_queue == NULL || (seg_queue->num_elements == 0 && curr_segment == NULL)){
    //     return true;
    // }
    // if(curr_segment->steps_left == 0 && seg_queue->num_elements != 0){
    //     struct segment_node new_node = pop_segment((struct segment_queue *)seg_queue);
    //     memcpy((struct segment_node *)curr_segment, &new_node, sizeof(curr_segment));
    //     // curr_segment = seg_queue->num_elements; 
     
    // }
    // for(int i = 0; i < 1; i++){
	// 	curr_segment->counter[i] += curr_segment->steps[i];
	// 	if (curr_segment->counter[i] >= curr_segment->max_steps){
	// 		// printf("Just stepped axis num :%d\n", i);
            // rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config);
	// 		curr_segment->counter[i] -= curr_segment->max_steps;
	// 	}
	// }
    
    
    // ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
}

void app_main() {
    const char* INIT_TAG = "INIT";
    ESP_LOGI(INIT_TAG, "INITIALIZING GPIO PINS");
    gpio_config_t output_pin_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE, 
        .pin_bit_mask = 1ULL << DIR_PIN | 1ULL << STEP_PIN,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config_t button_pin_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t test_pin_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TEST_PIN,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    ESP_LOGI(INIT_TAG, "Set spin direction");
    gpio_set_level(DIR_PIN, 0);

    ESP_ERROR_CHECK(gpio_config(&output_pin_cfg));
    ESP_ERROR_CHECK(gpio_config(&test_pin_cfg));
    
    ESP_LOGI(INIT_TAG, "Create RMT TX channel");
    
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = STEPPER_MOTOR_RESOLUTION,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, (rmt_channel_handle_t*) &motor_chan));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEPPER_MOTOR_RESOLUTION,
    };
    
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_LOGI(INIT_TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
    // char queue[24];
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000*1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = {
    .reload_count = 0, // counter will reload with 0 on alarm event
    .alarm_count = 10,
    .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbf = {
        .on_alarm = pulse_interrupt,
    };
    // ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbf, NULL));
    // ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // ESP_ERROR_CHECK(gptimer_start(gptimer));
    struct segment_queue *seg_queue = (struct segment_queue *) malloc(sizeof(struct segment_queue));
    init_segment_queue(seg_queue);

    curr_segment = (struct segment_node*) malloc(sizeof(struct segment_node));

    struct planning_data p;
    p.curr_accel[0] = 500;
    p.curr_accel[1] = 130;
    p.curr_vel[0] = 0;
    p.curr_vel[1] = 0;
    p.curr_dist[0] = 0;
    p.curr_dist[1] = 0;
    p.steps_output[0] = 0;
    p.steps_output[1] = 0;
    uint32_t step_buffer[2];

    const int acceleration_time = 1; // in seconds
    const int constant_vel_time = 10;
    const int acceleration_segments = acceleration_time / segment_length;
    const int constant_vel_segments = constant_vel_time / segment_length;
    const int total_segments = acceleration_segments * 2 + constant_vel_segments;

    ESP_LOGI("segment generation", 
              "acceleration segments %d, constant_vel_segments %d, totalSegments %d", 
               acceleration_segments, constant_vel_segments, total_segments);

    int segments_created = 0;
    for(int i = 0; i < 500; i++){
        rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config);
    }
    // while(1){
    //     if(seg_queue->num_elements == BUFFER_SIZE){
    //         if (segments_created < acceleration_segments){
    //             generate_accel_segment(&p, step_buffer);
    //         }
    //         else if (segments_created < acceleration_segments + constant_vel_segments){
    //             generate_uniform_segment(&p, step_buffer);
    //         }
    //         else if (segments_created < total_segments){
    //             generate_decel_segment(&p, step_buffer);
    //         }
            
    //         if(segments_created < total_segments){
    //             add_segment(seg_queue, step_buffer);
    //             segments_created++;
    //         }
            
            
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(1));

    // }

}