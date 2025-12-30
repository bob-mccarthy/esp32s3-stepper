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
#include "esp_rom_sys.h"

#include "hal/rmt_ll.h"
#include "hal/rmt_hal.h"
#include "soc/rmt_struct.h"
#include "soc/rmt_reg.h"
#include "driver/rmt_types.h"
#include "hal/rmt_types.h"
#include "esp_private/rmt.h"


void debug_rmt_registers(int channel_id) {
    // Access the global RMT register structure
    rmt_dev_t *dev = &RMT;
    static const char *TAG = "RMT_DEBUG";
    ESP_LOGI(TAG, "--- RMT Channel %d Register Debug ---", channel_id);

    // 1. Divider and Basic Config
    uint32_t conf0 = dev->chnconf0[channel_id].val;
    int div = dev->chnconf0[channel_id].div_cnt_chn;
    int carrier_en = dev->chnconf0[channel_id].carrier_en_chn;
    int wrap_en = dev->chnconf0[channel_id].mem_tx_wrap_en_chn;
    
    ESP_LOGI(TAG, "CONF0 (0x%08" PRIx32 "): Div=%d, Carrier=%d, Wrap=%d", 
             conf0, div, carrier_en, wrap_en);

    // 2. Carrier Settings 
    uint32_t carrier_high = dev->chncarrier_duty[channel_id].carrier_high_chn;
    uint32_t carrier_low = dev->chncarrier_duty[channel_id].carrier_low_chn;
    ESP_LOGI(TAG, "Carrier Duty: High Ticks=%lu, Low Ticks=%lu", carrier_high, carrier_low);

    // 3. Status and RAM Pointer
    uint32_t status = dev->chnstatus[channel_id].val;
    // On S3, bits 0-9 of status usually show the read pointer position
    ESP_LOGI(TAG, "Status Register: 0x%08" PRIx32, status);
    
    // 4. Memory Base and Content
    uint32_t mem_addr = DR_REG_RMT_BASE + 0x800 + (channel_id * 48 * 4);
    uint32_t val0 = *(volatile uint32_t*)mem_addr;
    uint32_t val1 = *(volatile uint32_t*)(mem_addr + 4);
    
    ESP_LOGI(TAG, "RAM Addr: 0x%08" PRIx32, mem_addr);
    ESP_LOGI(TAG, "RAM[0]: 0x%08" PRIx32 " (Dur0=%u, Lvl0=%u, Dur1=%u, Lvl1=%u)", 
             val0, (unsigned int)(val0 & 0x7FFF), (unsigned int)((val0 >> 15) & 1),
             (unsigned int)((val0 >> 16) & 0x7FFF), (unsigned int)((val0 >> 31) & 1));
    ESP_LOGI(TAG, "RAM[1]: 0x%08" PRIx32, val1);
    
    // 5. Global Config (Check for APB FIFO Access)
    // If bit 0 is set, the CPU cannot write to RAM properly while the RMT is active
    ESP_LOGI(TAG, "RMT_SYS_CONF: 0x%08" PRIx32, dev->sys_conf.val);
    
    ESP_LOGI(TAG, "--------------------------------------");
}
static const char *TAG = "STEPPER_MOTOR";

#define DIR_PIN 11
#define STEP_PIN 12

#define TEST_PIN 6

#define BUTTON_PIN 4

#define STEPPER_MOTOR_RESOLUTION 1000000 // 1Mhz
#define STEPPER_MOTOR_TICKS 2000
const static uint32_t uniform_speed_hz = 100000;
rmt_symbol_word_t freq_sample = {
    .level0 = 1,
    .duration0 = STEPPER_MOTOR_TICKS,
    .level1 = 0,
    .duration1 = 0,
};

volatile struct segment_queue *seg_queue = NULL;
volatile struct segment_node *curr_segment = NULL; 
bool refresh_next_segment_flag = false; 
rmt_dev_t *hw = &RMT;

void IRAM_ATTR trigger_pulse(int channel_id){
    
    hw->chnconf0[channel_id].mem_rd_rst_chn = 1;
    hw->chnconf0[channel_id].mem_rd_rst_chn = 0;
    // hw->chnconf0[channel_id].tx_stop_chn = 0;
    hw->chnconf0[channel_id].tx_start_chn = 1;
}

volatile rmt_channel_handle_t motor_chan = NULL;
rmt_encoder_handle_t uniform_motor_encoder = NULL;


float segment_length = 0.002;

rmt_encoder_handle_t *encoder;
rmt_transmit_config_t tx_config = {
    .loop_count = 1,
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
        .trans_queue_depth = 1, // set the number of transactions that can be pending in the background
        .intr_priority = 0,
        .flags.invert_out = 0,
        .flags.with_dma = 0
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, (rmt_channel_handle_t*) &motor_chan));

    // stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
    //     .resolution = STEPPER_MOTOR_RESOLUTION,
    // };
    
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

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
    
    // for(int i = 0; i < 10; i++){   
    //     gpio_set_level(TEST_PIN, 1);
    //     rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config);
    //     gpio_set_level(TEST_PIN, 0);
    // }
    int channel_id = -1;
    ESP_ERROR_CHECK(rmt_get_channel_id(motor_chan, &channel_id));
    hw->chnconf0[channel_id].mem_tx_wrap_en_chn = 0;
    hw->chnconf0[channel_id].carrier_en_chn = 0;
    hw->chnconf0[channel_id].div_cnt_chn = 80;
    hw->chnconf0[channel_id].conf_update_chn = 1;

    volatile uint32_t *chan_mem_ptr = (uint32_t *)(DR_REG_RMT_BASE + 0x800 + (channel_id * 48 * 4));
    uint32_t pulse_us = 5;
    uint32_t symbol = (1 << 15) | (pulse_us);
    rmt_symbol_word_t zero = {0,0,0,0};
    chan_mem_ptr[0] = symbol; // First data
    chan_mem_ptr[1] = 0;
    chan_mem_ptr[2] = 0;
    chan_mem_ptr[3] = 0;
    // 3. Ensure Wrap is DISABLED for raw manual pulses
    


    gpio_set_level(TEST_PIN, 1);
    // trigger_pulse(channel_id);
    for (int i = 0; i < 10; i++){
        trigger_pulse(channel_id);
        esp_rom_delay_us(8);
        // vTaskDelay(pdMS_TO_TICKS(2));
    }
    gpio_set_level(TEST_PIN, 0);

    
    
    while(1){
        // ESP_LOGI("debug", "channel id %d rmt pointer %p", channel_id, RMT_CH0DATA_REG);
        debug_rmt_registers(channel_id);
        // if(seg_queue->num_elements == BUFFER_SIZE){
        //     if (segments_created < acceleration_segments){
        //         generate_accel_segment(&p, step_buffer);
        //     }
        //     else if (segments_created < acceleration_segments + constant_vel_segments){
        //         generate_uniform_segment(&p, step_buffer);
        //     }
        //     else if (segments_created < total_segments){
        //         generate_decel_segment(&p, step_buffer);
        //     }
            
        //     if(segments_created < total_segments){
        //         add_segment(seg_queue, step_buffer);
        //         segments_created++;
        //     }
            
            
        // }
        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}