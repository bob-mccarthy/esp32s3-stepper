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

volatile struct segment_queue *seg_queue = NULL;
volatile int segments_created = 0;
struct segment_node *curr_segment = NULL; 
bool refresh_next_segment_flag = false; 
rmt_dev_t *hw = &RMT;
volatile int rmt_channel_id = -1;
void IRAM_ATTR trigger_pulse(int channel_id){
    
    hw->chnconf0[channel_id].mem_rd_rst_chn = 1;
    hw->chnconf0[channel_id].mem_rd_rst_chn = 0;
    // hw->chnconf0[channel_id].tx_stop_chn = 0;
    hw->chnconf0[channel_id].tx_start_chn = 1;
}

volatile rmt_channel_handle_t motor_chan = NULL;
volatile rmt_channel_handle_t test_chan = NULL;


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
gptimer_alarm_config_t alarm_config_one = {
    .reload_count = 0,      // When the alarm event occurs, the timer will automatically reload to 0
    .alarm_count = 10, // Set the actual alarm period, since the resolution is 1us, 1000000 represents 1s
    .flags.auto_reload_on_alarm = true, // Enable auto-reload function
};

gptimer_alarm_config_t alarm_config_two = {
    .reload_count = 0,      // When the alarm event occurs, the timer will automatically reload to 0
    .alarm_count = 50, // Set the actual alarm period, since the resolution is 1us, 1000000 represents 1s
    .flags.auto_reload_on_alarm = true, // Enable auto-reload function
};

volatile bool alarm_state = false; 

volatile uint8_t loop_count = 0;
volatile int timerTicks = 2000;
static bool pulse_interrupt(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + timerTicks, // Next alarm in 1s from the current alarm
    };
    // Update the alarm value
    int currTimerTicks = timerTicks;
    gptimer_set_alarm_action(timer, &alarm_config);
    // if(loop_count >= 10){
    //     ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, alarm_state ? &alarm_config_one : &alarm_config_two));
    //     alarm_state = !alarm_state;
    //     loop_count = 0;
    // }
    // if(rmt_channel_id != -1){
    //     trigger_pulse(rmt_channel_id);
    // }
    // loop_count++;
    
    // ESP_LOGI("INTERRUPT", "Just over hear testing");
    if(seg_queue == NULL || (seg_queue->num_elements == 0 && curr_segment->steps_left == 0)){
        return true;
    }

    
    if (curr_segment->steps_left > 0){
        for(int i = 0; i < 1; i++){
            curr_segment->counter[i] += curr_segment->steps[i];
            if (curr_segment->counter[i] >= curr_segment->max_steps){
                trigger_pulse(rmt_channel_id);
                curr_segment->steps_left -= 1;
                curr_segment->counter[i] -= curr_segment->max_steps;
            }
        }
    }

     if(curr_segment->steps_left == 0 && seg_queue->num_elements > 0){
            struct segment_node new_node = pop_segment((struct segment_queue *)seg_queue);
            if(new_node.isrTicks < 50){
                return true;
            }
            memcpy(curr_segment, &new_node, sizeof(struct segment_node));
            // ESP_LOGI("segment generation", "curr_segment with %lu steps left", curr_segment->steps_left);
            // memcpy(&tmp_node, &new_node, sizeof(new_node));
            // ESP_LOGI("segment generation", "curr_segment with %lu steps left", curr_segment->steps_left);
            timerTicks = curr_segment->isrTicks;
    }
    

    
    
    
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

    rmt_tx_channel_config_t tx_chan_config_test = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = TEST_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = STEPPER_MOTOR_RESOLUTION,
        .trans_queue_depth = 1, // set the number of transactions that can be pending in the background
        .intr_priority = 0,
        .flags.invert_out = 0,
        .flags.with_dma = 0
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_test, (rmt_channel_handle_t*) &test_chan));

    // stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
    //     .resolution = STEPPER_MOTOR_RESOLUTION,
    // };
    
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    ESP_LOGI(INIT_TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
    ESP_ERROR_CHECK(rmt_enable(test_chan));

    seg_queue = (struct segment_queue *) calloc(1, sizeof(struct segment_queue));
    init_segment_queue((struct segment_queue *)seg_queue);

    curr_segment = (struct segment_node*) calloc(1, sizeof(struct segment_node));
    curr_segment->steps_left = 0;

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
    .alarm_count = 2000,
    .flags.auto_reload_on_alarm = false, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbf = {
        .on_alarm = pulse_interrupt,
    };


   
    


    struct planning_data p;
    p.curr_accel[0] = 2000;
    // p.curr_accel[1] = 1500;
    p.curr_vel[0] = 0;
    // p.curr_vel[1] = 0;
    p.curr_dist[0] = 0;
    // p.curr_dist[1] = 0;
    p.steps_output[0] = 0;
    // p.steps_output[1] = 0;
    uint32_t step_buffer[1];

    const int acceleration_time = 2; // in seconds
    const int constant_vel_time = 5;
    const int acceleration_segments = acceleration_time / segment_length;
    const int constant_vel_segments = constant_vel_time / segment_length;
    const int total_segments = acceleration_segments * 2 + constant_vel_segments;

    ESP_LOGI("segment generation", 
              "acceleration segments %d, constant_vel_segments %d, totalSegments %d", 
               acceleration_segments, constant_vel_segments, total_segments);

   
    
    // for(int i = 0; i < 10; i++){   
    //     gpio_set_level(TEST_PIN, 1);
    //     rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config);
    //     gpio_set_level(TEST_PIN, 0);
    // }
    int channel_id = -1;
    ESP_ERROR_CHECK(rmt_get_channel_id(motor_chan, &channel_id));
    rmt_channel_id = channel_id; 
    int channel_id_test = -1;
    ESP_ERROR_CHECK(rmt_get_channel_id(test_chan, &channel_id_test));

    hw->chnconf0[channel_id].mem_tx_wrap_en_chn = 0; //do not transmit after sending transmission
    hw->chnconf0[channel_id].carrier_en_chn = 0; // do transform this into signal just turn the bits into raw high or low
    hw->chnconf0[channel_id].div_cnt_chn = 80; // prescaler divide 80Mhz clk
    hw->chnconf0[channel_id].conf_update_chn = 1; //latch the values from virtual register

    hw->chnconf0[channel_id_test].mem_tx_wrap_en_chn = 0;
    hw->chnconf0[channel_id_test].carrier_en_chn = 0;
    hw->chnconf0[channel_id_test].div_cnt_chn = 80;
    hw->chnconf0[channel_id_test].conf_update_chn = 1;

    volatile uint32_t *chan_mem_ptr = (uint32_t *)(DR_REG_RMT_BASE + 0x800 + (channel_id * 48 * 4));
    uint32_t pulse_us = 5;
    uint32_t symbol = (1 << 15) | (pulse_us);
    chan_mem_ptr[0] = symbol; // First data
    chan_mem_ptr[1] = 0;
    chan_mem_ptr[2] = 0;
    chan_mem_ptr[3] = 0;

    volatile uint32_t *chan_mem_ptr_test = (uint32_t *)(DR_REG_RMT_BASE + 0x800 + (channel_id_test * 48 * 4));
    chan_mem_ptr_test[0] = symbol; // First data
    chan_mem_ptr_test[1] = 0;
    chan_mem_ptr_test[2] = 0;
    chan_mem_ptr_test[3] = 0;

    
    
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbf, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    while(1){
        // ESP_LOGI("debug", "segment_queue len: %hu", seg_queue->num_elements);
        
        // ESP_LOGI("step debug", "curr_segment->steps_left%lu", curr_segment->steps_left);
        // ESP_LOGI("debug", "channel id %d rmt pointer %p", channel_id_test, RMT_CH0DATA_REG);
        // debug_rmt_registers(channel_id);
        // esp_rom_delay_us(1000000);
        if (segments_created %10 == 0 || segments_created == total_segments){
            vTaskDelay(1);
            // ESP_LOGI("debug", "segments created: %d", segments_created);
        }
        if (segments_created % 1000 == 0){
            // vTaskDelay(1);
            ESP_LOGI("debug", "segments created: %d", segments_created);
        }
        if(seg_queue->num_elements < BUFFER_SIZE){
            int num_segments = 0;
            if (segments_created < acceleration_segments){
                num_segments = generate_accel_segment(&p, step_buffer);
            }
            else if (segments_created < acceleration_segments + constant_vel_segments){
                num_segments = generate_uniform_segment(&p, step_buffer);
            }
            else if (segments_created < total_segments){
                num_segments = generate_decel_segment(&p, step_buffer);
            }
            
            if(segments_created < total_segments){
                add_segment((struct segment_queue *)seg_queue, step_buffer, num_segments);
                segments_created += num_segments;
            }
            
        }
        
        
    }
    

}