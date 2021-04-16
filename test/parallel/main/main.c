#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "parallel.h"

/*   parallel interface configure part  */

#define  CLK  GPIO_NUM_34

#define  D0  GPIO_NUM_35
#define  D1  GPIO_NUM_36
#define  D2  GPIO_NUM_37
#define  D3  GPIO_NUM_38
#define  D4  GPIO_NUM_39
#define  D5  GPIO_NUM_40
#define  D6  GPIO_NUM_41
#define  D7  GPIO_NUM_42

#define  D8   GPIO_NUM_21
#define  D9   GPIO_NUM_18
#define  D10  GPIO_NUM_17
#define  D11  GPIO_NUM_16
#define  D12  GPIO_NUM_15
#define  D13  GPIO_NUM_14
#define  D14  GPIO_NUM_13
#define  D15  GPIO_NUM_12

typedef struct {
    uint8_t bits_7_0;
    uint8_t bits_15_8; 
} parallel_data_t;

parallel_data_t data[4096];

void app_main(void)
{
    printf("parallel!\n");

    for (uint32_t x = 0; x < 4096; x++) {
        data[x].bits_15_8 =   x;
        data[x].bits_7_0 =    x;
    }

    parallel_config_t parallel_config = {
        .clk_div = 2, // clk_fre = 40MHz / clk_div
        .bit_width = 16,
        .pin_clk = CLK,
        .clk_polarity = 0,
        .dat_polarity = 0,
        .pin_data = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15},
    };

    parallel_init(&parallel_config);

    while (1) {
        if (is_parallel_write_idle()) {
            parallel_write_data((uint8_t *)data, sizeof(parallel_data_t) * 4096);
        }
        
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}
