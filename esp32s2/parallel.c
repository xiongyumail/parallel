// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp32s2/rom/lldesc.h"
#include "hal/gpio_ll.h"
#include "driver/ledc.h"
#include "system_reg.h"
#include "i2s_struct.h"
#include "parallel.h"

static parallel_config_t* p_parallel_config;
static lldesc_t __dma[DMA_DESC_MAX_CNT] = {0};
static uint8_t *parallel_buffer = NULL;
static volatile int busy_flag = 0;

static void parallel_config(void)
{
    periph_module_enable(PERIPH_I2S0_MODULE);

    //Configure pclk, max: 20M
    I2S0.clkm_conf.val = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a =63;
    I2S0.clkm_conf.clk_en = 1;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.sample_rate_conf.val = 0;
    I2S0.sample_rate_conf.tx_bck_div_num = p_parallel_config->clk_div;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_msb_right = 1;
    I2S0.conf.tx_dma_equal = 1;

    I2S0.conf1.val = 0;
    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.timing.val = 0;
    //Set lcd mode
    I2S0.conf2.val = 0;
    I2S0.conf2.lcd_en = 1;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.tx_fifo_mod = 4;

    I2S0.conf_chan.tx_chan_mod = 0;//remove

    I2S0.sample_rate_conf.tx_bits_mod = p_parallel_config->bit_width;
}

static void parallel_set_pin(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_parallel_config->pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(p_parallel_config->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(p_parallel_config->pin_clk,GPIO_PULLUP_ONLY);
    gpio_matrix_out(p_parallel_config->pin_clk, I2S0O_WS_OUT_IDX, !p_parallel_config->clk_polarity, 0);

    for(int i = 0; i < p_parallel_config->bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_parallel_config->pin_data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(p_parallel_config->pin_data[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(p_parallel_config->pin_data[i], GPIO_PULLUP_ONLY);
        gpio_matrix_out(p_parallel_config->pin_data[i], I2S0O_DATA_OUT0_IDX + 24 - p_parallel_config->bit_width + i, !p_parallel_config->dat_polarity, 0);
    }
}


static void IRAM_ATTR parallel_isr(void *arg)
{
   if(I2S0.int_st.out_total_eof) {
       busy_flag = 0;
   }
   I2S0.int_clr.val = ~0;
}

static void parallel_interface_init()
{
    for(int i = 0; i < DMA_DESC_MAX_CNT; i++) {
        __dma[i].size = 0;
        __dma[i].length = 0;
        __dma[i].sosf = 0;
        __dma[i].eof = 1;
        __dma[i].owner = 1;
        __dma[i].buf = NULL;
        __dma[i].empty = NULL;
    }
    parallel_set_pin();
    parallel_config();
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, parallel_isr, NULL, NULL);
    I2S0.int_ena.out_total_eof = 1;
}

static inline void IRAM_ATTR dma_start(uint32_t addr)
{
    busy_flag = 1;
    while (!I2S0.state.tx_idle); // Lower data frequency will cause the dma interrupt to arrive early, and the line has not yet been sent
    I2S0.conf.tx_start = 0;
    I2S0.fifo_conf.dscr_en = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.conf.tx_fifo_reset = 1;
    I2S0.conf.tx_fifo_reset = 0;
	I2S0.fifo_conf.dscr_en = 1;
    I2S0.out_link.addr = addr;
	I2S0.out_link.start = 1;
	esp_rom_delay_us(1);
	I2S0.conf.tx_start = 1;
}

static inline void IRAM_ATTR parallel_dma_write(uint8_t *buf, size_t length)
{
    int len = length;
    int trans_len = 0;
    int cnt = 0;
    while(len) {
        trans_len = len > 4000 ? 4000 : len;
        __dma[cnt].size = trans_len;
        __dma[cnt].length = trans_len;
        __dma[cnt].buf = buf;
        __dma[cnt].eof = 0;
        __dma[cnt].empty = &__dma[cnt+1];
        buf += trans_len;
        len -= trans_len;
        cnt++;
        if(cnt >= DMA_DESC_MAX_CNT)
            break;
    }
    __dma[cnt-1].eof = 1;
    __dma[cnt-1].empty = NULL;
    dma_start(((uint32_t)&__dma[0]) & 0xfffff);
}

void IRAM_ATTR parallel_write_data(uint8_t *data, size_t len)
{
#ifdef PARALLEL_BURST_BUFFER_SIZE
    int x = 0;
    for (x = 0; x < len / PARALLEL_BURST_BUFFER_SIZE; x++) {
        memcpy(parallel_buffer, data, PARALLEL_BURST_BUFFER_SIZE);
        parallel_dma_write(parallel_buffer, PARALLEL_BURST_BUFFER_SIZE);
        data += PARALLEL_BURST_BUFFER_SIZE;
    }
    if (len % PARALLEL_BURST_BUFFER_SIZE) {
        memcpy(parallel_buffer, data, len % PARALLEL_BURST_BUFFER_SIZE);
        parallel_dma_write(parallel_buffer, len % PARALLEL_BURST_BUFFER_SIZE);
    }
#else
    parallel_dma_write((uint8_t *)data, len);
#endif
}

uint8_t IRAM_ATTR is_parallel_write_idle(void)
{
    if(I2S0.conf.tx_start == 0 || busy_flag == 0)
        return 1;
    else
        return 0;
}

void parallel_init(parallel_config_t *parallel_config)
{
    if (parallel_config == NULL) {
        return;
    }
    p_parallel_config = parallel_config;

    parallel_interface_init();
}
