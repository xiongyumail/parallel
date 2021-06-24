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
#include "esp32s3/rom/lldesc.h"
#include "esp32s3/rom/gpio.h"
#include "driver/gpio.h"
#include "gdma_struct.h"
#include "interrupt_core0_reg.h"
#include "system_reg.h"
#include "lcd_cam_struct.h"
#include "parallel.h"

static parallel_config_t* p_parallel_config;
static lldesc_t __dma[DMA_DESC_MAX_CNT] = {0};
static uint8_t *parallel_buffer = NULL;
static volatile int busy_flag = 0;

#define LCD_CAM_INTR_SOURCE               (((INTERRUPT_CORE0_LCD_CAM_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))
#define LCD_CAM_DMA_NUM                   (4)

static void parallel_config(void)
{
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf1.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].in.conf0.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].in.conf1.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.int_clr.val = ~0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.int_ena.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].in.int_clr.val = ~0;
    GDMA.channel[LCD_CAM_DMA_NUM].in.int_ena.val = 0;

    LCD_CAM.lcd_clock.val = 0;
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 10;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = 2;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = p_parallel_config->clk_div * 2 - 1;
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_user.val = 0;
    LCD_CAM.lcd_user.lcd_2byte_en = (p_parallel_config->bit_width == 16) ? 1 : 0;
    LCD_CAM.lcd_user.lcd_byte_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;
    LCD_CAM.lcd_user.lcd_cmd = 0;		// FSM CMD phase
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;	// 2 cycle command
    LCD_CAM.lcd_user.lcd_dout = 1;	// FSM DOUT phase
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 2 - 1;
    LCD_CAM.lcd_user.lcd_8bits_order = 1;
    LCD_CAM.lcd_user.lcd_always_out_en = 1;
    LCD_CAM.lcd_misc.val = 0;
    LCD_CAM.lcd_misc.lcd_afifo_threshold_num = 11;
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 3;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_cd_idle_edge = 1;	// idle edge of CD is set to 0
    LCD_CAM.lcd_misc.lcd_cd_cmd_set = 1;
    LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_data_set = 0;	// change when DOUT start
    LCD_CAM.lcd_misc.lcd_bk_en = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    LCD_CAM.lcd_ctrl.val = 0;
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
    LCD_CAM.lcd_cmd_val = 0;	// write command
    LCD_CAM.lcd_user.lcd_update = 1;

    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_rst = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_rst = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_data_burst_en = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.peri_sel.sel = 5;
    GDMA.channel[LCD_CAM_DMA_NUM].out.pri.tx_pri = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.int_ena.out_eof = 1;
}

static void parallel_set_pin(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_parallel_config->pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(p_parallel_config->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(p_parallel_config->pin_clk, GPIO_FLOATING);
    gpio_matrix_out(p_parallel_config->pin_clk, LCD_PCLK_IDX, p_parallel_config->clk_polarity, false);

    for(int i = 0; i < p_parallel_config->bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[p_parallel_config->pin_data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(p_parallel_config->pin_data[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(p_parallel_config->pin_data[i], GPIO_FLOATING);
        gpio_matrix_out(p_parallel_config->pin_data[i], LCD_DATA_OUT0_IDX + i, p_parallel_config->dat_polarity, false);
    }
}

static void IRAM_ATTR parallel_isr(void *arg)
{
   if(LCD_CAM.lc_dma_int_st.lcd_trans_done) {
       busy_flag = 0;
   }
   LCD_CAM.lc_dma_int_clr.val = ~0;
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
    esp_intr_alloc(LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, parallel_isr, NULL, NULL);
    LCD_CAM.lc_dma_int_ena.lcd_trans_done = 1;
}

static inline void IRAM_ATTR dma_start(uint32_t addr)
{
    busy_flag = 1;
    while (LCD_CAM.lcd_user.lcd_start);
    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_user.lcd_reset = 0;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    while (GDMA.channel[LCD_CAM_DMA_NUM].out.link.start);
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf1.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.int_clr.val = ~0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.int_ena.val = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_rst = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_rst = 0;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.conf0.out_data_burst_en = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.peri_sel.sel = 5;
    GDMA.channel[LCD_CAM_DMA_NUM].out.pri.tx_pri = 1;
    GDMA.channel[LCD_CAM_DMA_NUM].out.link.addr = addr;
    GDMA.channel[LCD_CAM_DMA_NUM].out.link.start = 1;
    esp_rom_delay_us(1);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
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
    dma_start((uint32_t)&__dma[0]);
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
    if(LCD_CAM.lcd_user.lcd_start == 0 || busy_flag == 0)
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
