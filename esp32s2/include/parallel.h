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

#pragma once

#include "esp_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARALLEL_MAX_BUFFER_SIZE        (64*1024)
#define DMA_DESC_MAX_CNT                (PARALLEL_MAX_BUFFER_SIZE / 4000)

//#define PARALLEL_BURST_BUFFER_SIZE (16 * 1024)

#define PARALLEL_PIN_CNT    24

typedef struct {
    uint8_t clk_div;            //ck周期频率 clk_fre = 40MHz / clk_div
    uint8_t pin_clk;            //ck端口
    uint8_t clk_polarity;       //ck极性
    uint8_t dat_polarity;       //数据极性
    int8_t pin_data[PARALLEL_PIN_CNT];          //数据端口
    uint8_t bit_width;          //数据宽度8、16、24
}parallel_config_t;

void parallel_write_data(uint8_t *data, size_t len);

uint8_t is_parallel_write_idle(void);

void parallel_init(parallel_config_t *parallel_config); // clk_fre = 40MHz / clk_div, clk_div > 1

#ifdef __cplusplus
}
#endif