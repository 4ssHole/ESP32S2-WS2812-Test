/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_LEDCONTROLLER
#define H_LEDCONTROLLER

#include "driver/rmt.h"
#include "led_strip.h"
#include "esp_log.h"
#include "NVSHelper.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0

static const char *TAG = "LED Controller";
static led_strip_t *strip;

#ifdef __cplusplus
extern "C" {
#endif

void initLEDController();

void setAllLED(uint8_t setRed, uint8_t setGreen, uint8_t setBlue);

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);

uint8_t getRed();
uint8_t getGreen();
uint8_t getBlue();

#ifdef __cplusplus
}
#endif

#endif
