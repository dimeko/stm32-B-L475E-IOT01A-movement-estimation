/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"

#include <stdio.h>
#include <string> 
#include <array>

#include <cstring>
#include <cmath> 

#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"

#define LAST_N_SAMPLES 10

template<typename T>  std::array<T, LAST_N_SAMPLES> ppush(std::array<T, LAST_N_SAMPLES> arr, T new_item);
template<typename T> int mmax(std::array<T, LAST_N_SAMPLES> arr);
template<typename T> int mmin(std::array<T, LAST_N_SAMPLES> arr);

std::array<uint8_t, 2> AccelConfidence(
    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> old_values,
    int16_t* current,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res);

std::array<uint8_t, 2> GyroConfidence(
    std::array<std::array<float, LAST_N_SAMPLES>, 3> old_values,
    float* current,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res);

std::array<uint8_t, 2> MovementRecognition(
    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> old_acc_values,
    std::array<std::array<float, LAST_N_SAMPLES>, 3> old_gyro_values,
    int16_t* current_acc,
    float* current_gyro,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res);