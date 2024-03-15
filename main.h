
#include "mbed.h"
#include "aes.h"
#include "stm32l475e_iot01.h"
#include <stdio.h>

#include <string> 
#include <cstring> 
#include <array>

#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"

#define LAST_N_SAMPLES 10

int8_t AccelConfidence(std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> acc, int16_t* current_acc);
int8_t GyroConfidence(std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> gyro, int16_t* current_gyro);

std::array<int16_t, LAST_N_SAMPLES> acc_push(std::array<int16_t, LAST_N_SAMPLES> st, int16_t ni, int st_s=LAST_N_SAMPLES);
std::array<float, LAST_N_SAMPLES> gyro_push(std::array<float, LAST_N_SAMPLES> st, int16_t ni, int st_s=LAST_N_SAMPLES);

int acc_max(std::array<int16_t, LAST_N_SAMPLES> st);
int gyro_max(std::array<float, LAST_N_SAMPLES> st);
