/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "main.h"
#include <string> 
#include <string.h> 
#include <cstring> 
#include <stack>
#include <cmath>

#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"

using namespace std;

#define BLINKING_RATE     500ms
#define MAXIMUM_BUFFER_SIZE 16

#define STAND 'S' 
#define WALK 'W'
#define RUN 'R'
#define FALL 'F'

static DigitalOut led(LED1);

int SAMPLING_RATE_MILLISECONDS = 300;
float SAMPLING_RATE_EXP = 1;

static BufferedSerial serial_port(USBTX, USBRX);

uint8_t is_in_comfort_zone(float x, float y) {
  if(x < 0 || y < 0) {
    return 0;
  }
  float xy1a[2] = {21.4, 100.0};
  float xy1b[2] = {25.5, 0};

  float xy2a[2] = {26.2, 100.0};
  float xy2b[2] = {32, 0};

  float a1 = (xy1b[1] - xy1a[1]) / (xy1b[0] - xy1a[0]);
  float a2 = (xy2b[1] - xy2a[1]) / (xy2b[0] - xy2a[0]);

  float b1 = xy1a[1] - a1 * xy1a[0];
  float b2 = xy2a[1] - a2 * xy2a[0];

  float d1 = y - x*a1 - b1;
  float d2 = y - x*a2 - b2;
  return (d1 >= 0 && d2 <= 0) ? 1 : 0 ;
} 

void live_change_sampling_rate(float add_exp){
    // SAMPLING_RATE_MILLISECONDS = 300;
    SAMPLING_RATE_EXP = SAMPLING_RATE_EXP + add_exp;
    if(SAMPLING_RATE_EXP >= 8){
        SAMPLING_RATE_EXP = 8;
    }else if(SAMPLING_RATE_EXP <= 1) {
        SAMPLING_RATE_EXP = 1;
    }else {}
    SAMPLING_RATE_MILLISECONDS = 300 + exp(SAMPLING_RATE_EXP);
}

int main()
{
    HAL_Init();
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();

    DigitalOut led(LED1);
    DigitalOut led_comfort(LED2);

    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1);

    char buf[MAXIMUM_BUFFER_SIZE] = {0};
    uint8_t trigger[MAXIMUM_BUFFER_SIZE] = {0};

    uint8_t key[] = {0x5c, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    int8_t _started = 0;

    float gyro_value[3];
    int16_t accel_value[3];

    float prev_gyro_value[3];
    int16_t prev_accel_value[3];

    uint32_t _start = serial_port.read(trigger, sizeof(trigger));

    while (1) {
        
        BSP_ACCELERO_AccGetXYZ(accel_value);
        BSP_GYRO_GetXYZ(gyro_value);

        if (_started == 0) {
            prev_accel_value[0] = accel_value[0];
            prev_accel_value[1] = accel_value[1];
            prev_accel_value[2] = accel_value[2];

            prev_gyro_value[0] = gyro_value[0];
            prev_gyro_value[1] = gyro_value[1];
            prev_gyro_value[2] = gyro_value[2];
            _started = 1;
            continue;
        }

        if(accel_value[0] - prev_accel_value[0] > 500 || accel_value[0] - prev_accel_value[0] < -500 ) {
            buf[0] = RUN;
            live_change_sampling_rate(-0.7);
        } else if (accel_value[0] - prev_accel_value[0] > 100 || accel_value[0] - prev_accel_value[0] < -100) {
            buf[0] = WALK;
            live_change_sampling_rate(0.3);
        } else if (accel_value[0] - prev_accel_value[0] > 50 || accel_value[0] - prev_accel_value[0] < -50) {
            buf[0] = STAND;
            live_change_sampling_rate(1.1);
        } else {
            buf[0] = STAND;
            live_change_sampling_rate(1.1);
        }

        HAL_Delay(SAMPLING_RATE_MILLISECONDS);
        led = !led;

        serial_port.write(buf, sizeof(buf));

        // HAL_Delay(500);

        // uint8_t ciphertext[16];
        // AES_ECB_encrypt(buf , key, ciphertext, sizeof(buf));

        
    }
}

