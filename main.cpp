/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "main.h"

using namespace std;

#define BLINKING_RATE     500ms
#define MAXIMUM_BUFFER_SIZE 16

#define STAND 'S' 
#define WALK 'W'
#define RUN 'R'
#define FALL 'F'

#define ACCELEROMETER_RUN_THRESHOLD 500
#define ACCELEROMETER_WALK_THRESHOLD 80
#define ACCELEROMETER_STAND_THRESHOLD 50

#define GYROSCOPE_RUN_THRESHOLD 1500
#define GYROSCOPE_WALK_THRESHOLD 1100
#define GYROSCOPE_STAND_THRESHOLD 150
#define GYROSCOPE_FALL_THRESHOLD 2700

static DigitalOut led(LED1);

int SAMPLING_RATE_MILLISECONDS = 200;
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
    if(SAMPLING_RATE_EXP >= 6){
        SAMPLING_RATE_EXP = 6;
    }else if(SAMPLING_RATE_EXP <= 1) {
        SAMPLING_RATE_EXP = 1;
    }else {}
    SAMPLING_RATE_MILLISECONDS = SAMPLING_RATE_MILLISECONDS + exp(SAMPLING_RATE_EXP);
}

std::array<int16_t, LAST_N_SAMPLES> acc_push(std::array<int16_t, LAST_N_SAMPLES> st, int16_t ni, int st_s){
    for (int i = LAST_N_SAMPLES - 1; i > 0; --i) {
        st[i] = st[i - 1];
    }
    st[0] = ni;
    return st;
}

std::array<float, LAST_N_SAMPLES> gyro_push(std::array<float, LAST_N_SAMPLES> st, int16_t ni, int st_s){
    for (int i = LAST_N_SAMPLES - 1; i > 0; --i) {
        st[i] = st[i - 1];
    }
    st[0] = ni;
    return st;
}

int acc_max(std::array<int16_t, LAST_N_SAMPLES> st){
    int _max_idx = 0;
    for(int i = 1; i < st.size();i++){
        if(st[_max_idx] < st[i]) {
            _max_idx = i;
        }
    }
    return _max_idx;
}

int gyro_max(std::array<float, LAST_N_SAMPLES> st){
    int _max_idx = 0;
    for(int i = 1; i < st.size();i++){
        if(st[_max_idx] < st[i]) {
            _max_idx = i;
        }
    }
    return _max_idx;
}

int8_t AccelConfidence(std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> acc, int16_t* current_acc) {
    char move;
    if((current_acc[0] - acc[0][0] > ACCELEROMETER_RUN_THRESHOLD || current_acc[0] - acc[0][0] < (-1) * ACCELEROMETER_RUN_THRESHOLD) ||
       (current_acc[1] - acc[1][0] > ACCELEROMETER_RUN_THRESHOLD || current_acc[1] - acc[1][0] < (-1)* ACCELEROMETER_RUN_THRESHOLD) ||
       (current_acc[2] - acc[2][0] > ACCELEROMETER_RUN_THRESHOLD || current_acc[2] - acc[2][0] < (-1)* ACCELEROMETER_RUN_THRESHOLD) ) {
        move = RUN;
        live_change_sampling_rate(-0.9);
    } else if ((current_acc[0] - acc[0][0] > ACCELEROMETER_WALK_THRESHOLD || current_acc[0] - acc[0][0] < (-1) * ACCELEROMETER_WALK_THRESHOLD) ||
               (current_acc[1] - acc[1][0] > ACCELEROMETER_WALK_THRESHOLD || current_acc[1] - acc[1][0] < (-1) * ACCELEROMETER_WALK_THRESHOLD) ||
               (current_acc[2] - acc[2][0] > ACCELEROMETER_WALK_THRESHOLD || current_acc[2] - acc[2][0] < (-1) * ACCELEROMETER_WALK_THRESHOLD)) {
        move = WALK;
        live_change_sampling_rate(0.5);
    } else if ((current_acc[0] - acc[0][0] > ACCELEROMETER_STAND_THRESHOLD || current_acc[0] - acc[0][0] < (-1) * ACCELEROMETER_STAND_THRESHOLD) ||
               (current_acc[1] - acc[1][0] > ACCELEROMETER_STAND_THRESHOLD || current_acc[1] - acc[1][0] < (-1) * ACCELEROMETER_STAND_THRESHOLD) ||
               (current_acc[2] - acc[2][0] > ACCELEROMETER_STAND_THRESHOLD || current_acc[2] - acc[2][0] < (-1) * ACCELEROMETER_STAND_THRESHOLD)) {
        move = STAND;
        live_change_sampling_rate(1.1);
    } else {
        move = STAND;
        live_change_sampling_rate(1.1);
    }

    return move;
}

int8_t GyroConfidence(std::array<std::array<float, LAST_N_SAMPLES>, 3> gyro, float* current_gyro) {
    char move;
    int gx_max;
    int gy_max;
    int gz_max;

    if((current_gyro[0] - gyro[0][0] > GYROSCOPE_FALL_THRESHOLD || current_gyro[0] - gyro[0][0] < (-1) * GYROSCOPE_FALL_THRESHOLD) ||
       (current_gyro[1] - gyro[1][0] > GYROSCOPE_FALL_THRESHOLD || current_gyro[1] - gyro[1][0] < (-1) * GYROSCOPE_FALL_THRESHOLD) ||
       (current_gyro[2] - gyro[2][0] > GYROSCOPE_FALL_THRESHOLD || current_gyro[2] - gyro[2][0] < (-1) * GYROSCOPE_FALL_THRESHOLD)){
        gx_max = gyro_max(gyro[0]);
        gy_max = gyro_max(gyro[1]);
        gz_max = gyro_max(gyro[2]);

        if((gx_max > 3 && gx_max < 8) || (gy_max > 3 && gy_max < 8) || (gz_max > 3 && gz_max < 8)) {move = FALL;} else {move = RUN;}
        
        live_change_sampling_rate(-1.2);
    } else if((current_gyro[0] - gyro[0][0] > GYROSCOPE_RUN_THRESHOLD || current_gyro[0] - gyro[0][0] < (-1) * GYROSCOPE_RUN_THRESHOLD) ||
              (current_gyro[1] - gyro[1][0] > GYROSCOPE_RUN_THRESHOLD || current_gyro[1] - gyro[1][0] < (-1)* GYROSCOPE_RUN_THRESHOLD) ||
              (current_gyro[2] - gyro[2][0] > GYROSCOPE_RUN_THRESHOLD || current_gyro[2] - gyro[2][0] < (-1)* GYROSCOPE_RUN_THRESHOLD) ) {
        move = RUN;
        live_change_sampling_rate(-0.9);
    } else if ((current_gyro[0] - gyro[0][0] > GYROSCOPE_WALK_THRESHOLD || current_gyro[0] - gyro[0][0] < (-1) * GYROSCOPE_WALK_THRESHOLD) ||
               (current_gyro[1] - gyro[1][0] > GYROSCOPE_WALK_THRESHOLD || current_gyro[1] - gyro[1][0] < (-1) * GYROSCOPE_WALK_THRESHOLD) ||
               (current_gyro[2] - gyro[2][0] > GYROSCOPE_WALK_THRESHOLD || current_gyro[2] - gyro[2][0] < (-1) * GYROSCOPE_WALK_THRESHOLD)) {
        move = WALK;
        live_change_sampling_rate(0.5);
    } else if ((current_gyro[0] - gyro[0][0] > GYROSCOPE_STAND_THRESHOLD || current_gyro[0] - gyro[0][0] < (-1) * GYROSCOPE_STAND_THRESHOLD) ||
               (current_gyro[1] - gyro[1][0] > GYROSCOPE_STAND_THRESHOLD || current_gyro[1] - gyro[1][0] < (-1) * GYROSCOPE_STAND_THRESHOLD) ||
               (current_gyro[2] - gyro[2][0] > GYROSCOPE_STAND_THRESHOLD || current_gyro[2] - gyro[2][0] < (-1) * GYROSCOPE_STAND_THRESHOLD)) {
        move = STAND;
        live_change_sampling_rate(1.1);
    } else {
        move = STAND;
        live_change_sampling_rate(1.1);
    }

    return move;
}

int main()
{
    HAL_Init();
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();

    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> last_acc_samples;
    std::array<std::array<float, LAST_N_SAMPLES>, 3> last_gyro_samples;

    // DigitalOut led(LED1);
    // DigitalOut led_comfort(LED2);

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

    // float prev_gyro_value[3];
    // int16_t prev_accel_value[3];

    uint32_t _start = serial_port.read(trigger, sizeof(trigger));
    char amove;
    char gmove;
    char fmove;
    
    while (1) {
        
        BSP_ACCELERO_AccGetXYZ(accel_value);
        BSP_GYRO_GetXYZ(gyro_value);

        if (_started == 0) {
            last_acc_samples[0] = acc_push(last_acc_samples[0], accel_value[0]);
            last_acc_samples[1] = acc_push(last_acc_samples[1], accel_value[1]);
            last_acc_samples[2] = acc_push(last_acc_samples[2], accel_value[2]);

            last_gyro_samples[0] = gyro_push(last_gyro_samples[0], gyro_value[0]);
            last_gyro_samples[1] = gyro_push(last_gyro_samples[1], gyro_value[1]);
            last_gyro_samples[2] = gyro_push(last_gyro_samples[2], gyro_value[2]);

            _started = 1;
            continue;
        }

        amove = AccelConfidence(last_acc_samples, accel_value);
        gmove = GyroConfidence(last_gyro_samples, gyro_value);

        if          (amove == gmove)                    {fmove = amove;}
        else if     (amove == RUN && gmove == FALL)     {fmove = FALL;}
        else if     (amove == RUN && gmove == WALK)     {fmove = RUN;}
        else if     (amove == WALK && gmove == RUN)     {fmove = WALK;}
        else if     (amove == STAND)                    {fmove = STAND;}
        else                                            {fmove = STAND;}

        buf[0] = fmove;

        last_acc_samples[0] = acc_push(last_acc_samples[0], accel_value[0]);
        last_acc_samples[1] = acc_push(last_acc_samples[1], accel_value[1]);
        last_acc_samples[2] = acc_push(last_acc_samples[2], accel_value[2]);

        last_gyro_samples[0] = gyro_push(last_gyro_samples[0], gyro_value[0]);
        last_gyro_samples[1] = gyro_push(last_gyro_samples[1], gyro_value[1]);
        last_gyro_samples[2] = gyro_push(last_gyro_samples[2], gyro_value[2]);

        HAL_Delay(SAMPLING_RATE_MILLISECONDS);
        led = !led;

        serial_port.write(buf, sizeof(buf));

        // HAL_Delay(500);

        // uint8_t ciphertext[16];
        // AES_ECB_encrypt(buf , key, ciphertext, sizeof(buf));

        
    }
}
