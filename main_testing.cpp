
#include "mbed.h"

#include <stdio.h>
#include <string> 
#include <array>

#include <cstring>
#include <cmath> 

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

    std:string resultant;
    float gyro_value[3];
    int16_t accel_value[3];
    while (1) {
        
        BSP_ACCELERO_AccGetXYZ(accel_value);
        resultant=to_string(accel_value[0]);

        // Smaller buffer for string
        buf[0] = resultant.c_str()[0];
        buf[1] = resultant.c_str()[1];
        buf[2] = resultant.c_str()[2];
        buf[3] = resultant.c_str()[3];
        buf[4] = resultant.c_str()[4];
        buf[5] = resultant.c_str()[5];
        buf[6] = resultant.c_str()[6];
        buf[7] = resultant.c_str()[7];

        resultant=to_string(accel_value[1]);

        buf[8] = resultant.c_str()[0];
        buf[9] = resultant.c_str()[1];
        buf[10] = resultant.c_str()[2];
        buf[11] = resultant.c_str()[3];
        buf[12] = resultant.c_str()[4];
        buf[13] = resultant.c_str()[5];
        buf[14] = resultant.c_str()[6];
        buf[15] = resultant.c_str()[7];

        resultant=to_string(accel_value[2]);

        buf[16] = resultant.c_str()[0];
        buf[17] = resultant.c_str()[1];
        buf[18] = resultant.c_str()[2];
        buf[19] = resultant.c_str()[3];
        buf[20] = resultant.c_str()[4];
        buf[21] = resultant.c_str()[5];
        buf[22] = resultant.c_str()[6];
        buf[23] = resultant.c_str()[7];

        if (uint32_t num = serial_port.read(trigger, sizeof(trigger))) {
            led = !led;

            serial_port.write(buf, sizeof(buf));
        }

        
        BSP_GYRO_GetXYZ(gyro_value);
        resultant=to_string(gyro_value[0]);

        // Smaller buffer for string
        buf[0] = resultant.c_str()[0];
        buf[1] = resultant.c_str()[1];
        buf[2] = resultant.c_str()[2];
        buf[3] = resultant.c_str()[3];
        buf[4] = resultant.c_str()[4];
        buf[5] = resultant.c_str()[5];
        buf[6] = resultant.c_str()[6];
        buf[7] = resultant.c_str()[7];
        buf[8] = resultant.c_str()[8];
        buf[9] = resultant.c_str()[9];

        resultant=to_string(gyro_value[1]);
        buf[10] = resultant.c_str()[0];
        buf[11] = resultant.c_str()[1];
        buf[12] = resultant.c_str()[2];
        buf[13] = resultant.c_str()[3];
        buf[14] = resultant.c_str()[4];
        buf[15] = resultant.c_str()[5];
        buf[16] = resultant.c_str()[6];
        buf[17] = resultant.c_str()[7];
        buf[18] = resultant.c_str()[8];
        buf[19] = resultant.c_str()[9];

        resultant=to_string(gyro_value[2]);
        buf[21] = resultant.c_str()[0];
        buf[22] = resultant.c_str()[1];
        buf[23] = resultant.c_str()[2];
        buf[24] = resultant.c_str()[3];
        buf[25] = resultant.c_str()[4];
        buf[26] = resultant.c_str()[5];
        buf[27] = resultant.c_str()[6];
        buf[28] = resultant.c_str()[7];
        buf[29] = resultant.c_str()[8];
        buf[30] = resultant.c_str()[9];

        if (uint32_t num = serial_port.read(trigger, sizeof(trigger))) {
            led = !led;

            serial_port.write(buf, sizeof(buf));
        }        
    }
}