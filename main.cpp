#include "main.h"
#include "aes.h"

using namespace std;

#define MAXIMUM_BUFFER_SIZE 16

// Letters S,W,R,F in ASCII
#define STAND 83
#define WALK 87
#define RUN 82
#define FALL 70

// Sensors thresholds
#define ACCELEROMETER_RUN_THRESHOLD 600
#define ACCELEROMETER_WALK_THRESHOLD 550
#define ACCELEROMETER_SLOW_WALK_THRESHOLD 100
#define ACCELEROMETER_FALL_THRESHOLD 600

#define GYROSCOPE_RUN_THRESHOLD 5500
#define GYROSCOPE_WALK_THRESHOLD 4100
#define GYROSCOPE_SLOW_WALK_THRESHOLD 3050
#define GYROSCOPE_FALL_THRESHOLD 2700

static DigitalOut led(LED1);

int16_t SAMPLING_RATE_MILLISECONDS = 200;
int16_t LEAST_SAMPLING_RATE_MILLISECONDS = 200;
float SAMPLING_RATE_EXP = 2;
int16_t SAMPLES_ON_THE_SAME_MOVE = 0;

static BufferedSerial serial_port(USBTX, USBRX);

uint8_t is_in_comfort_zone(float x, float y) {
    /*
        Values taken from the CBE Thermal Comfort Tool
        https://comfort.cbe.berkeley.edu/ (Figure relative humidity vs air temperature)
    */
    if(x < 0 || y < 0) {
        return 0;
    }
    // Don't change the numbers below
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
    /*
        Changing sampling rate exponentialy 
        with a lowest value LEAST_SAMPLING_RATE_MILLISECONDS + e^1
        and max LEAST_SAMPLING_RATE_MILLISECONDS + e^6
    */
    SAMPLING_RATE_EXP = SAMPLING_RATE_EXP + add_exp;
    if(SAMPLING_RATE_EXP >= 7){
        SAMPLING_RATE_EXP = 6;
    } else if(SAMPLING_RATE_EXP <= 1) {
        SAMPLING_RATE_EXP = 1;
    } else {}
    SAMPLING_RATE_MILLISECONDS = LEAST_SAMPLING_RATE_MILLISECONDS + exp(SAMPLING_RATE_EXP);
}

template<typename T> std::array<T, LAST_N_SAMPLES> ppush(std::array<T, LAST_N_SAMPLES> arr, T new_item){
    for (int i = LAST_N_SAMPLES - 1; i > 0; --i) {
        arr[i] = arr[i - 1];
    }
    arr[0] = new_item;
    return arr;
}

template<typename T> int mmax(std::array<T, LAST_N_SAMPLES> arr){
    int _max_idx = 0;
    for(int i = 1; i < arr.size();i++){
        if(arr[_max_idx] < arr[i]) {
            _max_idx = i;
        }
    }
    return _max_idx;
}

template<typename T> int mmin(std::array<T, LAST_N_SAMPLES> arr){
    int _min_idx = 0;
    for(int i = 1; i < arr.size();i++){
        if(arr[_min_idx] > arr[i]) {
            _min_idx = i;
        }
    }
    return _min_idx;
}

std::array<uint8_t, 2> AccelConfidence(
    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> old_values,
    int16_t* current,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res) {

    int8_t mscore = 0;
    // Finding the max values of the accelrometer array
    int16_t _max_x = mmax(old_values[0]); int16_t _max_y = mmax(old_values[1]); int16_t _max_z = mmax(old_values[2]);
    int16_t _min_x = mmin(old_values[0]); int16_t _min_y = mmin(old_values[1]); int16_t _min_z = mmin(old_values[2]);

    std::array<uint8_t, 2>  moves;

    
     if(abs(current[0] - old_values[0][0]) > ACCELEROMETER_RUN_THRESHOLD ||
       abs(current[1] - old_values[1][0]) > ACCELEROMETER_RUN_THRESHOLD ||
       abs(current[2] - old_values[2][0]) > ACCELEROMETER_RUN_THRESHOLD) {
        if(mov_conf[0] == STAND) {moves[0] = STAND; moves[1] = RUN;}
        else if(mov_conf[0] == WALK) { moves[0] = WALK; moves[1] = RUN;}
        else { moves[0] = RUN; moves[1] = RUN;}

        live_change_sampling_rate(-0.5);
    } else if (abs(_max_z - _min_z) > ACCELEROMETER_FALL_THRESHOLD && abs((_max_z + _min_z) / 2 - current[2]) > ACCELEROMETER_FALL_THRESHOLD) {
        moves[0] = FALL;
        moves[1] = mov_conf[0];

        live_change_sampling_rate(-0.9);
    }
     else if (
        abs(current[0] - old_values[0][0]) > ACCELEROMETER_WALK_THRESHOLD ||
        abs(current[1] - old_values[1][0]) > ACCELEROMETER_WALK_THRESHOLD ||
        abs(current[2] - old_values[2][0]) > ACCELEROMETER_WALK_THRESHOLD ) {

        if(mov_conf[0] == WALK) {moves[0] = WALK; moves[1] = WALK;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = WALK;}
        else if(mov_conf[0] == STAND) { moves[0] = STAND; moves[1] = WALK;}
        else { mov_conf[0] = WALK; moves[1] = WALK;}

        live_change_sampling_rate(-0.4);
    } else if (
        abs(current[0] - old_values[0][0]) > ACCELEROMETER_SLOW_WALK_THRESHOLD ||
        abs(current[1] - old_values[1][0]) > ACCELEROMETER_SLOW_WALK_THRESHOLD ||
        abs(current[2] - old_values[2][0]) > ACCELEROMETER_SLOW_WALK_THRESHOLD) {

        if(mov_conf[0] == WALK) {moves[0] = WALK; moves[1] = WALK;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = WALK;}
        else if(mov_conf[0] == STAND) { moves[0] = STAND; moves[1] = WALK;}
        else { moves[0] = WALK; moves[1] = WALK;}
        
        live_change_sampling_rate(0.3);
    } else {

        if(mov_conf[0] == STAND) {moves[0] = STAND; moves[1] = STAND;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = STAND;}
        else if(mov_conf[0] == WALK) { moves[0] = WALK; moves[1] = STAND;}
        else { moves[0] = STAND; moves[1] = STAND;}
        
        live_change_sampling_rate(0.5);
    }


    if(mov_res[0] == FALL && moves[0] == STAND){
        moves[0] = FALL;
    }

    return moves;
}

std::array<uint8_t, 2> GyroConfidence(
    std::array<std::array<float, LAST_N_SAMPLES>, 3> old_values,
    float* current,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res) {

    // Finding the max values of the accelrometer array
    int16_t _max_x = mmax(old_values[0]); int16_t _max_y = mmax(old_values[1]); int16_t _max_z = mmax(old_values[2]);
    int16_t _min_x = mmin(old_values[0]); int16_t _min_y = mmin(old_values[1]); int16_t _min_z = mmin(old_values[2]);

    std::array<uint8_t, 2>  moves;
    if(abs(current[0] - old_values[0][0]) > GYROSCOPE_RUN_THRESHOLD ||
       abs(current[1] - old_values[1][0]) > GYROSCOPE_RUN_THRESHOLD ||
       abs(current[2] - old_values[2][0]) > GYROSCOPE_RUN_THRESHOLD) {
    
        if(mov_conf[0] == STAND) {moves[0] = STAND; moves[1] = RUN;}
        else if(mov_conf[0] == WALK) { moves[0] = WALK; moves[1] = RUN;}
        else { moves[0] = RUN; moves[1] = RUN;}

        live_change_sampling_rate(-0.5);
    } else if (
        abs(_max_y + _min_y / 2.0 - current[1]) > GYROSCOPE_FALL_THRESHOLD ||
        abs(_max_x + _min_x / 2.0 - current[0]) > GYROSCOPE_FALL_THRESHOLD ||
        abs(_max_z + _min_z / 2.0 - current[2]) > GYROSCOPE_FALL_THRESHOLD) {

        moves[0] = FALL;
        moves[1] = mov_conf[0];

        live_change_sampling_rate(-0.9);
    } else if (
        abs(current[0] - old_values[0][0]) > GYROSCOPE_WALK_THRESHOLD ||
        abs(current[1] - old_values[1][0]) > GYROSCOPE_WALK_THRESHOLD ||
        abs(current[2] - old_values[2][0]) > GYROSCOPE_WALK_THRESHOLD ) {

        if(mov_conf[0] == WALK) {moves[0] = WALK; moves[1] = WALK;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = WALK;}
        else if(mov_conf[0] == STAND) { moves[0] = STAND; moves[1] = WALK;}
        else { moves[0] = WALK; moves[1] = WALK;}

        live_change_sampling_rate(-0.4);
    } else if (
        abs(current[0] - old_values[0][0]) > GYROSCOPE_SLOW_WALK_THRESHOLD ||
        abs(current[1] - old_values[1][0]) > GYROSCOPE_SLOW_WALK_THRESHOLD ||
        abs(current[2] - old_values[2][0]) > GYROSCOPE_SLOW_WALK_THRESHOLD) {

        if(mov_conf[0] == WALK) {moves[0] = WALK; moves[1] = WALK;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = WALK;}
        else if(mov_conf[0] == STAND) { moves[0] = STAND; moves[1] = WALK;}
        else { moves[0] = WALK; moves[1] = WALK;}
        
        live_change_sampling_rate(0.3);
    } else {
        if(mov_conf[0] == STAND) {moves[0] = STAND; moves[1] = STAND;}
        else if(mov_conf[0] == RUN) { moves[0] = RUN; moves[1] = STAND;}
        else if(mov_conf[0] == WALK) { moves[0] = WALK; moves[1] = STAND;}
        else { moves[0] = STAND; moves[1] = STAND;}
        
        live_change_sampling_rate(0.5);
    }

    if(mov_res[0] == FALL && moves[0] == STAND){
        moves[0] = FALL;
    }

    return moves;
}

std::array<uint8_t, 2> MovementRecognition(
    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> old_acc_values,
    std::array<std::array<float, LAST_N_SAMPLES>, 3> old_gyro_values,
    int16_t* current_acc,
    float* current_gyro,
    std::array<int16_t, LAST_N_SAMPLES> samp_rate,
    std::array<uint8_t, LAST_N_SAMPLES> mov_conf,
    std::array<uint8_t, LAST_N_SAMPLES> mov_res
){
    std::array<uint8_t, 2> amove = AccelConfidence(old_acc_values, current_acc, samp_rate, mov_conf, mov_res);
    std::array<uint8_t, 2> gmove = GyroConfidence(old_gyro_values, current_gyro, samp_rate, mov_conf, mov_res);
    std::array<uint8_t, 2> fmove;

    if (amove[0] == gmove[0])                       {fmove[0] = amove[0];}
    else if(amove[0] == FALL || gmove[0] == FALL)   {fmove[0] = FALL;}
    else if(amove[0] != gmove[0])                   {fmove[0] = amove[0];}
    else if(amove[0] == STAND)                      {fmove[0] = STAND;}
    else                                            {fmove[0] = STAND;}

    return fmove;
}

int main()
{
    // Sensor initialization
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();

    std::array<std::array<int16_t, LAST_N_SAMPLES>, 3> last_acc_samples; // Last LAST_N_SAMPLES samples of accelerometer
    std::array<std::array<float, LAST_N_SAMPLES>, 3> last_gyro_samples; // Last LAST_N_SAMPLES samples of gyroscope
    std::array<uint8_t, LAST_N_SAMPLES> last_movement_results; // Last LAST_N_SAMPLES movement results
    std::array<uint8_t, LAST_N_SAMPLES> last_movement_confidence; // Last LAST_N_SAMPLES samples of gyroscope
    std::array<int16_t, LAST_N_SAMPLES> last_sampling_rates; // Last LAST_N_SAMPLES samples of gyroscope

    // Initializing the last movement conidence with a random possible state
    last_movement_confidence[0] = STAND;

    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1);

    uint8_t buf[MAXIMUM_BUFFER_SIZE] = {0};
    uint8_t trigger[MAXIMUM_BUFFER_SIZE] = {0};

    // The cryptographic key
    uint8_t key[] = {0x5c, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    int8_t _started = 0;

    // The accelerometer and gyroscope current values
    float gyro_value[3];
    int16_t accel_value[3];

    uint32_t _start = serial_port.read(trigger, sizeof(trigger));

    std::array<uint8_t, 2>  move;

    float sensor_t_value = 0;
    float sensor_h_value = 0;

    uint8_t temp_integer_part;
    uint8_t temp_decimal_part;

    uint8_t hum_integer_part;
    uint8_t hum_decimal_part;

    while (1) {
        
        BSP_ACCELERO_AccGetXYZ(accel_value);
        BSP_GYRO_GetXYZ(gyro_value);
        sensor_t_value = BSP_TSENSOR_ReadTemp();
        sensor_h_value = BSP_HSENSOR_ReadHumidity();

        temp_integer_part = (int) sensor_t_value;
        temp_decimal_part = (int)(sensor_t_value * 100) - temp_integer_part*100;

        buf[0] = sensor_t_value > 0 ? 1 : 0;
        buf[1] = temp_integer_part;
        buf[2] = temp_decimal_part;

        hum_integer_part = (int) sensor_h_value;
        hum_decimal_part = (int)(sensor_h_value * 100) - hum_integer_part*100;

        buf[3] = hum_integer_part;
        buf[4] = hum_decimal_part;

        buf[5] = is_in_comfort_zone(sensor_t_value , sensor_h_value);

        if (_started == 0) {
            last_acc_samples[0] = ppush(last_acc_samples[0], accel_value[0]);
            last_acc_samples[1] = ppush(last_acc_samples[1], accel_value[1]);
            last_acc_samples[2] = ppush(last_acc_samples[2], accel_value[2]);

            last_gyro_samples[0] = ppush(last_gyro_samples[0], gyro_value[0]);
            last_gyro_samples[1] = ppush(last_gyro_samples[1], gyro_value[1]);
            last_gyro_samples[2] = ppush(last_gyro_samples[2], gyro_value[2]);

            _started = 1;
            continue;
        }

        // Movement recognition confidence
        move = MovementRecognition(
            last_acc_samples, 
            last_gyro_samples, 
            accel_value, 
            gyro_value,
            last_sampling_rates, 
            last_movement_confidence,
            last_movement_results);
        
        buf[6] = move[0];

        // Keeping how many times the recognized movement 
        // has been stayed the same
        if(move[0] == last_movement_results[0]) {
            SAMPLES_ON_THE_SAME_MOVE += 1;
        } else {
            SAMPLES_ON_THE_SAME_MOVE = 0;
        }

        // Depending on how many samples the recognized movement
        // has been rmained the same, we increase the sampling rate
        if(SAMPLES_ON_THE_SAME_MOVE >= 6){
            live_change_sampling_rate(0.7);
        } else if(SAMPLES_ON_THE_SAME_MOVE >= 4){
            live_change_sampling_rate(0.4);
        } else if(SAMPLES_ON_THE_SAME_MOVE >= 2){
            live_change_sampling_rate(0.2);
        } else if(SAMPLES_ON_THE_SAME_MOVE >= 1){
            live_change_sampling_rate(0.1);
        }

        // Updating the accelerometer last n values
        last_acc_samples[0] = ppush(last_acc_samples[0], accel_value[0]);
        last_acc_samples[1] = ppush(last_acc_samples[1], accel_value[1]);
        last_acc_samples[2] = ppush(last_acc_samples[2], accel_value[2]);

        // Updating the gyroscope last n values
        last_gyro_samples[0] = ppush(last_gyro_samples[0], gyro_value[0]);
        last_gyro_samples[1] = ppush(last_gyro_samples[1], gyro_value[1]);
        last_gyro_samples[2] = ppush(last_gyro_samples[2], gyro_value[2]);

        last_movement_results = ppush(last_movement_results, move[0]);
        last_sampling_rates = ppush(last_sampling_rates, SAMPLING_RATE_MILLISECONDS);
        last_movement_confidence = ppush(last_movement_confidence, move[1]);

        led = !led;
        uint8_t ciphertext[16];
        AES_ECB_encrypt(buf , key, ciphertext, sizeof(buf));
        serial_port.write(ciphertext, sizeof(ciphertext));

        wait_us(SAMPLING_RATE_MILLISECONDS * 1000);
    }
}