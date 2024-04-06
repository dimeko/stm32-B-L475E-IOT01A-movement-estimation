### Simple movement recognition for wearable STM32 B-L475VG-IOT01A

A small program that recognizes the four movement/gestures standing, walking, running and falling and sends the results to a hub (Matlab).
Along with the gesture recognition, the microcontroller, captures the current humidity and temperature and sends them to the hub along with the a boolean value that says if the environment is inside the human comfort zone, specified on this (article from Berkeley, Tartarini, F., Schiavon, S., Cheung, T., Hoyt, T.)[https://comfort.cbe.berkeley.edu/]
The microcontroller is the STM32 B-L475VG-IOT01A with the mbed-os 6.17.0 firmware.

#### Movement recognition logic

The accelerometer and gyroscope, read the values in arrays of length 3 [x, y, z] and we store them in two separate 3-length arrays. The movement types that the program can recognize are standing, walking, running and falling. The user movement is considered as periodic and back-and-forth. So we just check the scale of the change in the accelerator and gyroscopre values.
For example, if the difference between the current accelerometer Z axis and previous one is above 600, then the result is "run".

In addition, we store the last 10 samples in two 2d matrices `[[x1,y1,z1], [x2,y2,z2],...,[x10,y10,z10]]`, one for each movement sensor, the last 10 recognition results and the last 10 result confidence.

#### Sampling rate change

In order to use the least possible resources, we modify the sampling rate depending on the scale of the sensor values change. If there are not big changes then we decrease the sampling rate.
The change of the sampling rate is happening in an exponential rate using the function  `live_change_sampling_rate`. The function argument, is actually the number that is going to be added to the exponent.
```c
void live_change_sampling_rate(float add_exp){
    SAMPLING_RATE_EXP = SAMPLING_RATE_EXP + add_exp;
    if(SAMPLING_RATE_EXP >= 7){
        SAMPLING_RATE_EXP = 6;
    } else if(SAMPLING_RATE_EXP <= 1) {
        SAMPLING_RATE_EXP = 1;
    } else {}
    SAMPLING_RATE_MILLISECONDS = LEAST_SAMPLING_RATE_MILLISECONDS + exp(SAMPLING_RATE_EXP);
}
```

##### Development 
Keil studio, mbed-os version 6.17.0
