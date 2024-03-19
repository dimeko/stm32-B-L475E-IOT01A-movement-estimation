#### STM32 B-L475VG-IOT01A

A small program that (tries) to recognize the four movement/gestures standing, waling, running and falling.
It is for the microcontroller STM32 B-L475VG-IOT01A.

The logic that has been folled is so simple as watching the difference between the previous and current sample.
Falling is a little more tricky as we must the average of the max and min value of the last 10 samples and measure
the current difference of that value. If it is above a specified threshold it means that the movement changed drastically.

The development took place in Keil studio using mbed-os version 6.17.0

The serial_comm.m Matlab script works as the IoT hub. The library can be found here (???).