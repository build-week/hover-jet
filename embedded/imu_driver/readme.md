IMU Driver
==========

This bq has a hard-coded sample rate for the imu; the IMU does not permit arbitrarily configured sample rates.
As such, we've chosen sample rates for the accelerometer and gyro that are as close as possible to 100Hz, while being greater than 100Hz.

## Notes
* The Linux i2c driver manages contention for the bus. Since the adafruit driver busy-waits for results from each imu, it makes sense to isolate multiple drivers to independent processes. Remember kids, threads are just processes with different flags.


## Issues with the Sparkfun BNO055 API
* (Indirectly through i2c) -- it polls the device without waiting for it to be ready
    * Next project; Invest in i2c infrastructure that doesn't use "sleep" as a synchronization primitive
* Not const-correct
* It hangs onto a `char*`; that's a disaster waiting to happen