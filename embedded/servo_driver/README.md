#Calibration Procedure

In order the calibration, you'll need to decide a few things:
1. Where you want the center of the servos to sit
2. Where you want the maximum of the servos to sit
3. The angle between these two (in degrees, or radians)

Run servo_calibration. Use the number keys to put all the servos into the "center" location. Write those numbers down. Then use the number keys to put all the servos in the "max" location. Write those numbers down too. Then, go edit the cfg/servo_cfgX.yaml file. Put the center number for each servo in "calibrated_center", the max in "calibrated_max", and the max_angle in "max_angle".

Now, you can call set_angle with an angle, and the servo will move to that angle. So, for example, servo.set_angle(0) will move to the calibrated_center, and servo.set_angle(max_angle) will go to calibrated_max.

## Improvements

This calibration could use some improvements to make it easier to work with. Here's a few:
- We should use one yaml file, instead of 4
- The calibration script should write the yaml files for the user
- The file paths are fairly arbitrary, and will be confusing without samples written


