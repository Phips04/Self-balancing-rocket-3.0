# Self-balancing-rocket-3.0
Official code repository of the self-balancing rocket 3.0.

The code file is meant to run on an arduino nano yC.
The yC reads values provided by an MPU6050 IMU module, these values are influenced by gyroscope readings as well as data obtained from the accelerometer.
The angels (pitch / roll) provided by the IMu will be fed into an PID-controller, which only consists of an P-component so far, the PID-controller will direct the servos to correct for an faulty attitude.

The conde is licensed under the GNU 3.0 license
