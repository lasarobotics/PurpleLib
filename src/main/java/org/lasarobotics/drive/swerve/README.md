# REV Swerve

In order to use Spark motor controllers, you MUST have a PWM absolute encoder connected to the Spark that controls the rotation motor.

Options include:
 * [REV Through Bore Encoder](https://www.revrobotics.com/rev-11-1271/)
 * [Redux Robotics HELIUM Canandcoder (in PWM mode)](https://shop.reduxrobotics.com/helium-canandmag/)
 * [Thriftybot Absolute Magnetic Encoder](https://www.thethriftybot.com/products/thrifty-absolute-magnetic-encoder)

 # CTRE Swerve

 At the current time, the CTRE CANCoder is the ONLY encoder supported.

## MAXSwerve

### Calibration
Modules must be calibrated as per manufacturer instructions: https://docs.revrobotics.com/brushless/spark-max/encoders/maxswerve

### Sample PID values
These values can be a good starting point for MAXSwerve.
Note that rotate feed forward values are only used for simulation.

Drive PID: 0.3, 0.0, 0.001

Drive kS: 0.2

Drive kV: 12 / (calculated max speed in meters per second)

Drive kA: 0.5

Rotate PID: 2.0, 0.0, 0.1

Rotate kS: 0.2

Rotate kA: 0.01

## Swerve X

### Calibration
Module must be calibrated as per the manufacturer instructions for each respective module:
 * [Swerve X/Xi](https://docs.wcproducts.com/wcp-swervex/overview-and-features/zeroing-module)
 * [Swerve X2](https://docs.wcproducts.com/wcp-swerve-x2/overview-and-features/zeroing-module)

## SDS

### Calibration
SDS does not use a "calibration tool" or jig like MAXSwerve or WCP. As such, I have assumed that users will zero the modules to be straight ahead, aligned with the chassis. Therefore the "zero offset" for these modules is zero.