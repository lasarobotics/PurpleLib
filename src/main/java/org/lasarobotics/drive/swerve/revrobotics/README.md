## MAXSwerve

### Calibration
Modules must be calibrated as per manufacturer instructions: https://docs.revrobotics.com/brushless/spark-max/encoders/maxswerve

### Sample PID values
These values can be a good starting point for your tuning.
Note that rotate feed forward values are only used for simulation.

Drive PID: 0.3, 0.0, 0.001

Drive kS: 0.2

Drive kV: 12 / (calculated max speed in meters per second)

Drive kA: 0.5

Rotate PID: 2.0, 0.0, 0.1

Rotate kS: 0.2

Rotate kA: 0.01

## SDS

### Calibration
SDS does not use a "calibration tool" like MAXSwerve or WCP. As such, I have simply used the same rotation offsets as MAXSwerve. This assumes that your team designs a calibration tool for SDS that is similar to the MAXSwerve one.