# Swerve Drive template

Simply extend the [SwerveDrive](src/main/java/org/lasarobotics/drive/swerve/SwerveDrive.java) for your drive subsystem, and call the `super()` constructor with the appropriate arguments.

Use the [PurpleSwerve](https://github.com/lasarobotics/PurpleSwerve) repo as an example

## REV Swerve

In order to use Spark motor controllers, you MUST have a PWM absolute encoder directly connected to the Spark that controls the rotation motor.

Options include:
 * [REV Through Bore Encoder](https://www.revrobotics.com/rev-11-1271/)
 * [Redux Robotics HELIUM Canandcoder (in PWM mode)](https://shop.reduxrobotics.com/helium-canandmag/)
 * [Thriftybot Absolute Magnetic Encoder](https://www.thethriftybot.com/products/thrifty-absolute-magnetic-encoder)

 To create an instance of a module, use the helper class for your module, and call the `create()` method for REV.

 For example, here's how to create a MAXSwerve module:
 ```
REVSwerveModule lFrontModule = MAXSwerveModule.create(
   REVSwerveModule.initializeHardware(
     new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
     new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
     MotorKind.NEO_VORTEX,
     MotorKind.NEO_550
   ),
   SwerveModule.Location.LeftFront,
   MAXSwerveModule.GearRatio.L3,
   DriveWheel.create(Units.Inches.of(3.0), Units.Value.of(0.9), Units.Value.of(0.8)),
   PIDConstants.of(0.3, 0.0, 0.001, 0.0, 0.0),
   FFConstants.of(0.2, 0.0, 0.0, 0.5),
   PIDConstants.of(2.0, 0.0, 0.1, 0.0, 0.0),
   FFConstants.of(0.2, 0.0, 0.0, 0.01),
   Units.Percent.of(8.0),
   Units.Pounds.of(135.0),
   Units.Meters.of(0.5588),
   Units.Meters.of(0.5588),
   Units.Seconds.of(3.0),
   Units.Amps.of(60.0)
);
```

## CTRE Swerve

 At the current time, the CTRE CANCoder is the ONLY absolute encoder supported.

 Use of CANivore is HIGHLY recommended, as adjustment of status frame rates isn't easily possible while protecting the underlying sensor getter methods.

 CANivore must be named "canivore" using Phoenix Tuner X.

 To create an instance of a module, use the helper class for your module, and call the `create()` method for CTRE.

For example, here's how to create a Swerve X2i module:
 ```
CTRESwerveModule lFrontModule = SwerveX2Module.create(
   CTRESwerveModule.initializeHardware(
     new TalonFX.ID("DriveHardware/Swerve/LeftFront/Drive", PhoenixCANBus.CANIVORE, 2);
     new TalonFX.ID("DriveHardware/Swerve/LeftFront/Rotate", PhoenixCANBus.CANIVORE, 3);
     new CANcoder.ID("DriveHardware/Swerve/LeftFront/Encoder", PhoenixCANBus.CANIVORE, 10);
   ),
   SwerveModule.Location.LeftFront,
   SwerveModule.MountOrientation.INVERTED
   SwerveX2Module.GearRatio.X4_3,
   DriveWheel.create(Units.Inches.of(3.0), Units.Value.of(0.9), Units.Value.of(0.8)),
   PIDConstants.of(0.3, 0.0, 0.001, 0.0, 0.0),
   FFConstants.of(0.2, 0.0, 0.0, 0.5),
   PIDConstants.of(2.0, 0.0, 0.1, 0.0, 0.0),
   FFConstants.of(0.2, 0.0, 0.0, 0.01),
   Units.Percent.of(8.0),
   Units.Pounds.of(135.0),
   Units.Meters.of(0.5588),
   Units.Meters.of(0.5588),
   Units.Seconds.of(3.0),
   Units.Amps.of(60.0)
);
```

### MAXSwerve

Only REV Spark motor controllers are currently supported with MAXSwerve.

#### Calibration
Modules must be calibrated as per manufacturer instructions:
https://docs.revrobotics.com/brushless/spark-max/encoders/maxswerve

#### Sample PID values
These values can be a good starting point for MAXSwerve.
Note that rotate feed forward values are only used for simulation.

Drive PID: 0.3, 0.0, 0.001

Drive kS: 0.2

Drive kV: 0.0

Drive kA: 0.5

Rotate PID: 2.0, 0.0, 0.1

Rotate kS: 0.2

Rotate kA: 0.01

### Swerve X

#### Calibration
Module must be calibrated as per the manufacturer instructions for each respective module:
 * [Swerve X/Xi](https://docs.wcproducts.com/wcp-swervex/overview-and-features/zeroing-module)
 * [Swerve X2](https://docs.wcproducts.com/wcp-swerve-x2/overview-and-features/zeroing-module)

### SDS

#### Calibration
SDS does not use a "calibration tool" or jig like MAXSwerve or WCP. As such, I have assumed that users will zero the modules to be straight ahead, aligned with the chassis. Therefore the "zero offset" for these modules is zero.