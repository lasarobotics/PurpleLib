
# PurpleLib [![Release](https://jitpack.io/v/lasarobotics/PurpleLib.svg)](https://jitpack.io/#lasarobotics/PurpleLib)


Custom library for 418 Purple Haze

Note: CTRE will not be as well supported as REV products as our team primarily lives in the REV Robotics ecosystem

## Features
* Hardware wrappers with built-in AdvantageKit logging
  * REV Robotics
    * Spark Max with SmoothMotion<sup>TM</sup>
    * Spark Flex with SmoothMotion<sup>TM</sup>
    * Through bore encoder connected to Spark Max/Flex in absolute mode ONLY!
    * Spark Flex and NEO Vortex MUST be paired together!
  * CTRE
    * CANivore
    * Pidgeon 2.0
    * CANCoder
    * VictorSPX
    * TalonSRX
  * Kauai Labs
    * NavX2
  * Generic
    * Analog sensor
    * Compressor
    * Single and double solenoid
    * Limit switch
    * Servo
* MAXSwerve module support
  * Supports NEO v1.0/1.1 or NEO Vortex + NEO 550 configuration only
* Robot rotation PID
* Traction control
* Swerve second order kinematics correction
* Configurable input maps
* LED strip support
* JSON read/write
* Battery scanning and tracking


## Installing
Add the following dependencies to your project:
* AdvantageKit - https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md
* NavX - https://dev.studica.com/releases/2024/NavX.json
* REVLib - https://software-metadata.revrobotics.com/REVLib-2024.json
* CTRE Phoenix5 - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-beta-latest.json
* CTRE Phoenix6 - https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-beta-latest.json

Add the following to `build.gradle` where VERSION is the release version, e.g. 2023.0.0
```
repositories {
  maven { url "https://jitpack.io" }
}
dependencies {
  implementation 'org.apache.commons:commons-math3:3.+'
  implementation 'com.github.lasarobotics:PurpleLib:VERSION'
}
```

## Releasing
* Create a release in GitHub. JitPack does the rest.

## Examples
Examples can be found [here](https://github.com/lasarobotics/PurpleLibExamples)

## Documentation
Javadocs available [here](https://jitpack.io/com/github/lasarobotics/PurpleLib/latest/javadoc/)
