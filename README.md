
# PurpleLib [![Release](https://jitpack.io/v/lasarobotics/PurpleLib.svg)](https://jitpack.io/#lasarobotics/PurpleLib)


Custom library for 418 Purple Haze


## Features
* Hardware wrappers with built-in AdvantageKit logging
  * REV Robotics
    * Spark fault monitoring
    * Through bore encoder connected to Spark Max/Flex in absolute mode ONLY!
    * Spark Flex and NEO Vortex MUST be paired together!
    * Improved velocity PID performance
    * More accurate velocity readings
  * CTRE
    * CANivore
      * Use of CANivore is HIGHLY recommended, as adjustment of status frame rates isn't easily possible while protecting the underlying sensor getter methods.
    * Pigeon 2.0
    * CANcoder
    * TalonFX
    * VictorSPX
    * TalonSRX
  * Kauai Labs
    * NavX2 (MXP port only)
      * X/Y axis swapping
      * X/Y axis inversion
      * Field centric and robot centric velocity readings
  * Redux Robotics
    * Canandgyro
  * ThriftyBot
    * ThriftyNova
  * PhotonVision
    * Includes camera simulation
    * AprilTag 3D pose estimation cameras
    * AI object detection cameras (coming soon)
  * Generic
    * Analog sensor
    * Compressor
    * Single and double solenoid
    * Limit switch
      * Includes support for interrupts
    * Servo
* Swerve module support with both REV and CTRE motor controllers
  * Supported modules:
    * REV MAXSwerve (REV Spark only)
    * SDS MK4/MK4i/MK4c/MK4n
    * WCP Swerve X/Xi/X2
    * TTB Swerve
    * Support for custom or other modules can be added easily!
  * Module must be calibrated using calibration tool provided by module manufacturer
  * Mixing motor controller vendors within a module is NOT supported (ex. TalonFX for drive, Spark Max for rotation)
  * Automatic module "locking"
  * Traction control
  * More info [here](src/main/java/org/lasarobotics/drive/swerve/README.md)
* Swerve drive parent class
  * Field centric or robot centric drive
  * High frequency, threaded odometry
    * 100Hz REV, 200Hz for CTRE
    * IMU must be set to same update frequency
  * Easy PhotonVision integration
  * Robot rotation PID
  * Swerve second order kinematics correction
  * More info [here](src/main/java/org/lasarobotics/drive/swerve/README.md)
* State Machine Structure
  * Easy-to-add states
  * Improved accessibility over solely command-based infrastucture
  * Implemented StateMachine class for additional compatability
  * More info [here](src/main/java/org/lasarobotics/fsm/README.md)
* Health monitoring and automatic recovery
  * Available in the hardware wrappers and for subsystems
* Configurable input maps
* LED strip support
* JSON read/write
* Battery scanning and tracking


## Installing

### Required dependencies
* AdvantageKit - https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md
* URCL - https://raw.githubusercontent.com/Mechanical-Advantage/URCL/maven/URCL.json
* NavX - https://dev.studica.com/releases/2025/Studica-2025.0.0.json
* REVLib - https://software-metadata.revrobotics.com/REVLib-2025.json
* ReduxLib - https://frcsdk.reduxrobotics.com/ReduxLib_2025.json
* ThriftyLib - https://docs.home.thethriftybot.com/ThriftyLib.json
* CTRE Phoenix5 - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2025-latest.json
* CTRE Phoenix6 - https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
* PhotonLib - https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json

### Online installation
Add the following to your project `build.gradle` where VERSION is the release version, e.g. 2023.0.0
```
repositories {
  maven { url "https://jitpack.io" }
}
dependencies {
  implementation 'org.apache.commons:commons-math3:3.+'
  implementation 'com.github.lasarobotics:PurpleLib:VERSION'
}
```

### Offline installation
When PurpleLib is built from source, files are placed in the `build/libs` directory.
Create a link to this directory and copy the link to the root of your project,
or copy the directory itself to the root of your robot project.

Add the following to your project `build.gradle`
```
dependencies {
  implementation 'org.apache.commons:commons-math3:3.+'
  implementation files('libs/PurpleLib.jar')
}
```
Change the file path as needed.

## Robot project integration

1. Follow AdvantageKit setup instructions, and make your main `Robot.java` class extend `LoggedRobot`.
2. In your `build.gradle` add the following:
  * To the `plugins` section: `id "com.peterabeles.gversion" version "1.10"`
  * Paste this before the `deploy` section:
  ```
  project.compileJava.dependsOn(createVersionFile)
  gversion {
    srcDir       = "src/main/java/"
    classPackage = "frc.robot"
    className    = "BuildConstants"
    dateFormat   = "yyyy-MM-dd HH:mm:ss z"
    timeZone     = "America/Chicago" // Use preferred time zone
    indent       = "  "
  }
  ```

3. In your `Robot.java` constructor, initialize the `PurpleManager`.
```
PurpleManager.initialize(
  this,
  AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
  Path.of("/media/sda1"),
  BuildConstants.MAVEN_NAME,
  BuildConstants.GIT_SHA,
  BuildConstants.BUILD_DATE,
  true
);
```
4. Call `PurpleManager.update()` in the beginning of the `robotPeriodic()` method of `Robot.java`.
5. Add the following to your `.gitignore`.
```
# Build Constants
src/main/java/frc/robot/BuildConstants.java

# Battery Tracker
previous_battery.txt

# Odometer
*-odometer.txt

# CTRE sim
ctre_sim/

# Local libs
libs
libs/
```

## Releasing
Create a release in GitHub. JitPack does the rest.

## Examples
Usage examples can be found [here](https://github.com/lasarobotics/PurpleLibExamples)

An example swerve project is [here](https://github.com/lasarobotics/PurpleSwerve)

## Documentation
Javadocs available [here](https://jitpack.io/com/github/lasarobotics/PurpleLib/master-SNAPSHOT/javadoc/)
