![Release](https://jitpack.io/v/lasarobotics/PurpleLib.svg)
(https://jitpack.io/#lasarobotics/PurpleLib)

# PurpleLib
Custom library for FRC team 418, Purple Haze

Features:
* Spark Max wrapper with AdvantageKit logging and smooth motion
* MAXSwerve module support
* Turn PID
* Traction control
* Swerve second order kinematics correction
* Configurable input maps
* LED strip support
* JSON read/write
* Battery scanning and tracking


# Installing
Add the following to `build.gradle` where VERSION is the release tag, e.g. 2023.0.0
```
repositories {
  maven { url "https://jitpack.io" }
}
dependencies {
  implementation 'com.github.lasarobotics:PurpleLib:VERSION'
}
```

# Releasing
* Create a release in GitHub. Jitpack does the rest.
