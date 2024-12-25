// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.TractionControlController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public abstract class SwerveModule implements AutoCloseable {

  /** Mount orientation */
  public enum MountOrientation {
    /** Motors are right side up */
    STANDARD,
    /** Motors are upside down */
    INVERTED;
  }

  /** Swerve Ratio for Swerve Modules */
  public interface GearRatio {
    /**
     * Get drive gear ratio
     * @return Gear ratio for driving the wheel
     */
    public double getDriveRatio();

    /**
     * Get rotate gear ratio
     * @return Gear ratio for rotating the wheel
     */
    public double getRotateRatio();
  }

  /** Module location */
  public enum Location {
    LeftFront(Rotation2d.kCW_Pi_2.div(2)),
    RightFront(Rotation2d.kCCW_Pi_2.div(2)),
    LeftRear(Rotation2d.kCCW_Pi_2.div(2)),
    RightRear(Rotation2d.kCW_Pi_2.div(2));

    private final Rotation2d lockPosition;
    private Location(Rotation2d lockPosition) {
      this.lockPosition = lockPosition;
    }

    /**
     * Get lock position for module
     * @return Orientation for module to lock robot in place
     */
    public Rotation2d getLockPosition() {
      return lockPosition;
    }
  }

  private double m_runningOdometer;
  private String m_odometerOutputPath;

  private Location m_location;
  private GearRatio m_gearRatio;
  private DriveWheel m_driveWheel;
  private Rotation2d m_zeroOffset;
  private Translation2d m_coordinate;
  private TractionControlController m_tractionControlController;
  private final int COSINE_CORRECTION;

  private boolean m_autoLock;

  public SwerveModule(Location location,
                      GearRatio gearRatio,
                      DriveWheel driveWheel,
                      Angle zeroOffset,
                      Distance wheelbase,
                      Distance trackWidth,
                      String driveMotorName) {
    this.m_location = location;
    this.m_gearRatio = gearRatio;
    this.m_driveWheel = driveWheel;
    this.m_zeroOffset = new Rotation2d(zeroOffset);
    this.m_runningOdometer = 0.0;
    this.m_autoLock = true;
    COSINE_CORRECTION = RobotBase.isReal() ? 1 : 0;

    // Calculate module coordinate
    switch (location) {
      case LeftFront:
        m_coordinate = new Translation2d(+wheelbase.in(Units.Meters) / 2, +trackWidth.in(Units.Meters) / 2);
        break;
      case RightFront:
        m_coordinate = new Translation2d(+wheelbase.in(Units.Meters) / 2, -trackWidth.in(Units.Meters) / 2);
        break;
      case LeftRear:
        m_coordinate = new Translation2d(-wheelbase.in(Units.Meters) / 2, +trackWidth.in(Units.Meters) / 2);
        break;
      case RightRear:
        m_coordinate = new Translation2d(-wheelbase.in(Units.Meters) / 2, -trackWidth.in(Units.Meters) / 2);
        break;
      default:
        m_coordinate = new Translation2d();
        break;
    }

    // Setup disabled triggers
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> disabledInit()).ignoringDisable(true));
    RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> disabledExit()).ignoringDisable(true));

    // Read odometer file if exists
    m_odometerOutputPath = (driveMotorName + "-odometer.txt").replace('/', '-');
    File file = new File(m_odometerOutputPath);
    Distance previousDistanceTraveled = Units.Meters.of(0.0);
    if (file.exists()) {
      try {
        previousDistanceTraveled =
          Units.Meters.of(Double.parseDouble(
            new String(Files.readAllBytes(Paths.get(m_odometerOutputPath)), StandardCharsets.UTF_8)
          ));
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
    m_runningOdometer += previousDistanceTraveled.in(Units.Meters);
  }

  /**
   * Call method during initialization of disabled mode to set drive motor to brake mode and log odometry
   */
  private void disabledInit() {
    enableBrakeMode(true);

    try {
      FileWriter fileWriter = new FileWriter(m_odometerOutputPath);
      fileWriter.write(String.valueOf(m_runningOdometer));
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Call method when exiting disabled mode to set drive motor to coast mode
   */
  private void disabledExit() {
    enableBrakeMode(false);
  }

  /**
   * Set traction control controller
   * <p>
   * MUST be called in constructor of child class!
   * @param controller Traction control controller
   */
  protected void setTractionControlController(TractionControlController controller) {
    this.m_tractionControlController = controller;
  }

  /**
   * Get traction control controller
   * @return Traction control controller for this module
   */
  protected TractionControlController getTractionControlController() {
    return m_tractionControlController;
  }

  /**
   * Get whether or not auto locking is enabled
   * @return True if enabled
   */
  protected boolean isAutoLockEnabled() {
    return m_autoLock;
  }

  /**
   * Add to odometer value
   * @param delta Value to add
   */
  protected void incrementOdometer(double delta) {
    m_runningOdometer += delta;
  }

  /**
   * Get velocity of module parallel to the desired orientation
   * @param state Desired swerve module state representing desired speed and orientation
   * @param realSpeeds Real speeds of robot from IMU
   * @return Velocity of module parallel to desired module orientation
   */
  protected LinearVelocity getParallelVelocity(SwerveModuleState state, ChassisSpeeds realSpeeds) {
    // Get physical velocity of module through space
    var moduleInertialVelocityState = AdvancedSwerveKinematics.getRealModuleVelocity(realSpeeds, m_coordinate);
    // Get vector of physical velocity
    var moduleInertialVelocityVector = new Vector2D(
      moduleInertialVelocityState.speedMetersPerSecond * moduleInertialVelocityState.angle.getCos(),
      moduleInertialVelocityState.speedMetersPerSecond * moduleInertialVelocityState.angle.getSin()
    );
    // Get vector of desired velocity
    var moduleDesiredVelocityVector = new Vector2D(
      state.speedMetersPerSecond * state.angle.getCos(),
      state.speedMetersPerSecond * state.angle.getSin()
    );

    // Get angle between vectors
    var vectorAngle = Units.Radians.of(Math.acos(
      moduleDesiredVelocityVector.dotProduct(moduleInertialVelocityVector) / (moduleDesiredVelocityVector.getNorm() * moduleInertialVelocityVector.getNorm())
    ));

    // Calculate portion of inertial velocity that is in direction of desired
    var parallelModuleVelocityVector = moduleDesiredVelocityVector.scalarMultiply(
      moduleDesiredVelocityVector.getNorm() != 0.0
        ? moduleInertialVelocityVector.dotProduct(moduleDesiredVelocityVector) / moduleDesiredVelocityVector.getNormSq()
        : 1.0
    );

    // Get velocity
    var parallelModuleVelocity = Units.MetersPerSecond.of(parallelModuleVelocityVector.getNorm());

    // If requested state is generally in the opposite direction of inertial, negate the velocity to be reported
    if (vectorAngle.gt(Units.Radians.of(Math.PI / 2))) parallelModuleVelocity = parallelModuleVelocity.times(-1);

    // Return velocity
    return parallelModuleVelocity;
  }

  /**
   * Get desired swerve module state
   * @param requestedState Requested state
   * @return Actual desired state for module
   */
  protected SwerveModuleState getDesiredState(SwerveModuleState requestedState, Rotation2d currentAngle) {
    // Apply chassis angular offset to the requested state.
    var desiredState = new SwerveModuleState(
      requestedState.speedMetersPerSecond,
      requestedState.angle.plus(m_zeroOffset)
    );

    // Optimize swerve module rotation state
    // REV encoder returns an angle in radians
    desiredState.optimize(currentAngle);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother driving.
    desiredState.speedMetersPerSecond *= Math.pow(desiredState.angle.minus(currentAngle).getCos(), COSINE_CORRECTION);

    // Return corrected desired swerve module state
    return desiredState;
  }

  /**
   * Get drive wheel installed on module
   * @return Drive wheel
   */
  public DriveWheel getDriveWheel() {
    return m_driveWheel;
  }

  /**
   * Get location of module on robot chassis
   * @return Location of module
   */
  public SwerveModule.Location getModuleLocation() {
    return m_location;
  }

  /**
   * Get drive gear ratio
   * @return Gear ratio for driving wheel
   */
  public SwerveModule.GearRatio getGearRatio() {
    return m_gearRatio;
  }

  /**
   * Get coordinate of module relative to the center of the robot
   * @return X, Y coordinate in meters
   */
  public Translation2d getModuleCoordinate() {
    return m_coordinate;
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param state Desired swerve module state representing desired speed
   * @param realSpeeds Real speeds of robot from IMU
   */
  public void set(SwerveModuleState state, ChassisSpeeds realSpeeds) {
    // Apply traction control
    state.speedMetersPerSecond = m_tractionControlController.calculate(
      Units.MetersPerSecond.of(state.speedMetersPerSecond),
      getParallelVelocity(state, realSpeeds),
      getDriveVelocity()
    ).in(Units.MetersPerSecond);

    // Set swerve module state
    set(state);
  }

  /**
   * Set swerve module direction and speed
   * @param states Array of states for all swerve modules
   */
  public void set(SwerveModuleState[] states) {
    set(states[m_location.ordinal()]);
  }

  /**
   * Set swerve module direction and speed, automatically applying traction control
   * @param states Array of states for all swerve modules representing desired speed
   * @param realSpeeds Real speeds of robot from IMU
   */
  public void set(SwerveModuleState[] states, ChassisSpeeds realSpeeds) {
    set(states[m_location.ordinal()], realSpeeds);
  }

  /**
   * Lock swerve module
   */
  public void lock() {
    set(new SwerveModuleState(0.0, m_location.getLockPosition()));
  }

  /**
   * Reset swerve module to 0 degrees
   */
  public void reset() {
    set(new SwerveModuleState(0.0, m_zeroOffset.unaryMinus()));
  }

  /**
   * Get if drive wheel is slipping
   * @return True if wheel is slipping excessively
   */
  public boolean isSlipping() {
    return m_tractionControlController.isSlipping();
  }

  /**
   * Get whether traction control is enabled
   * @return True if enabled
   */
  public boolean isTractionControlEnabled() {
    return m_tractionControlController.isEnabled();
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggleTractionControl();
    if (m_tractionControlController.isEnabled()) enableBrakeMode(false);
    else enableBrakeMode(true);
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enableTractionControl();
    enableBrakeMode(false);
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disableTractionControl();
    enableBrakeMode(true);
  }

  /**
   * Reset the total distance the swerve module has traveled
   */
  public void clearRunningOdometer() {
    m_runningOdometer = 0.0;
    try {
      FileWriter fileWriter = new FileWriter(m_odometerOutputPath);
      fileWriter.write(String.valueOf(m_runningOdometer));
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Get the total distance the swerve module has traveled
   * @return Odometer value for the swerve module
   */
  public Distance getRunningOdometer() {
    return Units.Meters.of(m_runningOdometer);
  }


  /**
   * Get maximum drive velocity of module
   * @return Max linear velocity
   */
  public LinearVelocity getMaxLinearVelocity() {
    return m_tractionControlController.getMaxLinearVelocity();
  }

  /**
   * Enables auto locking. If enabled, the wheels will rotate to a state that prevents translation when not driving.
   */
  public void enableAutoLock() {
    m_autoLock = true;
  }

  /**
   * Disables auto locking. If disabled, the wheels will not automatically rotate when not driving.
   */
  public void disableAutoLock() {
    m_autoLock = false;
  }

  /**
   * Set swerve module direction and speed
   * @param state Desired swerve module state
   */
  public abstract void set(SwerveModuleState state);


  /**
   * Enable brake mode on drive mtor
   * @param enable True to enable brake mode, false for coast
   */
  public abstract void enableBrakeMode(boolean enable);

  /**
   * Get current module state
   * @return Current module state
   */
  public abstract SwerveModuleState getState();

  /**
   * Get module position
   * @return Current module position
   */
  public abstract SwerveModulePosition getPosition();

  /**
   * Get velocity of drive motor
   * @return Linear velocity of drive motor
   */
  public abstract LinearVelocity getDriveVelocity();

  /**
   * Reset drive motor encoder
   */
  public abstract void resetDriveEncoder();

  /**
   * Stop swerve module
   */
  public abstract void stop();

  @Override
  public abstract void close();
}
