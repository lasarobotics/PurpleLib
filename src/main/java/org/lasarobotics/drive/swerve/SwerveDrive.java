// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.drive.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.SwervePoseEstimatorService;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.parent.CTRESwerveModule;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.hardware.IMU;
import org.lasarobotics.utils.CommonTriggers;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Swerve drive template */
public class SwerveDrive extends SubsystemBase implements AutoCloseable {

  /**
   * Swerve drive hardware
   */
  public record Hardware(IMU imu,
                         SwerveModule lFrontModule,
                         SwerveModule rFrontModule,
                         SwerveModule lRearModule,
                         SwerveModule rRearModule,
                         AprilTagCamera... cameras) {}

  // Drive specs
  public final LinearVelocity DRIVE_MAX_LINEAR_SPEED;
  public final LinearAcceleration DRIVE_AUTO_ACCELERATION;

  // Other settings
  private static final Angle DEFAULT_TIP_THRESHOLD = Units.Degrees.of(30.0);
  private static final double DEFAULT_BALANCED_THRESHOLD = 10.0;
  private static final double DEFAULT_AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR = 0.1;
  private static final double MIN_HEADING_TRANSLATION_DELTA = 1e-5;
  private static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, Math.toRadians(1.0));
  private static final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(2160.0, 4320.0);
  private static final Angle ROTATE_TOLERANCE = Units.Degrees.of(2.5);
  private static final AngularVelocity AIM_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(20.0);

  // Log
  private static final String POSE_LOG_ENTRY = "/Pose";
  private static final String DRIVE_MOTOR_SYSID_STATE_LOG_ENTRY = "/DriveMotorSysIDTestState";
  private static final String ROTATE_MOTOR_SYSID_STATE_LOG_ENTRY = "/RotateMotorSysIDTestState";
  private static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
  private static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";
  private static final String IS_AIMED_LOG_ENTRY = "/IsAimed";

  private final Command SET_ALLIANCE_COMMAND = Commands.runOnce(() -> {
    // Try to get alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return;

    // Set alliance if available
    setAlliance(alliance.get());
  }).andThen(Commands.waitSeconds(2)).ignoringDisable(true).repeatedly();

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> {},
    () -> antiTip(),
    (interrupted) -> {
      resetRotatePID();
      stop();
      lock();
    },
    this::isBalanced,
    this
  );

  private static Alliance s_currentAlliance;

  private ThrottleMap m_throttleMap;
  private RotatePIDController m_rotatePIDController;
  private ProfiledPIDController m_autoAimPIDControllerFront;
  private ProfiledPIDController m_autoAimPIDControllerBack;
  private SwerveDriveKinematics m_kinematics;
  private SwervePoseEstimatorService m_swervePoseEstimatorService;
  private AdvancedSwerveKinematics m_advancedKinematics;

  private IMU m_imu;
  private SwerveModule m_lFrontModule;
  private SwerveModule m_rFrontModule;
  private SwerveModule m_lRearModule;
  private SwerveModule m_rRearModule;

  private ControlCentricity m_controlCentricity;
  private ChassisSpeeds m_desiredChassisSpeeds;
  private Rotation2d m_allianceCorrection;
  private Pose2d m_previousPose;
  private Optional<Rotation2d> m_currentHeading;
  private Field2d m_field;
  private Trigger m_tippingTrigger;
  private Angle m_tipThreshold;
  private List<Runnable> m_simCallbacks;
  private Consumer<SysIdRoutineLog.State> m_driveSysIDLogConsumer;
  private Consumer<SysIdRoutineLog.State> m_rotateSysIDLogConsumer;


  private boolean m_isTractionControlEnabled = true;
  private boolean m_autoAimFront = false;
  private boolean m_autoAimBack = false;
  private double m_aimVelocityFudgeFactor;

  /**
   * Create an instance of SwerveDrive
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by swerve drive
   * @param rotatePIDF PID gains for rotate PID
   * @param aimPIDF PID gains for aim PID
   * @param controlCentricity Control centricity
   * @param throttleInputCurve Spline function characterising throttle input
   * @param rotateInputCurve Spline function characterising rotate input
   * @param rotateScalar Scalar for rotate input
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param lookAhead Rotate PID lookahead time
   */
  public SwerveDrive(Hardware drivetrainHardware, PIDConstants rotatePIDF, PIDConstants aimPIDF, ControlCentricity controlCentricity,
                     PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction rotateInputCurve,
                     Angle rotateScalar, Dimensionless deadband, Time lookAhead) {
    DRIVE_MAX_LINEAR_SPEED = drivetrainHardware.lFrontModule.getMaxLinearVelocity();
    DRIVE_AUTO_ACCELERATION = Units.Gs.one().times(drivetrainHardware.lFrontModule.getDriveWheel().staticCoF);
    this.m_imu = drivetrainHardware.imu;
    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    this.m_controlCentricity = controlCentricity;
    this.m_throttleMap = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
    this.m_rotatePIDController = new RotatePIDController(rotateInputCurve, rotatePIDF, rotateScalar, deadband, lookAhead);
    this.m_allianceCorrection = Rotation2d.kZero;
    this.m_tippingTrigger = new Trigger(this::isTipping);
    this.m_tipThreshold = DEFAULT_TIP_THRESHOLD;
    this.m_simCallbacks = new ArrayList<>();
    this.m_aimVelocityFudgeFactor = DEFAULT_AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR;
    s_currentAlliance = Alliance.Blue;

    if (m_lFrontModule instanceof CTRESwerveModule) {
      m_driveSysIDLogConsumer = state -> SignalLogger.writeString(getName() + DRIVE_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());
      m_rotateSysIDLogConsumer = state -> SignalLogger.writeString(getName() + ROTATE_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());
    } else if (m_lFrontModule instanceof REVSwerveModule) {
      m_driveSysIDLogConsumer = state -> Logger.recordOutput(getName() + DRIVE_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());
      m_rotateSysIDLogConsumer = state -> Logger.recordOutput(getName() + ROTATE_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());
    }

    // Reset IMU
    m_imu.reset();

    // Setup rotate PID
    m_rotatePIDController.setTolerance(ROTATE_TOLERANCE, AIM_VELOCITY_THRESHOLD);
    m_rotatePIDController.setSetpoint(getAngle());

    // Define drivetrain kinematics
    m_kinematics = new SwerveDriveKinematics(m_lFrontModule.getModuleCoordinate(),
                                             m_rFrontModule.getModuleCoordinate(),
                                             m_lRearModule.getModuleCoordinate(),
                                             m_rRearModule.getModuleCoordinate());

    // Define advanced drivetrain kinematics
    m_advancedKinematics = new AdvancedSwerveKinematics(m_lFrontModule.getModuleCoordinate(),
                                                        m_rFrontModule.getModuleCoordinate(),
                                                        m_lRearModule.getModuleCoordinate(),
                                                        m_rRearModule.getModuleCoordinate());

    // Initialise pose estimator
    m_swervePoseEstimatorService = new SwervePoseEstimatorService(
      ODOMETRY_STDDEV,
      m_imu,
      m_lFrontModule,
      m_rFrontModule,
      m_lRearModule,
      m_rRearModule
    );
    m_swervePoseEstimatorService.addAprilTagCamera(drivetrainHardware.cameras);
    m_swervePoseEstimatorService.start();

    // Initialise chassis speeds
    m_desiredChassisSpeeds = new ChassisSpeeds();

    // Setup anti-tip command
    m_tippingTrigger.whileTrue(ANTI_TIP_COMMAND);

    // Setup auto-aim PID controller
    m_autoAimPIDControllerFront = new ProfiledPIDController(aimPIDF.kP, 0.0, aimPIDF.kD, AIM_PID_CONSTRAINT, aimPIDF.period.in(Units.Seconds));
    m_autoAimPIDControllerFront.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerFront.setTolerance(ROTATE_TOLERANCE.in(Units.Degrees));
    m_autoAimPIDControllerFront.setIZone(aimPIDF.kIZone);
    m_autoAimPIDControllerBack = new ProfiledPIDController(aimPIDF.kP, 0.0, aimPIDF.kD, AIM_PID_CONSTRAINT, aimPIDF.period.in(Units.Seconds));
    m_autoAimPIDControllerBack.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerBack.setTolerance(ROTATE_TOLERANCE.in(Units.Degrees));
    m_autoAimPIDControllerBack.setIZone(aimPIDF.kIZone);

    // Initialise other variables
    m_previousPose = new Pose2d();
    m_currentHeading = Optional.empty();

    // Set alliance triggers
    CommonTriggers.isDSAttached().onTrue(SET_ALLIANCE_COMMAND);
    RobotModeTriggers.disabled().whileTrue(SET_ALLIANCE_COMMAND);

    // Initialise field
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Disable traction control in simulation
    if (RobotBase.isSimulation()) disableTractionControl();
  }

  /**
   * Get current alliance
   * @return Current alliance
   */
  public static Alliance getAlliance() {
    return s_currentAlliance;
  }

  /**
   * Set alliance
   * <p>
   * Must be set to correct for field oriented drive
   * @param alliance alliance
   */
  private void setAlliance(Alliance alliance) {
    s_currentAlliance = alliance;
    m_allianceCorrection = s_currentAlliance.equals(Alliance.Red) ? Rotation2d.kPi : Rotation2d.kZero;
  }

  /**
   * Set swerve modules
   * @param moduleStates Array of calculated module states
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates) {
    m_lFrontModule.set(moduleStates);
    m_rFrontModule.set(moduleStates);
    m_lRearModule.set(moduleStates);
    m_rRearModule.set(moduleStates);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Set swerve modules, automatically applying traction control
   * @param moduleStates Array of calculated module states
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Current robot rotate rate
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates, ChassisSpeeds inertialSpeeds) {
    m_lFrontModule.set(moduleStates, inertialSpeeds);
    m_rFrontModule.set(moduleStates, inertialSpeeds);
    m_lRearModule.set(moduleStates, inertialSpeeds);
    m_rRearModule.set(moduleStates, inertialSpeeds);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Drive robot and apply traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   * @param inertialVelocity Current robot inertial velocity
   */
  protected void drive(ControlCentricity controlCentricity,
                       LinearVelocity xRequest,
                       LinearVelocity yRequest,
                       AngularVelocity rotateRequest,
                       ChassisSpeeds inertialVelocity) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation().plus(m_allianceCorrection),
      controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITH traction control
    setSwerveModules(moduleStates, inertialVelocity);
  }

  /**
   * Drive robot without traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   */
  protected void drive(ControlCentricity controlCentricity,
                     LinearVelocity xRequest,
                     LinearVelocity yRequest,
                     AngularVelocity rotateRequest) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation().plus(m_allianceCorrection),
      controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);
  }

  /**
   * Get current module states
   * @return Array of swerve module states
   */
  private SwerveModuleState[] getModuleStates() {
     return new SwerveModuleState[] {
      m_lFrontModule.getState(),
      m_rFrontModule.getState(),
      m_lRearModule.getState(),
      m_rRearModule.getState()
    };
  }

  /**
   * Add custom call back to run during simulation
   * @param callback Runnable to run
   */
  protected void addSimCallback(Runnable callback) {
    m_simCallbacks.add(callback);
  }

  /**
   * Log DriveSubsystem outputs
   */
  protected void logOutputs() {
    Logger.recordOutput(getName() + POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
    Logger.recordOutput(getName() + IS_AIMED_LOG_ENTRY, isAimed());
  }

  /**
   * SmartDashboard indicators
   */
  protected void smartDashboard() {
    m_field.setRobotPose(getPose());
    SmartDashboard.putBoolean("TC", m_isTractionControlEnabled);
    SmartDashboard.putBoolean("FC", m_controlCentricity.equals(ControlCentricity.FIELD_CENTRIC));
    SmartDashboard.putString("Alliance", s_currentAlliance.name());
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  protected void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll().in(Units.Degrees), getPitch().in(Units.Degrees));

    // Drive to counter tipping motion
    drive(
      ControlCentricity.ROBOT_CENTRIC,
      DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.cos(direction)),
      DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.sin(direction)),
      Units.DegreesPerSecond.of(0.0)
    );
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Target point, pass in null to signify invalid point
   * @param reversed True to point back of robot to target
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  protected void aimAtPoint(ControlCentricity controlCentricity, double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).unaryMinus();

    var currentHeading = m_currentHeading.orElseGet(() -> Rotation2d.kZero);

    // Drive normally and return if invalid point
    if (point == null) {
      var rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).unaryMinus();
      drive(
        m_controlCentricity,
        velocityOutput.times(Math.cos(moveDirection)),
        velocityOutput.times(Math.sin(moveDirection)),
        rotateOutput,
        getInertialSpeeds()
      );
      return;
    }

    // Mark PID controller being used
    if (reversed) {
      m_autoAimFront = false;
      m_autoAimBack = true;
    } else {
      m_autoAimFront = true;
      m_autoAimBack = false;
    }

    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput.times(currentHeading.getCos()).in(Units.MetersPerSecond), velocityOutput.times(currentHeading.getSin()).in(Units.MetersPerSecond));
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ? m_aimVelocityFudgeFactor : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    var rotateOutput = reversed
      ? Units.DegreesPerSecond.of(m_autoAimPIDControllerBack.calculate(currentPose.getRotation().plus(Rotation2d.kPi).getDegrees(), adjustedAngle.getDegrees()))
      : Units.DegreesPerSecond.of(m_autoAimPIDControllerFront.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees()));

    // Log aim point
    double aimError = currentPose.getRotation().getDegrees() - adjustedAngle.getDegrees();
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
    Logger.recordOutput(getName() + "/AimError", Math.copySign(((180 - Math.abs(aimError)) % 180), (aimError)));

    // Drive robot accordingly
     drive(
      m_controlCentricity,
      velocityOutput.times(Math.cos(moveDirection)),
      velocityOutput.times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialSpeeds()
    );
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  protected void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    // Calculate move request and direction
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    // Get throttle and rotate output
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).times(-1);
    var rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).times(-1);

    // Update auto-aim controllers
    m_autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    m_autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(Rotation2d.kPi).getDegrees(),
      getPose().getRotation().plus(Rotation2d.kPi).getDegrees()
    );
    m_autoAimFront = false;
    m_autoAimBack = false;

    // Drive robot
    drive(
      m_controlCentricity,
      velocityOutput.times(Math.cos(moveDirection)),
      velocityOutput.times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialSpeeds()
    );
  }

  /**
   * Lock swerve modules
   */
  protected void lock() {
    m_lFrontModule.lock();
    m_rFrontModule.lock();
    m_lRearModule.lock();
    m_rRearModule.lock();
  }

  /**
   * Stop robot
   */
  protected void stop() {
    m_lFrontModule.stop();
    m_rFrontModule.stop();
    m_lRearModule.stop();
    m_rRearModule.stop();
  }

  /**
   * Toggle traction control
   */
  protected void toggleTractionControl() {
    m_isTractionControlEnabled = !m_isTractionControlEnabled;
    m_lFrontModule.toggleTractionControl();
    m_rFrontModule.toggleTractionControl();
    m_lRearModule.toggleTractionControl();
    m_rRearModule.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  protected void enableTractionControl() {
    m_isTractionControlEnabled = true;
    m_lFrontModule.enableTractionControl();
    m_rFrontModule.enableTractionControl();
    m_lRearModule.enableTractionControl();
    m_rRearModule.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  protected void disableTractionControl() {
    m_isTractionControlEnabled = false;
    m_lFrontModule.disableTractionControl();
    m_rFrontModule.disableTractionControl();
    m_lRearModule.disableTractionControl();
    m_rRearModule.disableTractionControl();
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  protected void resetPose(Pose2d pose) {
    m_swervePoseEstimatorService.resetPose(pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update current heading
    double xDifference = getPose().getX() - m_previousPose.getX();
    double yDifference = getPose().getY() - m_previousPose.getY();
    if (Math.abs(xDifference) > MIN_HEADING_TRANSLATION_DELTA || Math.abs(yDifference) > MIN_HEADING_TRANSLATION_DELTA)
      m_currentHeading = Optional.of(new Rotation2d(xDifference, yDifference));
    else m_currentHeading = Optional.empty();

    // Save previous pose
    m_previousPose = getPose();

    if (RobotBase.isSimulation()) return;
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    m_imu.updateSim(getPose().getRotation(), getDesiredChassisSpeeds(), m_controlCentricity);
    m_simCallbacks.stream().forEach(Runnable::run);

    smartDashboard();
    logOutputs();
  }

  /**
   * Set auto-aim velocity compensation fudge factor
   * <p>
   * This adjusts how aggressively the robot will compensate for its own velocity perpendicular to the aim vector
   * @param value Desired value, smaller is less aggressive
   */
  public void setAimVelocityCompensationFudgeFactor(double value) {
    m_aimVelocityFudgeFactor = value;
  }

  /**
   * Set tip threshold to desired angle
   * <p>
   * When the robot is tipped more than this angle in either the pitch or roll directions,
   * anti-tip will be triggered, unless disabled
   * @param threshold Anti-tip threshold
   */
  public void setTipThreshold(Angle threshold) {
    m_tipThreshold = threshold;
  }

  /**
   * Enable anti-tip functionality
   */
  public void enableAntiTip() {
    m_tippingTrigger.whileTrue(ANTI_TIP_COMMAND);
  }

  /**
   * Disable anti-tip functionality
   */
  public void disableAntiTip() {
    m_tippingTrigger.whileTrue(Commands.none());
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param speeds Calculated chassis speeds
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds);

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation(),
      ControlCentricity.ROBOT_CENTRIC
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);

    // Update turn PID
    m_rotatePIDController.calculate(getAngle(), getRotateRate(), 0.0);

    // Update auto-aim controllers
    m_autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    m_autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(Rotation2d.kPi).getDegrees(),
      getPose().getRotation().plus(Rotation2d.kPi).getDegrees()
    );
  }

  /**
   * Toggles between field centric and robot centric drive control
   */
  private void toggleControlCentricity() {
    if (m_controlCentricity == ControlCentricity.FIELD_CENTRIC) {
      this.m_controlCentricity = ControlCentricity.ROBOT_CENTRIC;
    } else {
      this.m_controlCentricity = ControlCentricity.FIELD_CENTRIC;
    }
  }

  /**
   * Get SysID command for swerve drive motors
   * @return SysID routine
   */
  public SysIdRoutine getDriveSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null,               // Use default ramp rate (1 V/s)
        Units.Volts.of(4),  // Reduce dynamic step voltage to 4 V to prevent brownout
        null,               // Use default timeout (10 s)
        m_driveSysIDLogConsumer
      ),
      new SysIdRoutine.Mechanism(
        voltage -> {
          m_lFrontModule.setDriveSysID(voltage);
          m_rFrontModule.setDriveSysID(voltage);
          m_lRearModule.setDriveSysID(voltage);
          m_rRearModule.setDriveSysID(voltage);
        },
        null,
        this
      )
    );
  }

  /**
   * Get SysID routine for swerve rotate motors
   * @return SysID routine
   */
  public SysIdRoutine getRotateSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null,               // Use default ramp rate (1 V/s)
        Units.Volts.of(7),  // Use dynamic voltage of 7 V
        null,               // Use default timeout (10 s)
        m_rotateSysIDLogConsumer
      ),
      new SysIdRoutine.Mechanism(
        voltage -> {
          m_lFrontModule.setRotateSysID(voltage);
          m_rFrontModule.setRotateSysID(voltage);
          m_lRearModule.setRotateSysID(voltage);
          m_rRearModule.setRotateSysID(voltage);
        },
        null,
        this
      )
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(
      () -> aimAtPoint(
        m_controlCentricity,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
      () -> resetRotatePID()
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(xRequestSupplier, yRequestSupplier, rotateRequestSupplier, () -> point, reversed, velocityCorrection);
  }

  /**
   * Aim robot at desired point on the field
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim robot at point while strafing
   */
  public Command aimAtPointCommand(Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> point, reversed, velocityCorrection);
  }

  /**
   * Drive the robot
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @param rotateRequestSupplier Rotate speed supplier
   * @return Command that will drive robot
   */
  public Command driveCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return run(() -> teleopPID(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()));
  }

  /**
   * Lock swerve modules
   * @return Command to lock swerve modules
   */
  public Command lockCommand() {
    return runOnce(() -> lock());
  }

  /**
   * Stop robot
   * @return Command to stop robot
   */
  public Command stopCommand() {
    return runOnce(() -> {
      stop();
      resetRotatePID();
    });
  }

  /**
   * Toggle traction control
   * @return Command to toggle traction control
   */
  public Command toggleTractionControlCommand() {
    return runOnce(() -> toggleTractionControl());
  }

  /**
   * Toggles between field and robot oriented drive control
   * @return Command to toggle control centricity between robot and field centric drive control
   */
  public Command toggleCentricityCommand() {
    return runOnce(() -> toggleControlCentricity());
  }

  /**
   * Enable traction control
   * @return Command to enable traction control
   */
  public Command enableTractionControlCommand() {
    return runOnce(() -> enableTractionControl());
  }

  /**
   * Disable traction control
   * @return Command to disable traction control
   */
  public Command disableTractionControlCommand() {
    return runOnce(() -> disableTractionControl());
  }

  /**
   * Reset pose estimator
   * @param poseSupplier Pose supplier
   * @return Command to reset pose
   */
  public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
    return runOnce(() -> resetPose(poseSupplier.get()));
  }

  /**
   * @return Command to aim a point on the field in robot centric mode
   */
  public Command aimAtPointRobotCentric(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                        Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() ->
      aimAtPoint(
        ControlCentricity.ROBOT_CENTRIC,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
      () -> resetRotatePID()
    );

  }

  /**
   * Reset DriveSubsystem turn PID
   */
  public void resetRotatePID() {
    m_rotatePIDController.setSetpoint(getAngle());
    m_rotatePIDController.reset();
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return m_swervePoseEstimatorService.getPose();
  }

  /**
   * Get drivetrain kinematics
   * @return Kinematics object
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Get whether or not robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(getPitch().in(Units.Degrees)) > m_tipThreshold.in(Units.Degrees) ||
           Math.abs(getRoll().in(Units.Degrees)) > m_tipThreshold.in(Units.Degrees);
  }


  /**
   * Get whether or not robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch().in(Units.Degrees)) < DEFAULT_BALANCED_THRESHOLD &&
           Math.abs(getRoll().in(Units.Degrees)) < DEFAULT_BALANCED_THRESHOLD;
  }

  /**
   * Get if robot is aimed at desired target
   * @return True if aimed
   */
  public boolean isAimed() {
    return (
      m_autoAimPIDControllerFront.atGoal() & m_autoAimFront
      |
      m_autoAimPIDControllerBack.atGoal() & m_autoAimBack
    ) && getRotateRate().lt(AIM_VELOCITY_THRESHOLD);
  }

  /**
   * Get desired chassis speeds
   * <p>
   * These are the last chassis speeds that were last commanded by the user
   * @return
   */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  /**
   * Get current heading of the robot (direction of travel)
   * @return Direction of travel
   */
  public Optional<Rotation2d> getCurrentHeading() {
    return m_currentHeading;
  }

  /**
   * Get inertial velocity of robot in X axis
   * @return Inertial velocity of robot in m/s
   */
  public LinearVelocity getInertialVelocityX() {
    return m_imu.getVelocityX();
  }

  /**
   * Get inertial velocity of robot in X axis
   * @return Inertial velocity of robot in m/s
   */
  public LinearVelocity getInertialVelocityY() {
    return m_imu.getVelocityY();
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public Angle getPitch() {
    return m_imu.getPitch();
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public Angle getRoll() {
    return m_imu.getRoll();
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public Angle getAngle() {
    return m_imu.getYaw();
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot
   */
  public AngularVelocity getRotateRate() {
    return m_imu.getYawRate();
  }

  /**
   * Return the heading of the robot as a Rotation2d.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * @return Current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return m_imu.getRotation2d();
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial chassis speeds of robot from IMU
   */
  public ChassisSpeeds getInertialSpeeds() {
    return new ChassisSpeeds(getInertialVelocityX(), getInertialVelocityY(), getRotateRate());
  }

  @Override
  public void close() {
    m_imu.close();
    m_lFrontModule.close();
    m_rFrontModule.close();
    m_lRearModule.close();
    m_rRearModule.close();
  }
}
