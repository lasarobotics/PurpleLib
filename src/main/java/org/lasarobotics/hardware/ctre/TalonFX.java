// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package org.lasarobotics.hardware.ctre;


import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DifferentialDutyCycle;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DifferentialMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.DifferentialStrictFollower;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVelocityVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Velocity;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


/** TalonFX */
public class TalonFX extends LoggableHardware {
  /** TalonFX ID */
  public static class ID {
    public final String name;
    public final PhoenixCANBus bus;
    public final int deviceID;

    /**
     * TalonFX ID
     * @param name Device name for logging
     * @param bus CAN bus
     * @param deviceID CAN ID
     */
    public ID(String name, PhoenixCANBus bus, int deviceID) {
      this.name = name;
      this.bus = bus;
      this.deviceID = deviceID;
    }
  }

  /**
   * TalonFX sensor inputs
   */
  @AutoLog
  public static class TalonFXInputs {
    public double selectedSensorPosition = 0.0;
    public double selectedSensorVelocity = 0.0;
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";

  private com.ctre.phoenix6.hardware.TalonFX m_talon;

  private ID m_id;
  private TalonFXInputsAutoLogged m_inputs;

  private TalonFXConfiguration m_TalonFXConfiguration;
  private TalonPIDConfig m_TalonPIDConfig;
  
  //Feedback sensor types
  private enum FeedbackSensor {FUSED, REMOTE, SYNC}


  /**
   * Create a TalonFX object with built-in logging
   * @param id TalonFX ID
   */
  public TalonFX(TalonFX.ID id) {
    this.m_id = id;
    this.m_talon = new com.ctre.phoenix6.hardware.TalonFX(id.deviceID);
    this.m_inputs = new TalonFXInputsAutoLogged();


    // Disable motor safety
    m_talon.setSafetyEnabled(false);


    periodic();
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlRequest mode, double value) {
    if (mode instanceof DutyCycleOut || mode instanceof TorqueCurrentFOC ||
        mode instanceof 
        )
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.toString());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  private double getSelectedSensorPosition() {
    return m_talon.getPosition().getValue();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Phoenix-Documentation for how to interpret.
   */
  private double getSelectedSensorVelocity() {
    return m_talon.getVelocity().getValue();
  }
 
  private void updateInputs() {
    m_inputs.selectedSensorPosition = getSelectedSensorPosition();
    m_inputs.selectedSensorVelocity = getSelectedSensorVelocity();
  }

  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public TalonFXInputsAutoLogged getInputs() {
    return m_inputs;
  }

  /**
   * Get device ID
   * @return Device ID
   */
  public ID getID() {
    return m_id;
  }

  
  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(VoltageConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(TorqueCurrentConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(OpenLoopRampsConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(ClosedLoopGeneralConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(DifferentialConstantsConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  } 

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(DifferentialSensorsConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(CustomParamsConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(AudioConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(ClosedLoopRampsConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

 /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(SoftwareLimitSwitchConfigs configs) {
     return m_talon.getConfigurator().apply(configs);
  }

  /**
   * Initialize stator current limits
   * @param statorCurrentLimit 
   * The amount of current allowed in the motor (motoring and regen
   * current).  This is only applicable for non-torque current control
   * modes.  Note this requires the corresponding enable to be true.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 800.0
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeStatorCurrentLimits(double statorCurrentLimit) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
      limitConfigs.StatorCurrentLimit = statorCurrentLimit;
      limitConfigs.StatorCurrentLimitEnable = true;

    m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initialize supply current limits
   * @param supplyCurrentLimit
   * The amount of supply current allowed.  This is only applicable for
   * non-torque current control modes.  Note this requires the
   * corresponding enable to be true.  Use SupplyCurrentThreshold and
   * SupplyTimeThreshold to allow brief periods of high-current before
   * limiting occurs.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 800.0
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeSupplyCurrentLimits(double supplyCurrentLimit) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
      limitConfigs.SupplyCurrentLimit = supplyCurrentLimit;
      limitConfigs.SupplyCurrentLimitEnable = true;

    m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initialize supply current threshold
   * @param supplyCurrentThreshold
   * Delay supply current limiting until current exceeds this threshold
   * for longer than SupplyTimeThreshold.  This allows current draws
   * above SupplyCurrentLimit for a fixed period of time.  This has no
   * effect if SupplyCurrentLimit is greater than this value.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 511
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeSupplyCurrentThreshold(double supplyCurrentThreshold) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
      limitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;

     m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initialize supply time threshold
   * 
   * Allows unlimited current for a period of time before current
   * limiting occurs.  Current threshold is the maximum of
   * SupplyCurrentThreshold and SupplyCurrentLimit.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 1.275
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   */
  public void initializeSupplyTimeThreshold(double supplyTimeThreshold) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
    limitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
   
    m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initiialize remote limit switches with TalonFX as the sensor
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
     limitConfigs.ForwardLimitRemoteSensorID = m_talon.getDeviceID();
   }
   if (reverse) {
     limitConfigs.ReverseLimitRemoteSensorID = m_talon.getDeviceID();
   }
   m_talon.getConfigurator().apply(limitConfigs);
  }
  
  /**
   * Initialize remote limit switches with CANcoder as the sensor
   * <p> 
   * Take note of the fact that we are using our own version of CANCoder
   * @param cancoder Sensor for limit switches
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(CANCoder cancoder, boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
    limitConfigs.ForwardLimitRemoteSensorID = cancoder.getID().deviceID;  
   }
   if (reverse) {
    limitConfigs.ReverseLimitRemoteSensorID = cancoder.getID().deviceID;
   }
   m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   Choose what sensor source is reported via API and used by
   * closed-loop and limit features.  The default is RotorSensor, which
   * uses the internal rotor sensor in the Talon FX.  Choose
   * RemoteCANcoder to use another CANcoder on the same CAN bus (this
   * also requires setting FeedbackRemoteSensorID).  Talon FX will
   * update its position and velocity whenever CANcoder publishes its
   * information on CAN bus.  Choose FusedCANcoder (requires Phoenix
   * Pro) and Talon FX will fuse another CANcoder's information with the
   * internal rotor, which provides the best possible position and
   * velocity for accuracy and bandwidth (note this requires setting
   * FeedbackRemoteSensorID).  FusedCANcoder was developed for
   * applications such as swerve-azimuth.  Choose SyncCANcoder (requires
   * Phoenix Pro) and Talon FX will synchronize its internal rotor
   * position against another CANcoder, then continue to use the rotor
   * sensor for closed loop control (note this requires setting
   * FeedbackRemoteSensorID).  The TalonFX will report if its internal
   * position differs significantly from the reported CANcoder position.
   *  SyncCANcoder was developed for mechanisms where there is a risk of
   * the CANcoder failing in such a way that it reports a position that
   * does not match the mechanism, such as the sensor mounting assembly
   * breaking off.  Choose RemotePigeon2_Yaw, RemotePigeon2_Pitch, and
   * RemotePigeon2_Roll to use another Pigeon2 on the same CAN bus (this
   * also requires setting FeedbackRemoteSensorID).  Talon FX will
   * update its position to match the selected value whenever Pigeon2
   * publishes its information on CAN bus. Note that the Talon FX
   * position will be in rotations and not degrees.
   * <p>
   * Note: When the Talon Source is changed to FusedCANcoder, the Talon
   * needs a period of time to fuse before sensor-based (soft-limit,
   * closed loop, etc.) features are used. This period of time is
   * determined by the update frequency of the CANcoder's Position
   * signal.
   * @param cancoder CANCoder object to use for feedback
   * @param sensor Enum to choose which type of CANCoder
   */
  public void initializeFeedbackSensor(CANCoder cancoder, FeedbackSensor sensor) {
    FeedbackConfigs config = m_TalonFXConfiguration.Feedback;

    //Automatically configure feedback sensor to built in rotor sensor
    config.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    switch (sensor) {
      case REMOTE:
       config.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED:
       config.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       config.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC:
       config.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       config.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
      }
      m_talon.getConfigurator().apply(config);
    }

  /**
   * Initialize Talon PID Configuration and Motion Magic
   * @param pidconfig PID Config to use
   */
  private void initializeTalonPID(TalonPIDConfig pidconfig) {
    //Initialize PID configs
    m_TalonPIDConfig = pidconfig;

    Slot0Configs slot0Configs = m_TalonFXConfiguration.Slot0;

    //Configure PID Values
      slot0Configs.kP = m_TalonPIDConfig.getkP();
      slot0Configs.kI = m_TalonPIDConfig.getkI();
      slot0Configs.kD = m_TalonPIDConfig.getkD();
      slot0Configs.kS = m_TalonPIDConfig.getkS();
      slot0Configs.kV = m_TalonPIDConfig.getkV();
      slot0Configs.kG = m_TalonPIDConfig.getkG();
      slot0Configs.kA = m_TalonPIDConfig.getkA();

    /**
     * Gravity Feedforward Type
     * <p>
     * This determines the type of the gravity feedforward. Choose
     * Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always have the same sign. Choose Arm_Cosine for systems where the
     * gravity feedforward is dependent on the angular position of the
     * mechanism, such as an arm. The gravity feedforward output will vary
     * depending on the mechanism angular position. Note that the sensor
     * offset and ratios must be configured so that the sensor reports a
     * position of 0 when the mechanism is horizonal (parallel to the
     * ground), and the reported sensor position is 1:1 with the
     * mechanism.
     */
   slot0Configs.GravityType = pidconfig.getGravityType();

     /**
     * Static Feedforward Sign during position closed loop
     * <p>
     * This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
  slot0Configs.StaticFeedforwardSign = pidconfig.getStaticFeedForward();

    //Configure MotionMagic
    MotionMagicConfigs m_motionMagic = m_TalonFXConfiguration.MotionMagic;
    if (m_TalonPIDConfig.getMotionMagic()) {
     /**
     * This is the maximum velocity Motion Magic based control modes are
     * allowed to use.  Motion Magic Velocity control modes do not use
     * this config.  When using Motion Magic Expo control modes, setting
     * this to 0 will allow the profile to run to the max possible
     * velocity based on Expo_kV.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rps
     *   </ul>
     */
     m_motionMagic.MotionMagicCruiseVelocity = m_TalonPIDConfig.getVelocityRPM();
     
     /**
     * This is the target acceleration Motion Magic based control modes
     * are allowed to use.  Motion Magic Expo control modes do not use
     * this config.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rot per sec^2
     *   </ul>
     */
     m_motionMagic.MotionMagicAcceleration = m_TalonPIDConfig.getAccelerationRPMPerSec();
     
     /**
     * This is the target jerk (acceleration derivative) Motion Magic
     * based control modes are allowed to use.  Motion Magic Expo control
     * modes do not use this config.  This allows Motion Magic support of
     * S-Curves.  If this is set to zero, then Motion Magic will not
     * apply a Jerk limit.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rot per sec^2
     *   </ul>
     */
     m_motionMagic.MotionMagicJerk = m_TalonPIDConfig.getMotionMagicJerk();
    }
   }

   /**
    * 
    */
   public StatusCode set(double speed) {
    return setControl(new DutyCycleOut(speed));
   }

  /**
   * Request a specified motor duty cycle.
   * <p>
   * This control mode will output a proportion of the supplied voltage
   * which is supplied by the user.
   *   <ul>
   *   <li> <b>DutyCycleOut Parameters:</b> 
   *    <ul>
   *    <li> <b>Output:</b> Proportion of supply voltage to apply in fractional units
   *                     between -1 and +1
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DutyCycleOut request) {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request a specified motor current (field oriented control).
   * <p>
   * This control request will drive the motor to the requested motor
   * (stator) current value.  This leverages field oriented control
   * (FOC), which means greater peak power than what is documented. 
   * This scales to torque based on Motor's kT constant.
   *   <ul>
   *   <li> <b>TorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Output:</b> Amount of motor current in Amperes
   *    <li> <b>MaxAbsDutyCycle:</b> The maximum absolute motor output that can be
   *                              applied, which effectively limits the velocity.
   *                              For example, 0.50 means no more than 50% output
   *                              in either direction.  This is useful for
   *                              preventing the motor from spinning to its
   *                              terminal velocity when there is no external
   *                              torque applied unto the rotor.  Note this is
   *                              absolute maximum, so the value should be between
   *                              zero and one.
   *    <li> <b>Deadband:</b> Deadband in Amperes.  If torque request is within
   *                       deadband, the bridge output is neutral. If deadband is
   *                       set to zero then there is effectively no deadband. Note
   *                       if deadband is zero, a free spinning motor will spin
   *                       for quite a while as the firmware attempts to hold the
   *                       motor's bemf. If user expects motor to cease spinning
   *                       quickly with a demand of zero, we recommend a deadband
   *                       of one Ampere. This value will be converted to an
   *                       integral value of amps.
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(TorqueCurrentFOC request)
  {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request a specified voltage.
   * <p>
   * This control mode will attempt to apply the specified voltage to
   * the motor. If the supply voltage is below the requested voltage,
   * the motor controller will output the supply voltage.
   *   <ul>
   *   <li> <b>VoltageOut Parameters:</b> 
   *    <ul>
   *    <li> <b>Output:</b> Voltage to attempt to drive at
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VoltageOut request)
  {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target position with duty cycle feedforward.
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. In addition, it will apply an
   * additional duty cycle as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>PositionDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second. This
   *                       is typically used for motion profiles generated by the
   *                       robot program.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionDutyCycle request)
  {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target position with voltage feedforward
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. In addition, it will apply an
   * additional voltage as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>PositionVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second. This
   *                       is typically used for motion profiles generated by the
   *                       robot program.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionVoltage request)
  {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target position with torque current feedforward.
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. In addition, it will apply an
   * additional torque current as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>PositionTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second. This
   *                       is typically used for motion profiles generated by the
   *                       robot program.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionTorqueCurrentFOC request)
  {
    logOutputs(request, request.Output);
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target velocity with duty cycle feedforward.
   * <p>
   * This control mode will set the motor's velocity setpoint to the
   * velocity specified by the user. In addition, it will apply an
   * additional voltage as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>VelocityDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
   *    <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
   *                           second squared. This is typically used for motion
   *                           profiles generated by the robot program.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target velocity with voltage feedforward.
   * <p>
   * This control mode will set the motor's velocity setpoint to the
   * velocity specified by the user. In addition, it will apply an
   * additional voltage as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>VelocityVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
   *    <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
   *                           second squared. This is typically used for motion
   *                           profiles generated by the robot program.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target velocity with torque current feedforward.
   * <p>
   * This control mode will set the motor's velocity setpoint to the
   * velocity specified by the user. In addition, it will apply an
   * additional torque current as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>VelocityTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
   *    <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
   *                           second squared. This is typically used for motion
   *                           profiles generated by the robot program.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityTorqueCurrentFOC request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  Users can optionally provide a duty cycle feedforward.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity, Acceleration, and Jerk
   * value specified via the Motion Magic® configuration values.  This
   * control mode does not use the Expo_kV or Expo_kA configs.  Target
   * position can be changed on-the-fly and Motion Magic® will do its
   * best to adjust the profile.  This control mode is duty cycle based,
   * so relevant closed-loop gains will use fractional duty cycle for
   * the numerator:  +1.0 represents full forward output.
   *   <ul>
   *   <li> <b>MotionMagicDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  Users can optionally provide a voltage feedforward.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity, Acceleration, and Jerk
   * value specified via the Motion Magic® configuration values.  This
   * control mode does not use the Expo_kV or Expo_kA configs.  Target
   * position can be changed on-the-fly and Motion Magic® will do its
   * best to adjust the profile.  This control mode is voltage-based, so
   * relevant closed-loop gains will use Volts for the numerator.
   *   <ul>
   *   <li> <b>MotionMagicVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  Users can optionally provide a torque current
   * feedforward.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity, Acceleration, and Jerk
   * value specified via the Motion Magic® configuration values.  This
   * control mode does not use the Expo_kV or Expo_kA configs.  Target
   * position can be changed on-the-fly and Motion Magic® will do its
   * best to adjust the profile.  This control mode is based on torque
   * current, so relevant closed-loop gains will use Amperes for the
   * numerator.
   *   <ul>
   *   <li> <b>MotionMagicTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicTorqueCurrentFOC request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request a specified motor duty cycle with a differential position
   * closed-loop.
   * <p>
   * This control mode will output a proportion of the supplied voltage
   * which is supplied by the user. It will also set the motor's
   * differential position setpoint to the specified position.
   *   <ul>
   *   <li> <b>DifferentialDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetOutput:</b> Proportion of supply voltage to apply in fractional
   *                           units between -1 and +1
   *    <li> <b>DifferentialPosition:</b> Differential position to drive towards in
   *                                   rotations
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request a specified voltage with a differential position
   * closed-loop.
   * <p>
   * This control mode will attempt to apply the specified voltage to
   * the motor. If the supply voltage is below the requested voltage,
   * the motor controller will output the supply voltage. It will also
   * set the motor's differential position setpoint to the specified
   * position.
   *   <ul>
   *   <li> <b>DifferentialVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetOutput:</b> Voltage to attempt to drive at
   *    <li> <b>DifferentialPosition:</b> Differential position to drive towards in
   *                                   rotations
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target position with a differential position
   * setpoint.
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. It will also set the motor's
   * differential position setpoint to the specified position.
   *   <ul>
   *   <li> <b>DifferentialPositionDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetPosition:</b> Average position to drive toward in rotations.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialPositionDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target position with a differential position
   * setpoint
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. It will also set the motor's
   * differential position setpoint to the specified position.
   *   <ul>
   *   <li> <b>DifferentialPositionVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetPosition:</b> Average position to drive toward in rotations.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialPositionVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target velocity with a differential position
   * setpoint.
   * <p>
   * This control mode will set the motor's velocity setpoint to the
   * velocity specified by the user. It will also set the motor's
   * differential position setpoint to the specified position.
   *   <ul>
   *   <li> <b>DifferentialVelocityDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetVelocity:</b> Average velocity to drive toward in rotations per
   *                             second.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVelocityDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request PID to target velocity with a differential position
   * setpoint.
   * <p>
   * This control mode will set the motor's velocity setpoint to the
   * velocity specified by the user. It will also set the motor's
   * differential position setpoint to the specified position.
   *   <ul>
   *   <li> <b>DifferentialVelocityVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetVelocity:</b> Average velocity to drive toward in rotations per
   *                             second.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVelocityVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile, and PID to a differential position setpoint.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity, Acceleration, and Jerk
   * value specified via the Motion Magic® configuration values.  This
   * control mode does not use the Expo_kV or Expo_kA configs.  Target
   * position can be changed on-the-fly and Motion Magic® will do its
   * best to adjust the profile.  This control mode is duty cycle based,
   * so relevant closed-loop gains will use fractional duty cycle for
   * the numerator:  +1.0 represents full forward output.
   *   <ul>
   *   <li> <b>DifferentialMotionMagicDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetPosition:</b> Average position to drive toward in rotations.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialMotionMagicDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile, and PID to a differential position setpoint.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity, Acceleration, and Jerk
   * value specified via the Motion Magic® configuration values.  This
   * control mode does not use the Expo_kV or Expo_kA configs.  Target
   * position can be changed on-the-fly and Motion Magic® will do its
   * best to adjust the profile.  This control mode is voltage-based, so
   * relevant closed-loop gains will use Volts for the numerator.
   *   <ul>
   *   <li> <b>DifferentialMotionMagicVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>TargetPosition:</b> Average position to drive toward in rotations.
   *    <li> <b>DifferentialPosition:</b> Differential position to drive toward in
   *                                   rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>TargetSlot:</b> Select which gains are applied to the primary
   *                         controller by selecting the slot.  Use the
   *                         configuration api to set the gain values for the
   *                         selected slot before enabling this feature. Slot must
   *                         be within [0,2].
   *    <li> <b>DifferentialSlot:</b> Select which gains are applied to the
   *                               differential controller by selecting the slot. 
   *                               Use the configuration api to set the gain
   *                               values for the selected slot before enabling
   *                               this feature. Slot must be within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialMotionMagicVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Follow the motor output of another Talon.
   * <p>
   * If Talon is in torque control, the torque is copied - which will
   * increase the total torque applied. If Talon is in percent supply
   * output control, the duty cycle is matched.  Motor direction either
   * matches master's configured direction or opposes it based on
   * OpposeMasterDirection.
   *   <ul>
   *   <li> <b>Follower Parameters:</b> 
   *    <ul>
   *    <li> <b>MasterID:</b> Device ID of the master to follow.
   *    <li> <b>OpposeMasterDirection:</b> Set to false for motor invert to match the
   *                                    master's configured Invert - which is
   *                                    typical when master and follower are
   *                                    mechanically linked and spin in the same
   *                                    direction.  Set to true for motor invert
   *                                    to oppose the master's configured Invert -
   *                                    this is typical where the the master and
   *                                    follower mechanically spin in opposite
   *                                    directions.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Follower request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Follow the motor output of another Talon while ignoring the
   * master's invert setting.
   * <p>
   * If Talon is in torque control, the torque is copied - which will
   * increase the total torque applied. If Talon is in percent supply
   * output control, the duty cycle is matched.  Motor direction is
   * strictly determined by the configured invert and not the master. 
   * If you want motor direction to match or oppose the master, use
   * FollowerRequest instead.
   *   <ul>
   *   <li> <b>StrictFollower Parameters:</b> 
   *    <ul>
   *    <li> <b>MasterID:</b> Device ID of the master to follow.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(StrictFollower request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Follow the differential motor output of another Talon.
   * <p>
   * If Talon is in torque control, the torque is copied - which will
   * increase the total torque applied. If Talon is in percent supply
   * output control, the duty cycle is matched.  Motor direction either
   * matches master's configured direction or opposes it based on
   * OpposeMasterDirection.
   *   <ul>
   *   <li> <b>DifferentialFollower Parameters:</b> 
   *    <ul>
   *    <li> <b>MasterID:</b> Device ID of the differential master to follow.
   *    <li> <b>OpposeMasterDirection:</b> Set to false for motor invert to match the
   *                                    master's configured Invert - which is
   *                                    typical when master and follower are
   *                                    mechanically linked and spin in the same
   *                                    direction.  Set to true for motor invert
   *                                    to oppose the master's configured Invert -
   *                                    this is typical where the the master and
   *                                    follower mechanically spin in opposite
   *                                    directions.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialFollower request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Follow the differential motor output of another Talon while
   * ignoring the master's invert setting.
   * <p>
   * If Talon is in torque control, the torque is copied - which will
   * increase the total torque applied. If Talon is in percent supply
   * output control, the duty cycle is matched.  Motor direction is
   * strictly determined by the configured invert and not the master. 
   * If you want motor direction to match or oppose the master, use
   * FollowerRequest instead.
   *   <ul>
   *   <li> <b>DifferentialStrictFollower Parameters:</b> 
   *    <ul>
   *    <li> <b>MasterID:</b> Device ID of the differential master to follow.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialStrictFollower request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request neutral output of actuator. The applied brake type is
   * determined by the NeutralMode configuration.
   *   <ul>
   *   <li> <b>NeutralOut Parameters:</b> 
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(NeutralOut request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Request coast neutral output of actuator.  The bridge is disabled
   * and the rotor is allowed to coast.
   *   <ul>
   *   <li> <b>CoastOut Parameters:</b> 
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(CoastOut request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Applies full neutral-brake by shorting motor leads together.
   *   <ul>
   *   <li> <b>StaticBrake Parameters:</b> 
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(StaticBrake request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Plays a single tone at the user specified frequency.
   *   <ul>
   *   <li> <b>MusicTone Parameters:</b> 
   *    <ul>
   *    <li> <b>AudioFrequency:</b> Sound frequency to play.  A value of zero will
   *                             silence the device. The effective frequency range
   *                             is 10-20000Hz.  Any nonzero frequency less than
   *                             10 Hz will be capped to 10Hz.  Any frequency
   *                             above 20Khz will be capped to 20KHz.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MusicTone request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final velocity using a motion
   * profile.  This allows smooth transitions between velocity set
   * points.  Users can optionally provide a duty cycle feedforward.
   * <p>
   * Motion Magic® Velocity produces a motion profile in real-time while
   * attempting to honor the specified Acceleration and Jerk value. This
   * control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA
   * configs. If the specified acceleration is zero, the Acceleration
   * under Motion Magic® configuration parameter is used instead. This
   * allows for runtime adjustment of acceleration for advanced users. 
   * Jerk is also specified in the Motion Magic® persistent
   * configuration values.  If Jerk is set to zero, Motion Magic® will
   * produce a trapezoidal acceleration profile.  Target velocity can
   * also be changed on-the-fly and Motion Magic® will do its best to
   * adjust the profile.  This control mode is duty cycle based, so
   * relevant closed-loop gains will use fractional duty cycle for the
   * numerator:  +1.0 represents full forward output.
   *   <ul>
   *   <li> <b>MotionMagicVelocityDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
   *                       second.  This can be changed on-the fly.
   *    <li> <b>Acceleration:</b> This is the absolute Acceleration to use generating
   *                           the profile.  If this parameter is zero, the
   *                           Acceleration persistent configuration parameter is
   *                           used instead. Acceleration is in rotations per
   *                           second squared.  If nonzero, the signage does not
   *                           matter as the absolute value is used.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final velocity using a motion
   * profile.  This allows smooth transitions between velocity set
   * points.  Users can optionally provide a torque feedforward.
   * <p>
   * Motion Magic® Velocity produces a motion profile in real-time while
   * attempting to honor the specified Acceleration and Jerk value. This
   * control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA
   * configs. If the specified acceleration is zero, the Acceleration
   * under Motion Magic® configuration parameter is used instead. This
   * allows for runtime adjustment of acceleration for advanced users. 
   * Jerk is also specified in the Motion Magic® persistent
   * configuration values.  If Jerk is set to zero, Motion Magic® will
   * produce a trapezoidal acceleration profile.  Target velocity can
   * also be changed on-the-fly and Motion Magic® will do its best to
   * adjust the profile.  This control mode is based on torque current,
   * so relevant closed-loop gains will use Amperes for the numerator.
   *   <ul>
   *   <li> <b>MotionMagicVelocityTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
   *                       second.  This can be changed on-the fly.
   *    <li> <b>Acceleration:</b> This is the absolute Acceleration to use generating
   *                           the profile.  If this parameter is zero, the
   *                           Acceleration persistent configuration parameter is
   *                           used instead. Acceleration is in rotations per
   *                           second squared.  If nonzero, the signage does not
   *                           matter as the absolute value is used.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityTorqueCurrentFOC request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final velocity using a motion
   * profile.  This allows smooth transitions between velocity set
   * points.  Users can optionally provide a voltage feedforward.
   * <p>
   * Motion Magic® Velocity produces a motion profile in real-time while
   * attempting to honor the specified Acceleration and Jerk value. This
   * control mode does not use the CruiseVelocity, Expo_kV, or Expo_kA
   * configs. If the specified acceleration is zero, the Acceleration
   * under Motion Magic® configuration parameter is used instead. This
   * allows for runtime adjustment of acceleration for advanced users. 
   * Jerk is also specified in the Motion Magic® persistent
   * configuration values.  If Jerk is set to zero, Motion Magic® will
   * produce a trapezoidal acceleration profile.  Target velocity can
   * also be changed on-the-fly and Motion Magic® will do its best to
   * adjust the profile.  This control mode is voltage-based, so
   * relevant closed-loop gains will use Volts for the numerator.
   *   <ul>
   *   <li> <b>MotionMagicVelocityVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
   *                       second.  This can be changed on-the fly.
   *    <li> <b>Acceleration:</b> This is the absolute Acceleration to use generating
   *                           the profile.  If this parameter is zero, the
   *                           Acceleration persistent configuration parameter is
   *                           used instead. Acceleration is in rotations per
   *                           second squared.  If nonzero, the signage does not
   *                           matter as the absolute value is used.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using an
   * exponential motion profile.  Users can optionally provide a duty
   * cycle feedforward.
   * <p>
   * Motion Magic® Expo produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity (optional) and the
   * mechanism kV and kA, specified via the Motion Magic® configuration
   * values.  Setting Cruise Velocity to 0 will allow the profile to run
   * to the max possible velocity based on Expo_kV.  This control mode
   * does not use the Acceleration or Jerk configs.  Target position can
   * be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile.  This control mode is duty cycle based, so relevant
   * closed-loop gains will use fractional duty cycle for the numerator:
   *  +1.0 represents full forward output.
   *   <ul>
   *   <li> <b>MotionMagicExpoDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using an
   * exponential motion profile.  Users can optionally provide a voltage
   * feedforward.
   * <p>
   * Motion Magic® Expo produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity (optional) and the
   * mechanism kV and kA, specified via the Motion Magic® configuration
   * values.  Setting Cruise Velocity to 0 will allow the profile to run
   * to the max possible velocity based on Expo_kV.  This control mode
   * does not use the Acceleration or Jerk configs.  Target position can
   * be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile.  This control mode is voltage-based, so relevant
   * closed-loop gains will use Volts for the numerator.
   *   <ul>
   *   <li> <b>MotionMagicExpoVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using an
   * exponential motion profile.  Users can optionally provide a torque
   * current feedforward.
   * <p>
   * Motion Magic® Expo produces a motion profile in real-time while
   * attempting to honor the Cruise Velocity (optional) and the
   * mechanism kV and kA, specified via the Motion Magic® configuration
   * values.  Setting Cruise Velocity to 0 will allow the profile to run
   * to the max possible velocity based on Expo_kV.  This control mode
   * does not use the Acceleration or Jerk configs.  Target position can
   * be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile.  This control mode is based on torque current, so
   * relevant closed-loop gains will use Amperes for the numerator.
   *   <ul>
   *   <li> <b>MotionMagicExpoTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoTorqueCurrentFOC request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  This dynamic request allows runtime changes to Cruise
   * Velocity, Acceleration, and Jerk.  Users can optionally provide a
   * duty cycle feedforward.  This control requires use of a CANivore.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the specified Cruise Velocity, Acceleration,
   * and Jerk value.  This control mode does not use the Expo_kV or
   * Expo_kA configs.  Target position can be changed on-the-fly and
   * Motion Magic® will do its best to adjust the profile. This control
   * mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator:  +1.0 represents full
   * forward output.
   *   <ul>
   *   <li> <b>DynamicMotionMagicDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does not
   *                       matter as the device will use the absolute value for
   *                       profile generation.
   *    <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does not
   *                           matter as the device will use the absolute value
   *                           for profile generation
   *    <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
   *                   device will use the absolute value for profile generation
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicDutyCycle request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  This dynamic request allows runtime changes to Cruise
   * Velocity, Acceleration, and Jerk.  Users can optionally provide a
   * voltage feedforward.  This control requires use of a CANivore.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the specified Cruise Velocity, Acceleration,
   * and Jerk value.  This control mode does not use the Expo_kV or
   * Expo_kA configs.  Target position can be changed on-the-fly and
   * Motion Magic® will do its best to adjust the profile.  This control
   * mode is voltage-based, so relevant closed-loop gains will use Volts
   * for the numerator.
   *   <ul>
   *   <li> <b>DynamicMotionMagicVoltage Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does not
   *                       matter as the device will use the absolute value for
   *                       profile generation.
   *    <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does not
   *                           matter as the device will use the absolute value
   *                           for profile generation.
   *    <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
   *                   device will use the absolute value for profile generation.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in volts
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicVoltage request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Requests Motion Magic® to target a final position using a motion
   * profile.  This dynamic request allows runtime changes to Cruise
   * Velocity, Acceleration, and Jerk.  Users can optionally provide a
   * torque current feedforward.  This control requires use of a
   * CANivore.
   * <p>
   * Motion Magic® produces a motion profile in real-time while
   * attempting to honor the specified Cruise Velocity, Acceleration,
   * and Jerk value.  This control mode does not use the Expo_kV or
   * Expo_kA configs.  Target position can be changed on-the-fly and
   * Motion Magic® will do its best to adjust the profile. This control
   * mode is based on torque current, so relevant closed-loop gains will
   * use Amperes for the numerator.
   *   <ul>
   *   <li> <b>DynamicMotionMagicTorqueCurrentFOC Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does not
   *                       matter as the device will use the absolute value for
   *                       profile generation.
   *    <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does not
   *                           matter as the device will use the absolute value
   *                           for profile generation.
   *    <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
   *                   device will use the absolute value for profile generation.
   *    <li> <b>FeedForward:</b> Feedforward to apply in torque current in Amperes. 
   *                          User can use motor's kT to scale Newton-meter to
   *                          Amperes.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
   *                                      output is zero (or within deadband). 
   *                                      Set to false to use the NeutralMode
   *                                      configuration setting (default). This
   *                                      flag exists to provide the fundamental
   *                                      behavior of this control when output is
   *                                      zero, which is to provide 0A (zero
   *                                      torque).
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicTorqueCurrentFOC request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with duty cycle average target and position
   * difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_DutyCycleOut_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average DutyCycleOut request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_DutyCycleOut_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and position
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionDutyCycle_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionDutyCycle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionDutyCycle_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and position
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityDutyCycle_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityDutyCYcle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityDutyCycle_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and position
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicDutyCycle_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicDutyCycle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicDutyCycle_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with duty cycle average target and velocity
   * difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_DutyCycleOut_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average DutyCycleOut request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_DutyCycleOut_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and velocity
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionDutyCycle_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionDutyCycle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionDutyCycle_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and velocity
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityDutyCycle_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityDutyCycle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityDutyCycle_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and velocity
   * difference target using dutycycle control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicDutyCycle_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicDutyCycle request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request of
   *                                  the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicDutyCycle_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with voltage average target and position
   * difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_VoltageOut_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VoltageOut request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VoltageOut_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and position
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionVoltage_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionVoltage request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionVoltage_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and position
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityVoltage_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityVoltage request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityVoltage_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and position
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicVoltage_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicVoltage request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicVoltage_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with voltage average target and velocity
   * difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_VoltageOut_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VoltageOut request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VoltageOut_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and velocity
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionVoltage_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionVoltage request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionVoltage_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and velocity
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityVoltage_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityVoltage request of the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityVoltage_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and velocity
   * difference target using voltage control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicVoltage_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicVoltage request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of the
   *                                  mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicVoltage_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with torque current average target and
   * position difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_TorqueCurrentFOC_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average TorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_TorqueCurrentFOC_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and position
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionTorqueCurrentFOC_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionTorqueCurrentFOC_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and position
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityTorqueCurrentFOC_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and position
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicTorqueCurrentFOC_Position Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request of
   *                             the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Position request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with torque current average target and
   * velocity difference target.
   * 
   *   <ul>
   *   <li> <b>Diff_TorqueCurrentFOC_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average TorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_TorqueCurrentFOC_Velocity request)
  {

    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with position average target and velocity
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_PositionTorqueCurrentFOC_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionTorqueCurrentFOC_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with velocity average target and velocity
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_VelocityTorqueCurrentFOC_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of the
   *                             mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Velocity request)
  {
    return m_talon.setControl(request);
  }
  
  /**
   * Differential control with Motion Magic® average target and velocity
   * difference target using torque current control.
   * 
   *   <ul>
   *   <li> <b>Diff_MotionMagicTorqueCurrentFOC_Velocity Parameters:</b> 
   *    <ul>
   *    <li> <b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request of
   *                             the mechanism.
   *    <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
   *                                  request of the mechanism.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Velocity request)
  {
    return m_talon.setControl(request);
  }

  /**
  * Closes the TalonFX motor controller
  */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_talon.close();
  }
}
