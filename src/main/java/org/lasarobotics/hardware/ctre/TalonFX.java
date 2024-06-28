// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package org.lasarobotics.hardware.ctre;


import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;


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
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MOTOR_DEADBAND = 0.01;


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

  //Different slot types
  private enum SlotTypes {Slot0, Slot1, Slot2}

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
  private void logOutputs(ControlMode mode, double value) {
    Logger.recordOutput(m_id.name + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.toString());
    Logger.recordOutput(m_id.name + CURRENT_LOG_ENTRY, m_talon.getStatorCurrent().getValue());
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
   * Initialize supply voltage time constant
   * 
   * @param supplyVoltageTimeConstant 
   * The time constant (in seconds) of the low-pass filter for the
   * supply voltage.
   * <p>
   * This impacts the filtering for the reported supply voltage, and any
   * control strategies that use the supply voltage (such as voltage
   * control on a motor controller).
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 0.1
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   */
  public void initializeSupplyVoltageTimeConstant(double supplyVoltageTimeConstant) {
    VoltageConfigs config = m_TalonFXConfiguration.Voltage;
      config.SupplyVoltageTimeConstant = supplyVoltageTimeConstant;

    m_talon.getConfigurator().apply(config);
  }

  /**
   * Initialize peak voltage
   * @param peakReverseVoltage 
   * Minimum (reverse) output during voltage based control modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> -16
   *   <li> <b>Maximum Value:</b> 16
   *   <li> <b>Default Value:</b> -16
   *   <li> <b>Units:</b> V
   *   </ul>
   * @param peakForwardVoltage
   * Maximum (forward) output during voltage based control modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> -16
   *   <li> <b>Maximum Value:</b> 16
   *   <li> <b>Default Value:</b> 16
   *   <li> <b>Units:</b> V
   *   </ul>
   */
  public void initializePeakVoltage(double peakReverseVoltage, double peakForwardVoltage) {
    VoltageConfigs config = m_TalonFXConfiguration.Voltage;
      config.PeakReverseVoltage = peakReverseVoltage;
      config.PeakForwardVoltage = peakForwardVoltage;

    m_talon.getConfigurator().apply(config);
  }

  /**
   * Initialize peak torque currents
   * @param peakReverseTorqueCurrent
   * Minimum (reverse) output during torque current based control modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> -800
   *   <li> <b>Maximum Value:</b> 800
   *   <li> <b>Default Value:</b> -800
   *   <li> <b>Units:</b> A
   *   </ul>
   * @param peakForwardTorqueCurrent
   * Maximum (forward) output during torque current based control modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> -800
   *   <li> <b>Maximum Value:</b> 800
   *   <li> <b>Default Value:</b> 800
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializePeakTorqueCurrents(double peakReverseTorqueCurrent, 
                                           double peakForwardTorqueCurrent) {
    TorqueCurrentConfigs config = m_TalonFXConfiguration.TorqueCurrent;
      config.PeakReverseTorqueCurrent = peakReverseTorqueCurrent;
      config.PeakForwardTorqueCurrent = peakForwardTorqueCurrent;

    m_talon.getConfigurator().apply(config);
  }

  /**
   * Initialize torque neutral deadband
   * @param torqueNeutralDeadband
   * Configures the output deadband during torque current based control
   * modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0
   *   <li> <b>Maximum Value:</b> 25
   *   <li> <b>Default Value:</b> 0.0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeTorqueNeutralDeadband(double torqueNeutralDeadband) {
    TorqueCurrentConfigs configs = m_TalonFXConfiguration.TorqueCurrent;
      configs.TorqueNeutralDeadband = torqueNeutralDeadband;
    
    m_talon.getConfigurator().apply(configs);
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
    FeedbackConfigs feedbackConfigs = m_TalonFXConfiguration.Feedback;

    //Automatically configure feedback sensor to built in rotor sensor
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    switch (sensor) {
      case REMOTE:
       feedbackConfigs.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED:
       feedbackConfigs.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC:
       feedbackConfigs.FeedbackRemoteSensorID = cancoder.getID().deviceID;
       feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
      }
      m_talon.getConfigurator().apply(feedbackConfigs);
    }
    
  /**
   * Initialize open-loop period times
   * @param dutyCycleOpenLoopRampPeriod
   * If non-zero, this determines how much time to ramp from 0% output
   * to 100% during open-loop modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0
   *   <li> <b>Maximum Value:</b> 1
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   * @param voltageOpenLoopRampPeriod
   * If non-zero, this determines how much time to ramp from 0V output
   * to 12V during open-loop modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0
   *   <li> <b>Maximum Value:</b> 1
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   * @param torqueOpenLoopRampPeriod
   * If non-zero, this determines how much time to ramp from 0V output
   * to 12V during open-loop modes.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0
   *   <li> <b>Maximum Value:</b> 1
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   */
  public void initializeOpenLoopRampPeriods(double dutyCycleOpenLoopRampPeriod,
                                            double voltageOpenLoopRampPeriod,
                                            double torqueOpenLoopRampPeriod) {
    OpenLoopRampsConfigs openloopsrampsconfigs = m_TalonFXConfiguration.OpenLoopRamps;
      if (dutyCycleOpenLoopRampPeriod != 0) {
        openloopsrampsconfigs.DutyCycleOpenLoopRampPeriod = dutyCycleOpenLoopRampPeriod;
      }          
      if (voltageOpenLoopRampPeriod != 0) {
        openloopsrampsconfigs.VoltageOpenLoopRampPeriod = voltageOpenLoopRampPeriod;
      }     
      if (torqueOpenLoopRampPeriod != 0) {
        openloopsrampsconfigs.TorqueOpenLoopRampPeriod = torqueOpenLoopRampPeriod;
      }      
    
    m_talon.getConfigurator().apply(openloopsrampsconfigs);
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
   * Initialize software limit switches for TalonFX
   * @param forward Boolean to choose whether to init forward soft limit switch
   * @param reverse Boolean to choose whether to init reverse soft limit switch
   * @param forwardSoftLimitThreshold Double for threshold of forward soft limit
   * @param reverseSoftLimitThreshold Double for threshold of reverse soft limit
   */
  public void initializeSoftLimits(boolean forward, 
                                  boolean reverse, 
                                  double forwardSoftLimitThreshold, 
                                  double reverseSoftLimitThreshold) {

    SoftwareLimitSwitchConfigs softLimitConfigs = m_TalonFXConfiguration.SoftwareLimitSwitch;
     if (forward) {
       softLimitConfigs.ForwardSoftLimitEnable = true;
       softLimitConfigs.ForwardSoftLimitThreshold = forwardSoftLimitThreshold;
      }
     if (reverse) {
       softLimitConfigs.ReverseSoftLimitEnable = true;
       softLimitConfigs.ReverseSoftLimitThreshold = reverseSoftLimitThreshold;
      }
    m_talon.getConfigurator().apply(softLimitConfigs);
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
   * Initialize Talon PID Configuration and Motion Magic
   * @param pidconfig PID Config to use
   * @param gravityType Type of gravity to use
   * @param staticffsign Static feedforward sign to use
   */
  private void initializeTalonPID(TalonPIDConfig pidconfig,
                                 GravityTypeValue gravityType,
                                 StaticFeedforwardSignValue staticffsign,
                                 SlotTypes slotTypes) {
    //Initialize PID configs
    m_TalonPIDConfig = pidconfig;

    Slot0Configs slot0Configs = m_TalonFXConfiguration.Slot0;
    Slot1Configs slot1Configs = m_TalonFXConfiguration.Slot1;
    Slot2Configs slot2Configs = m_TalonFXConfiguration.Slot2;

    //Configure PID Values
    switch (slotTypes) {
     case Slot0:
      slot0Configs.kP = m_TalonPIDConfig.getkP();
      slot0Configs.kI = m_TalonPIDConfig.getkI();
      slot0Configs.kD = m_TalonPIDConfig.getkD();
      slot0Configs.kS = m_TalonPIDConfig.getkS();
      slot0Configs.kV = m_TalonPIDConfig.getkV();
      slot0Configs.kG = m_TalonPIDConfig.getkG();
      slot0Configs.kA = m_TalonPIDConfig.getkA();
     case Slot1:
      slot1Configs.kP = m_TalonPIDConfig.getkP();
      slot1Configs.kI = m_TalonPIDConfig.getkI();
      slot1Configs.kD = m_TalonPIDConfig.getkD();
      slot1Configs.kS = m_TalonPIDConfig.getkS();
      slot1Configs.kV = m_TalonPIDConfig.getkV();
      slot1Configs.kG = m_TalonPIDConfig.getkG();
      slot1Configs.kA = m_TalonPIDConfig.getkA();
     case Slot2:
      slot2Configs.kP = m_TalonPIDConfig.getkP();
      slot2Configs.kI = m_TalonPIDConfig.getkI();
      slot2Configs.kD = m_TalonPIDConfig.getkD();
      slot2Configs.kS = m_TalonPIDConfig.getkS();
      slot2Configs.kV = m_TalonPIDConfig.getkV();
      slot2Configs.kG = m_TalonPIDConfig.getkG();
      slot2Configs.kA = m_TalonPIDConfig.getkA();
    }

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
    switch (gravityType) {
      case Arm_Cosine:
       switch (slotTypes) {
        case Slot0:
         slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        case Slot1:
         slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;
        case Slot2:
         slot2Configs.GravityType = GravityTypeValue.Arm_Cosine;
    }
      case Elevator_Static:
       switch (slotTypes) {
        case Slot0:
         slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        case Slot1:
         slot1Configs.GravityType = GravityTypeValue.Elevator_Static;
        case Slot2:
         slot2Configs.GravityType = GravityTypeValue.Elevator_Static;
    }

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
    switch (staticffsign) {
      case UseClosedLoopSign:
       slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      case UseVelocitySign:
       slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign; 
    }  

    //Configure MotionMagic
    MotionMagicConfigs m_motionMagic = m_TalonFXConfiguration.MotionMagic;
    if (m_TalonPIDConfig.getMotionMagic()) {
     /**
     * This is the maximum velocity Motion Magic based control modes are
     * allowed to use.  Motion Magic® Velocity control modes do not use
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
     *   <li> <b>Units:</b> rot per sec²
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
     *   <li> <b>Units:</b> rot per sec³
     *   </ul>
     */
     m_motionMagic.MotionMagicJerk = m_TalonPIDConfig.getMotionMagicJerk();
    }
   }

   if (pidconfig.getInvertMotor()) {
     
   }
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
