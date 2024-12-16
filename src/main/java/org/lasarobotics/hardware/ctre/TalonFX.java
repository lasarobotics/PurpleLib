// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package org.lasarobotics.hardware.ctre;


import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  public StatusCode applyConfigs(TalonFXConfiguration configs) {
    return m_talon.getConfigurator().apply(configs);
  }

  /**
     * Control motor with generic control request object.
     * <p>
     * User must make sure the specified object is castable to a valid control request,
     * otherwise this function will fail at run-time and return the NotSupported StatusCode
     *
     * @param request Control object to request of the device
     * @return Status Code of the request, 0 is OK
     */
    public StatusCode setControl(ControlRequest request)
    {
        if (request instanceof DutyCycleOut)
            return m_talon.setControl((DutyCycleOut)request);
        if (request instanceof TorqueCurrentFOC)
            return m_talon.setControl((TorqueCurrentFOC)request);
        if (request instanceof VoltageOut)
            return m_talon.setControl((VoltageOut)request);
        if (request instanceof PositionDutyCycle)
            return m_talon.setControl((PositionDutyCycle)request);
        if (request instanceof PositionVoltage)
            return m_talon.setControl((PositionVoltage)request);
        if (request instanceof PositionTorqueCurrentFOC)
            return m_talon.setControl((PositionTorqueCurrentFOC)request);
        if (request instanceof VelocityDutyCycle)
            return m_talon.setControl((VelocityDutyCycle)request);
        if (request instanceof VelocityVoltage)
            return m_talon.setControl((VelocityVoltage)request);
        if (request instanceof VelocityTorqueCurrentFOC)
            return m_talon.setControl((VelocityTorqueCurrentFOC)request);
        if (request instanceof MotionMagicDutyCycle)
            return m_talon.setControl((MotionMagicDutyCycle)request);
        if (request instanceof MotionMagicVoltage)
            return m_talon.setControl((MotionMagicVoltage)request);
        if (request instanceof MotionMagicTorqueCurrentFOC)
            return m_talon.setControl((MotionMagicTorqueCurrentFOC)request);
        if (request instanceof DifferentialDutyCycle)
            return m_talon.setControl((DifferentialDutyCycle)request);
        if (request instanceof DifferentialVoltage)
            return m_talon.setControl((DifferentialVoltage)request);
        if (request instanceof DifferentialPositionDutyCycle)
            return m_talon.setControl((DifferentialPositionDutyCycle)request);
        if (request instanceof DifferentialPositionVoltage)
            return m_talon.setControl((DifferentialPositionVoltage)request);
        if (request instanceof DifferentialVelocityDutyCycle)
            return m_talon.setControl((DifferentialVelocityDutyCycle)request);
        if (request instanceof DifferentialVelocityVoltage)
            return m_talon.setControl((DifferentialVelocityVoltage)request);
        if (request instanceof DifferentialMotionMagicDutyCycle)
            return m_talon.setControl((DifferentialMotionMagicDutyCycle)request);
        if (request instanceof DifferentialMotionMagicVoltage)
            return m_talon.setControl((DifferentialMotionMagicVoltage)request);
        if (request instanceof Follower)
            return m_talon.setControl((Follower)request);
        if (request instanceof StrictFollower)
            return m_talon.setControl((StrictFollower)request);
        if (request instanceof DifferentialFollower)
            return m_talon.setControl((DifferentialFollower)request);
        if (request instanceof DifferentialStrictFollower)
            return m_talon.setControl((DifferentialStrictFollower)request);
        if (request instanceof NeutralOut)
            return m_talon.setControl((NeutralOut)request);
        if (request instanceof CoastOut)
            return m_talon.setControl((CoastOut)request);
        if (request instanceof StaticBrake)
            return m_talon.setControl((StaticBrake)request);
        if (request instanceof MusicTone)
            return m_talon.setControl((MusicTone)request);
        if (request instanceof MotionMagicVelocityDutyCycle)
            return m_talon.setControl((MotionMagicVelocityDutyCycle)request);
        if (request instanceof MotionMagicVelocityTorqueCurrentFOC)
            return m_talon.setControl((MotionMagicVelocityTorqueCurrentFOC)request);
        if (request instanceof MotionMagicVelocityVoltage)
            return m_talon.setControl((MotionMagicVelocityVoltage)request);
        if (request instanceof MotionMagicExpoDutyCycle)
            return m_talon.setControl((MotionMagicExpoDutyCycle)request);
        if (request instanceof MotionMagicExpoVoltage)
            return m_talon.setControl((MotionMagicExpoVoltage)request);
        if (request instanceof MotionMagicExpoTorqueCurrentFOC)
            return m_talon.setControl((MotionMagicExpoTorqueCurrentFOC)request);
        if (request instanceof DynamicMotionMagicDutyCycle)
            return m_talon.setControl((DynamicMotionMagicDutyCycle)request);
        if (request instanceof DynamicMotionMagicVoltage)
            return m_talon.setControl((DynamicMotionMagicVoltage)request);
        if (request instanceof DynamicMotionMagicTorqueCurrentFOC)
            return m_talon.setControl((DynamicMotionMagicTorqueCurrentFOC)request);
        if (request instanceof Diff_DutyCycleOut_Position)
            return m_talon.setControl((Diff_DutyCycleOut_Position)request);
        if (request instanceof Diff_PositionDutyCycle_Position)
            return m_talon.setControl((Diff_PositionDutyCycle_Position)request);
        if (request instanceof Diff_VelocityDutyCycle_Position)
            return m_talon.setControl((Diff_VelocityDutyCycle_Position)request);
        if (request instanceof Diff_MotionMagicDutyCycle_Position)
            return m_talon.setControl((Diff_MotionMagicDutyCycle_Position)request);
        if (request instanceof Diff_DutyCycleOut_Velocity)
            return m_talon.setControl((Diff_DutyCycleOut_Velocity)request);
        if (request instanceof Diff_PositionDutyCycle_Velocity)
            return m_talon.setControl((Diff_PositionDutyCycle_Velocity)request);
        if (request instanceof Diff_VelocityDutyCycle_Velocity)
            return m_talon.setControl((Diff_VelocityDutyCycle_Velocity)request);
        if (request instanceof Diff_MotionMagicDutyCycle_Velocity)
            return m_talon.setControl((Diff_MotionMagicDutyCycle_Velocity)request);
        if (request instanceof Diff_VoltageOut_Position)
            return m_talon.setControl((Diff_VoltageOut_Position)request);
        if (request instanceof Diff_PositionVoltage_Position)
            return m_talon.setControl((Diff_PositionVoltage_Position)request);
        if (request instanceof Diff_VelocityVoltage_Position)
            return m_talon.setControl((Diff_VelocityVoltage_Position)request);
        if (request instanceof Diff_MotionMagicVoltage_Position)
            return m_talon.setControl((Diff_MotionMagicVoltage_Position)request);
        if (request instanceof Diff_VoltageOut_Velocity)
            return m_talon.setControl((Diff_VoltageOut_Velocity)request);
        if (request instanceof Diff_PositionVoltage_Velocity)
            return m_talon.setControl((Diff_PositionVoltage_Velocity)request);
        if (request instanceof Diff_VelocityVoltage_Velocity)
            return m_talon.setControl((Diff_VelocityVoltage_Velocity)request);
        if (request instanceof Diff_MotionMagicVoltage_Velocity)
            return m_talon.setControl((Diff_MotionMagicVoltage_Velocity)request);
        if (request instanceof Diff_TorqueCurrentFOC_Position)
            return m_talon.setControl((Diff_TorqueCurrentFOC_Position)request);
        if (request instanceof Diff_PositionTorqueCurrentFOC_Position)
            return m_talon.setControl((Diff_PositionTorqueCurrentFOC_Position)request);
        if (request instanceof Diff_VelocityTorqueCurrentFOC_Position)
            return m_talon.setControl((Diff_VelocityTorqueCurrentFOC_Position)request);
        if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Position)
            return m_talon.setControl((Diff_MotionMagicTorqueCurrentFOC_Position)request);
        if (request instanceof Diff_TorqueCurrentFOC_Velocity)
            return m_talon.setControl((Diff_TorqueCurrentFOC_Velocity)request);
        if (request instanceof Diff_PositionTorqueCurrentFOC_Velocity)
            return m_talon.setControl((Diff_PositionTorqueCurrentFOC_Velocity)request);
        if (request instanceof Diff_VelocityTorqueCurrentFOC_Velocity)
            return m_talon.setControl((Diff_VelocityTorqueCurrentFOC_Velocity)request);
        if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Velocity)
            return m_talon.setControl((Diff_MotionMagicTorqueCurrentFOC_Velocity)request);
        return StatusCode.NotSupported;
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
