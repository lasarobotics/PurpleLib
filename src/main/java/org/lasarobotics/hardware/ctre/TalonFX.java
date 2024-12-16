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

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;


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

  private static final String VALUE1_LOG_ENTRY = "/OutputValue1";
  private static final String VALUE2_LOG_ENTRY = "/OutputValue2";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String SUPPLY_CURRENT_LOG_ENTRY = "/SupplyCurrent";
  private static final String STATOR_CURRENT_LOG_ENTRY = "/StatorCurrent";

  private com.ctre.phoenix6.hardware.TalonFX m_talon;

  private ID m_id;
  private Notifier m_inputThread;
  private Measure<Time> m_inputThreadPeriod;
  private volatile TalonFXInputsAutoLogged m_inputs;


  /**
   * Create a TalonFX object with built-in logging
   * @param id TalonFX ID
   */
  public TalonFX(TalonFX.ID id, Measure<Time> inputThreadPeriod) {
    this.m_id = id;
    this.m_talon = new com.ctre.phoenix6.hardware.TalonFX(id.deviceID);
    this.m_inputs = new TalonFXInputsAutoLogged();
    this.m_inputThread = new Notifier(this::updateInputs);
    this.m_inputThreadPeriod = inputThreadPeriod;



    // Disable motor safety
    m_talon.setSafetyEnabled(false);

    // Update inputs on init
    updateInputs();
    periodic();

    // Start sensor input thread
    m_inputThread.setName(m_id.name);
    if (!Logger.hasReplaySource()) m_inputThread.startPeriodic(m_inputThreadPeriod.in(Units.Seconds));
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlRequest mode, double value1, double value2) {
    Logger.recordOutput(m_id.name + VALUE1_LOG_ENTRY, value1);
    Logger.recordOutput(m_id.name + VALUE2_LOG_ENTRY, value2);
    Logger.recordOutput(m_id.name + MODE_LOG_ENTRY, mode.getName());
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

  /**
   * Update sensor input readings
   */
  private void updateInputs() {
    synchronized (m_inputs) {
      m_inputs.selectedSensorPosition = getSelectedSensorPosition();
      m_inputs.selectedSensorVelocity = getSelectedSensorVelocity();
    }
  }

  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_id.name, m_inputs);

    Logger.recordOutput(m_id.name + SUPPLY_CURRENT_LOG_ENTRY, m_talon.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(m_id.name + STATOR_CURRENT_LOG_ENTRY, m_talon.getStatorCurrent().getValueAsDouble());
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public TalonFXInputsAutoLogged getInputs() {
    synchronized (m_inputs) { return m_inputs; }
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
  public StatusCode setControl(ControlRequest request) {

    if (request instanceof DutyCycleOut) {
      logOutputs(request, ((DutyCycleOut)request).Output, 0.0);
      return m_talon.setControl((DutyCycleOut)request);
    }
    if (request instanceof TorqueCurrentFOC) {
      logOutputs(request, ((TorqueCurrentFOC)request).Output, 0.0);
      return m_talon.setControl((TorqueCurrentFOC)request);
    }
    if (request instanceof VoltageOut) {
      logOutputs(request, ((VoltageOut)request).Output, 0.0);
      return m_talon.setControl((VoltageOut)request);
    }
    if (request instanceof PositionDutyCycle) {
      logOutputs(request, ((PositionDutyCycle)request).Position, 0.0);
      return m_talon.setControl((PositionDutyCycle)request);
    }
    if (request instanceof PositionVoltage) {
      logOutputs(request, ((PositionVoltage)request).Position, 0.0);
      return m_talon.setControl((PositionVoltage)request);
    }
    if (request instanceof PositionTorqueCurrentFOC) {
      logOutputs(request, ((PositionTorqueCurrentFOC)request).Position, 0.0);
      return m_talon.setControl((PositionTorqueCurrentFOC)request);
    }
    if (request instanceof VelocityDutyCycle) {
      logOutputs(request, ((VelocityDutyCycle)request).Velocity, 0.0);
      return m_talon.setControl((VelocityDutyCycle)request);
    }
    if (request instanceof VelocityVoltage) {
      logOutputs(request, ((VelocityVoltage)request).Velocity, 0.0);
      return m_talon.setControl((VelocityVoltage)request);
    }
    if (request instanceof VelocityTorqueCurrentFOC) {
      logOutputs(request, ((VelocityTorqueCurrentFOC)request).Velocity, 0.0);
      return m_talon.setControl((VelocityTorqueCurrentFOC)request);
    }
    if (request instanceof MotionMagicDutyCycle) {
      logOutputs(request, ((MotionMagicDutyCycle)request).Position, 0.0);
      return m_talon.setControl((MotionMagicDutyCycle)request);
    }
    if (request instanceof MotionMagicVoltage) {
      logOutputs(request, ((MotionMagicVoltage)request).Position, 0.0);
      return m_talon.setControl((MotionMagicVoltage)request);
    }
    if (request instanceof MotionMagicTorqueCurrentFOC) {
      logOutputs(request, ((MotionMagicTorqueCurrentFOC)request).Position, 0.0);
      return m_talon.setControl((MotionMagicTorqueCurrentFOC)request);
    }
    if (request instanceof DifferentialDutyCycle) {
      logOutputs(request, ((DifferentialDutyCycle)request).TargetOutput, 0.0);
      return m_talon.setControl((DifferentialDutyCycle)request);
    }
    if (request instanceof DifferentialVoltage) {
      logOutputs(request, ((DifferentialVoltage)request).TargetOutput, 0.0);
      return m_talon.setControl((DifferentialVoltage)request);
    }
    if (request instanceof DifferentialPositionDutyCycle) {
      logOutputs(request, ((DifferentialPositionDutyCycle)request).TargetPosition, 0.0);
      return m_talon.setControl((DifferentialPositionDutyCycle)request);
    }
    if (request instanceof DifferentialPositionVoltage) {
      logOutputs(request, ((DifferentialPositionVoltage)request).TargetPosition, 0.0);
      return m_talon.setControl((DifferentialPositionVoltage)request);
    }
    if (request instanceof DifferentialVelocityDutyCycle) {
      logOutputs(request, ((DifferentialVelocityDutyCycle)request).TargetVelocity, 0.0);
      return m_talon.setControl((DifferentialVelocityDutyCycle)request);
    }
    if (request instanceof DifferentialVelocityVoltage) {
      logOutputs(request, ((DifferentialVelocityVoltage)request).TargetVelocity, 0.0);
      return m_talon.setControl((DifferentialVelocityVoltage)request);
    }
    if (request instanceof DifferentialMotionMagicDutyCycle) {
      logOutputs(request, ((DifferentialMotionMagicDutyCycle)request).TargetPosition, 0.0);
      return m_talon.setControl((DifferentialMotionMagicDutyCycle)request);
    }
    if (request instanceof DifferentialMotionMagicVoltage) {
      logOutputs(request, ((DifferentialMotionMagicVoltage)request).TargetPosition, 0.0);
      return m_talon.setControl((DifferentialMotionMagicVoltage)request);
    }
    if (request instanceof Follower) {
      logOutputs(request, ((Follower)request).MasterID, 0.0);
      return m_talon.setControl((Follower)request);
    }
    if (request instanceof StrictFollower) {
      logOutputs(request, ((StrictFollower)request).MasterID, 0.0);
      return m_talon.setControl((StrictFollower)request);
    }
    if (request instanceof DifferentialFollower) {
      logOutputs(request, ((DifferentialFollower)request).MasterID, 0.0);
      return m_talon.setControl((DifferentialFollower)request);
    }
    if (request instanceof DifferentialStrictFollower) {
      logOutputs(request, ((DifferentialStrictFollower)request).MasterID, 0.0);
      return m_talon.setControl((DifferentialStrictFollower)request);
    }
    if (request instanceof NeutralOut) {
      logOutputs(request, 0.0, 0.0);
      return m_talon.setControl((NeutralOut)request);
    }
    if (request instanceof CoastOut) {
      logOutputs(request, 0.0, 0.0);
      return m_talon.setControl((CoastOut)request);
    }
    if (request instanceof StaticBrake) {
      logOutputs(request, 0.0, 0.0);
      return m_talon.setControl((StaticBrake)request);
    }
    if (request instanceof MusicTone) {
      logOutputs(request, ((MusicTone)request).AudioFrequency, 0.0);
      return m_talon.setControl((MusicTone)request);
    }
    if (request instanceof MotionMagicVelocityDutyCycle) {
      logOutputs(request, ((MotionMagicVelocityDutyCycle)request).Velocity, 0.0);
      return m_talon.setControl((MotionMagicVelocityDutyCycle)request);
    }
    if (request instanceof MotionMagicVelocityTorqueCurrentFOC) {
      logOutputs(request, ((MotionMagicVelocityDutyCycle)request).Velocity, 0.0);
      return m_talon.setControl((MotionMagicVelocityTorqueCurrentFOC)request);
    }
    if (request instanceof MotionMagicVelocityVoltage) {
      logOutputs(request, ((MotionMagicVelocityVoltage)request).Velocity, 0.0);
      return m_talon.setControl((MotionMagicVelocityVoltage)request);
    }
    if (request instanceof MotionMagicExpoDutyCycle) {
      logOutputs(request, ((MotionMagicExpoDutyCycle)request).Position, 0.0);
      return m_talon.setControl((MotionMagicExpoDutyCycle)request);
    }
    if (request instanceof MotionMagicExpoVoltage) {
      logOutputs(request, ((MotionMagicExpoVoltage)request).Position, 0.0);
      return m_talon.setControl((MotionMagicExpoVoltage)request);
    }
    if (request instanceof MotionMagicExpoTorqueCurrentFOC) {
      logOutputs(request, ((MotionMagicExpoTorqueCurrentFOC)request).Position, 0.0);
      return m_talon.setControl((MotionMagicExpoTorqueCurrentFOC)request);
    }
    if (request instanceof DynamicMotionMagicDutyCycle) {
      logOutputs(request, ((DynamicMotionMagicDutyCycle)request).Position, 0.0);
      return m_talon.setControl((DynamicMotionMagicDutyCycle)request);
    }
    if (request instanceof DynamicMotionMagicVoltage) {
      logOutputs(request, ((DynamicMotionMagicVoltage)request).Position, 0.0);
      return m_talon.setControl((DynamicMotionMagicVoltage)request);
    }
    if (request instanceof DynamicMotionMagicTorqueCurrentFOC) {
      logOutputs(request, ((DynamicMotionMagicTorqueCurrentFOC)request).Position, 0.0);
      return m_talon.setControl((DynamicMotionMagicTorqueCurrentFOC)request);
    }
    if (request instanceof Diff_DutyCycleOut_Position) {
      logOutputs(
        request,
        ((Diff_DutyCycleOut_Position)request).AverageRequest.Output,
        ((Diff_DutyCycleOut_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_DutyCycleOut_Position)request);
    }
    if (request instanceof Diff_PositionDutyCycle_Position) {
      logOutputs(
        request,
        ((Diff_PositionDutyCycle_Position)request).AverageRequest.Position,
        ((Diff_PositionDutyCycle_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_PositionDutyCycle_Position)request);
    }
    if (request instanceof Diff_VelocityDutyCycle_Position) {
      logOutputs(
        request,
        ((Diff_VelocityDutyCycle_Position)request).AverageRequest.Velocity,
        ((Diff_VelocityDutyCycle_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_VelocityDutyCycle_Position)request);
    }
    if (request instanceof Diff_MotionMagicDutyCycle_Position) {
      logOutputs(
        request,
        ((Diff_MotionMagicDutyCycle_Position)request).AverageRequest.Position,
        ((Diff_MotionMagicDutyCycle_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_MotionMagicDutyCycle_Position)request);
    }
    if (request instanceof Diff_DutyCycleOut_Velocity) {
      logOutputs(
        request,
        ((Diff_DutyCycleOut_Velocity)request).AverageRequest.Output,
        ((Diff_DutyCycleOut_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_DutyCycleOut_Velocity)request);
    }
    if (request instanceof Diff_PositionDutyCycle_Velocity) {
      logOutputs(
        request,
        ((Diff_PositionDutyCycle_Velocity)request).AverageRequest.Position,
        ((Diff_PositionDutyCycle_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_PositionDutyCycle_Velocity)request);
    }
    if (request instanceof Diff_VelocityDutyCycle_Velocity) {
      logOutputs(
        request,
        ((Diff_VelocityDutyCycle_Velocity)request).AverageRequest.Velocity,
        ((Diff_VelocityDutyCycle_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_VelocityDutyCycle_Velocity)request);
    }
    if (request instanceof Diff_MotionMagicDutyCycle_Velocity) {
      logOutputs(
        request,
        ((Diff_MotionMagicDutyCycle_Velocity)request).AverageRequest.Position,
        ((Diff_MotionMagicDutyCycle_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_MotionMagicDutyCycle_Velocity)request);
    }
    if (request instanceof Diff_VoltageOut_Position) {
      logOutputs(
        request,
        ((Diff_VoltageOut_Position)request).AverageRequest.Output,
        ((Diff_VoltageOut_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_VoltageOut_Position)request);
    }
    if (request instanceof Diff_PositionVoltage_Position) {
      logOutputs(
        request,
        ((Diff_PositionVoltage_Position)request).AverageRequest.Position,
        ((Diff_PositionVoltage_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_PositionVoltage_Position)request);
    }
    if (request instanceof Diff_VelocityVoltage_Position) {
      logOutputs(
        request,
        ((Diff_VelocityVoltage_Position)request).AverageRequest.Velocity,
        ((Diff_VelocityVoltage_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_VelocityVoltage_Position)request);
    }
    if (request instanceof Diff_MotionMagicVoltage_Position) {
      logOutputs(
        request,
        ((Diff_MotionMagicVoltage_Position)request).AverageRequest.Position,
        ((Diff_MotionMagicVoltage_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_MotionMagicVoltage_Position)request);
    }
    if (request instanceof Diff_VoltageOut_Velocity) {
      logOutputs(
        request,
        ((Diff_VoltageOut_Velocity)request).AverageRequest.Output,
        ((Diff_VoltageOut_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_VoltageOut_Velocity)request);
    }
    if (request instanceof Diff_PositionVoltage_Velocity) {
      logOutputs(
        request,
        ((Diff_PositionVoltage_Velocity)request).AverageRequest.Position,
        ((Diff_PositionVoltage_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_PositionVoltage_Velocity)request);
    }
    if (request instanceof Diff_VelocityVoltage_Velocity) {
      logOutputs(
        request,
        ((Diff_VelocityVoltage_Velocity)request).AverageRequest.Velocity,
        ((Diff_VelocityVoltage_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_VelocityVoltage_Velocity)request);
    }
    if (request instanceof Diff_MotionMagicVoltage_Velocity) {
      logOutputs(
        request,
        ((Diff_MotionMagicDutyCycle_Velocity)request).AverageRequest.Position,
        ((Diff_MotionMagicDutyCycle_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_MotionMagicVoltage_Velocity)request);
    }
    if (request instanceof Diff_TorqueCurrentFOC_Position) {
      logOutputs(
        request,
        ((Diff_TorqueCurrentFOC_Position)request).AverageRequest.Output,
        ((Diff_TorqueCurrentFOC_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_TorqueCurrentFOC_Position)request);
    }
    if (request instanceof Diff_PositionTorqueCurrentFOC_Position) {
      logOutputs(
        request,
        ((Diff_PositionTorqueCurrentFOC_Position)request).AverageRequest.Position,
        ((Diff_PositionTorqueCurrentFOC_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_PositionTorqueCurrentFOC_Position)request);
    }
    if (request instanceof Diff_VelocityTorqueCurrentFOC_Position) {
      logOutputs(
        request,
        ((Diff_VelocityTorqueCurrentFOC_Position)request).AverageRequest.Velocity,
        ((Diff_VelocityTorqueCurrentFOC_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_VelocityTorqueCurrentFOC_Position)request);
    }
    if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Position) {
      logOutputs(
        request,
        ((Diff_MotionMagicTorqueCurrentFOC_Position)request).AverageRequest.Position,
        ((Diff_MotionMagicTorqueCurrentFOC_Position)request).DifferentialRequest.Position
      );
      return m_talon.setControl((Diff_MotionMagicTorqueCurrentFOC_Position)request);
    }
    if (request instanceof Diff_TorqueCurrentFOC_Velocity) {
      logOutputs(
        request,
        ((Diff_TorqueCurrentFOC_Velocity)request).AverageRequest.Output,
        ((Diff_TorqueCurrentFOC_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_TorqueCurrentFOC_Velocity)request);
    }
    if (request instanceof Diff_PositionTorqueCurrentFOC_Velocity) {
      logOutputs(
        request,
        ((Diff_PositionTorqueCurrentFOC_Velocity)request).AverageRequest.Position,
        ((Diff_PositionTorqueCurrentFOC_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_PositionTorqueCurrentFOC_Velocity)request);
    }
    if (request instanceof Diff_VelocityTorqueCurrentFOC_Velocity) {
      logOutputs(
        request,
        ((Diff_VelocityTorqueCurrentFOC_Velocity)request).AverageRequest.Velocity,
        ((Diff_VelocityTorqueCurrentFOC_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_VelocityTorqueCurrentFOC_Velocity)request);
    }
    if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Velocity) {
      logOutputs(
        request,
        ((Diff_MotionMagicTorqueCurrentFOC_Velocity)request).AverageRequest.Position,
        ((Diff_MotionMagicTorqueCurrentFOC_Velocity)request).DifferentialRequest.Velocity
      );
      return m_talon.setControl((Diff_MotionMagicTorqueCurrentFOC_Velocity)request);
    }

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
