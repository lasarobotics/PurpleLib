// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.revrobotics;

import java.util.function.Supplier;

import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.Spark.SparkOutput;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.MovingAverageFilterSim;
import org.lasarobotics.utils.NoiseGenerator;
import org.lasarobotics.utils.SimDynamics;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

@SuppressWarnings("unused")
public class SparkSim {
  private static final String LOG_TAG = "SimSpark";
  private static final int PID_SLOT = 0;
  private final SimDouble m_appliedOutput;
  private final SimDouble m_velocity;
  private final SimDouble m_position;
  private final SimDouble m_analogVelocity;
  private final SimDouble m_analogPosition;
  private final SimDouble m_analogVoltage;
  private final SimDouble m_busVoltage;
  private final SimDouble m_motorCurrent;
  private final SimInt m_faults;
  private final SimInt m_stickyFaults;
  private final Spark m_spark;
  private final SimDynamics m_simulatedDynamics;
  private final SimInt m_controlMode;
  private final MovingAverageFilterSim m_velocityAverage;
  private boolean m_forwardLimit = false;
  private boolean m_reverseLimit = false;
  private Boolean m_enable = null;
  private double m_iterateByPositionLast = 0.0;
  private Supplier<SparkOutput> m_latestOutput;

  // PID State
  private double m_iState = 0.0;
  private double m_prev_err = 0.0;
  private double m_pidOutput = 0.0;

  /**
   * This *should* track to the API version. Since this file is dependent on REV, warn if the
   * version changes to go back and verify 1) this still works and 2) that this file is still needed
   * at all.
   */
  //private final int kAPIversionExpected = 132627716;

  /**
   * Create a simulated CAN Spark Max object. This class simulates some of the internal behavior of
   * the device.
   *
   * @param spark The Spark to simulate.
   * @param dynamicsSim A basic dynamics model. If you are not modeling the system externally this
   *     can be used instead. If you have a second motor controller as a follower, you can use a
   *     single CANSparkMaxSim object on the leader, and specify multiple motors using the DCMotor
   *     constructor.
   */
  public SparkSim(Spark spark, SimDynamics dynamicsSim) {
    SimDeviceSim simSpark = new SimDeviceSim("SPARK MAX" + " [" + spark.getID().deviceID + "]");
    this.m_appliedOutput = simSpark.getDouble("Applied Output");
    this.m_velocity = simSpark.getDouble("Velocity");
    this.m_position = simSpark.getDouble("Position");
    this.m_analogVelocity = simSpark.getDouble("Analog Velocity");
    this.m_analogPosition = simSpark.getDouble("Analog Position");
    this.m_analogVoltage = simSpark.getDouble("Analog Voltage");
    this.m_busVoltage = simSpark.getDouble("Bus Voltage");
    this.m_motorCurrent = simSpark.getDouble("Motor Current");
    this.m_controlMode = simSpark.getInt("Control Mode");
    this.m_faults = simSpark.getInt("Faults");
    this.m_stickyFaults = simSpark.getInt("Sticky Faults");
    this.m_spark = spark;
    this.m_latestOutput = () -> m_spark.getLatestOutput();
    this.m_velocityAverage = m_spark.getKind().equals(MotorKind.NEO_VORTEX)
      ? new MovingAverageFilterSim(8, 0.032)
      : new MovingAverageFilterSim(2, 0.016);
    this.m_simulatedDynamics = dynamicsSim;


    // int apiVersion = CANSparkMaxJNI.c_SparkMax_GetAPIVersion();
    // if (apiVersion != kAPIversionExpected) {
    //   System.err.printf(getClass().getSimpleName() +
    //           "CAN Spark Max API version changed, verify that the sim setup still works correctly. Got {} expected {}",
    //           apiVersion,
    //           kAPIversionExpected);
    // }
  }

  /**
   * Create a simulated Spark object. This class simulates some of the internal behavior of
   * the device.
   *
   * <p>This constructor uses a single NEO as default with no load, however if you simulating the
   * motor using a different sim mechanism it can be ignored and setMotorCurrent() can be used
   * instead.
   *
   * @param spark The CANSparkMax to simulate.
   */
  public SparkSim(Spark spark) {
    this(spark, SimDynamics.fromSim(new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.00003973)));
  }

  /**
   * Get the simulated applied output. This matches the value from the CANSparkMax
   * getAppliedOutput(). Multiply by vbus to get the motor voltage.
   *
   * @return applied output [-1, 1]
   */
  public double getAppliedOutput() {
    return m_appliedOutput.get();
  }

  /**
   * Set the simulated applied output. Use this only in place of {@link SparkSim#update(Measure, Measure)}.
   *
   * @param appliedOutput simulated applied output value [-1, 1]
   */
  public void setAppliedOutput(double appliedOutput) {
    m_appliedOutput.set(appliedOutput);
  }

  // Modified from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
  private double runPID(int pidSlot, double setpoint, double pv, double dt) {
    var controller = m_spark.getPIDController();
    double error = setpoint - pv;

    double p = error * controller.getP(pidSlot);

    if (Math.abs(error) <= controller.getIZone(pidSlot) || controller.getIZone(pidSlot) != 0.0f) {
      m_iState = m_iState + (error * controller.getI(pidSlot) * dt);
    } else m_iState = 0;

    double d = (error - m_prev_err) / dt;
    m_prev_err = error;
    d *= controller.getD(pidSlot);

    double f = setpoint * controller.getFF(pidSlot);

    double output = p + m_iState + d + f;
    m_pidOutput =
        Math.min(Math.max(output, controller.getOutputMin(pidSlot)), controller.getOutputMax(pidSlot));

    return m_pidOutput;
  }

  /**
   * Run soft limit logic
   * @param forward True if motor is running forward
   * @return True if limit is tripped
   */
  private boolean runLimitLogic(boolean forward) {
    if (forward) {
      if ((m_spark.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get())
          && m_spark.isSoftLimitEnabled(SoftLimitDirection.kForward)) {
        return true;
      }

      return m_spark.isSoftLimitEnabled(SoftLimitDirection.kForward) && m_forwardLimit;
    } else {
      if ((m_spark.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get())
          && m_spark.isSoftLimitEnabled(SoftLimitDirection.kReverse)) {
        return true;
      }

      return m_spark.isSoftLimitEnabled(SoftLimitDirection.kReverse) && m_reverseLimit;
    }
  }

  /**
   * Run internal calculations and set internal state when using simulated dynamics built in. If not
   * using the built in dynamics, call the three parameter version instead.
   *
   * @param vbus Bus voltage
   */
  public void update(Measure<Voltage> vbus) {
    update(Units.Value.of(m_simulatedDynamics.getAngularVelocity().in(Units.RPM)), vbus);
    m_simulatedDynamics.setInputVoltage(vbus.times(m_spark.getAppliedOutput()));
    m_simulatedDynamics.update();
    setMotorCurrent(m_simulatedDynamics.getCurrentDraw());
  }

  /**
   * Run internal calculations and set internal state when using simulated dynamics built in from position. If not
   * using the built in dynamics, call the three parameter version instead.
   *
   * @param position Position of mechanism
   * @param vbus Bus voltage
   */
  public void updateByPosition(Measure<Dimensionless> position, Measure<Voltage> vbus) {
    // Non-ideal way to do this, but okay. Calculate the velocity here and force position
    update(Units.Value.of((position.in(Units.Value) - m_iterateByPositionLast) * GlobalConstants.ROBOT_LOOP_PERIOD), vbus);
    m_position.set(position.in(Units.Value));
    m_iterateByPositionLast = position.in(Units.Value);
  }

  /**
   * Run internal calculations and set internal state including
   *
   * <p>- Velocity (set by velocity, not calculated in this method) - Position - Bus Voltage (set by
   * vbus, not calculated in this method) - Current - Applied Output
   *
   * @param velocity The externally calculated velocity in units after conversion. For example, if
   *     the velocity factor is 1, use RPM. If the velocity factor is (1 / 60) use RPS. The internal
   *     simulation state will 'lag' behind this input due to the Spark internal filtering.
   *     Noise will also be added.
   * @param velocity Mechanism velocity
   * @param vbus Bus voltage
   */
  public void update(Measure<Dimensionless> velocity, Measure<Voltage> vbus) {
    // Velocity input is the system simulated input.
    double internalVelocity = NoiseGenerator.hallSensorVelocity(velocity.in(Units.Value));
    m_velocityAverage.put(internalVelocity, GlobalConstants.ROBOT_LOOP_PERIOD);
    internalVelocity = m_velocityAverage.get();

    // Get latest output
    var latestOutput = m_latestOutput.get();

    // First set the states that are given
    m_velocity.set(velocity.in(Units.Value));

    // Get conversion factors
    double positionFactor = m_spark.getPositionConversionFactor(FeedbackSensor.NEO_ENCODER);
    double velocityFactor = m_spark.getVelocityConversionFactor(FeedbackSensor.NEO_ENCODER);

    assert positionFactor != 0.0;
    assert velocityFactor != 0.0;

    double velocityRPM = velocity.in(Units.Value) / velocityFactor;
    m_position.set(m_position.get() + ((velocityRPM / 60) * GlobalConstants.ROBOT_LOOP_PERIOD) / positionFactor);
    m_busVoltage.set(vbus.in(Units.Volts));

    // Calcuate the applied output
    double appliedOutput = 0.0;
    switch (m_controlMode.get()) {
        // Duty Cycle
      case 0:
        appliedOutput = m_latestOutput.get().value;
        break;

        // Velocity
      case 1:
        appliedOutput = runPID(PID_SLOT, latestOutput.value, m_velocity.get(), GlobalConstants.ROBOT_LOOP_PERIOD);
        break;

        // Voltage
      case 2:
        appliedOutput = m_latestOutput.get().value / vbus.in(Units.Volts);
        break;

        // Position
      case 3:
        appliedOutput = runPID(PID_SLOT, latestOutput.value, m_position.get(), GlobalConstants.ROBOT_LOOP_PERIOD);
        break;

        // Smart Motion
      case 4:
        // TODO... This control mechanism is not documented
        break;

        // Current
      case 5:
        appliedOutput = runPID(PID_SLOT, latestOutput.value, m_motorCurrent.get(), GlobalConstants.ROBOT_LOOP_PERIOD);
        break;

        // Smart Velocity
      case 6:
        // TODO... This control mechanism is not documented
        break;

      default:
        org.tinylog.Logger.tag(LOG_TAG).error(m_spark.getID().name + " Invalid control mode: {}", m_controlMode.get());
    }

    // ArbFF
    if (latestOutput.arbFFUnits.value == 0) {
      // Voltage
      appliedOutput += latestOutput.arbFeedforward / vbus.in(Units.Volts);
    } else {
      // Duty Cycle
      appliedOutput += latestOutput.arbFeedforward;
    }

    // Limit to [-1, 1] or limit switch value
    double maxOutput = runLimitLogic(true) ? 0 : 1;
    double minOutput = runLimitLogic(false) ? 0 : -1;
    appliedOutput = Math.min(Math.max(appliedOutput, minOutput), maxOutput);

    // TODO: Voltage Comp

    // TODO: Selected Sensor?

    // TODO: Faults

    // And finally, set remaining states
    boolean doEnable = false;
    if (m_enable == null) {
      doEnable = DriverStation.isEnabled();
    } else {
      doEnable = m_enable;
    }

    if (doEnable) {
      m_appliedOutput.set(appliedOutput);
    } else {
      m_appliedOutput.set(0.0);
    }

    // TODO: Current Limits
  }

  public void setFaults(CANSparkMax.FaultID... faults) {
    int result = 0;
    for (var fault : faults) {
      result |= (1 << fault.value);
    }
    m_faults.set(result);

    int stickyFaults = result | m_stickyFaults.get();
    m_stickyFaults.set(stickyFaults);
  }

  /**
   * Set the simulation velocity. This method expects units after the conversion factor (your
   * program's native units).
   *
   * <p>Only use this method if not calling {@link SparkSim#update(Measure, Measure)}
   *
   * @param velocity simulation velocity
   */
  public void setVelocity(double velocity) {
    m_velocity.set(velocity);
  }

  /**
   * Set the simulated position. This is equivilant to calling CANEncoder().setPosition(), in fact
   * you probably are using that unless you have a good reason to set the sim value separately, or
   * are running simulation without using {@link SparkSim#update(Measure, Measure)}
   *
   * @param position simulated position in your programs units (after conversion)
   */
  public void setPosition(double position) {
    m_position.set(position);
  }

  /**
   * Set the simulated bus voltage. Use this if you are not using the {@link SparkSim#update(Measure, Measure)} method.
   * @param vbus Bus voltage
   */
  public void setBusVoltage(Measure<Voltage> vbus) {
    m_busVoltage.set(vbus.in(Units.Volts));
  }

  /**
   * Set the simulated motor current. The {@link SparkSim#update(Measure, Measure)} method also sets this value. If you are using an
   * external method to calculate the current, but still want to use the {@link SparkSim#update(Measure, Measure)} method, call this
   * function *after* {@link SparkSim#update(Measure, Measure)}
   *
   * @param current Motor current
   */
  public void setMotorCurrent(Measure<Current> current) {
    m_motorCurrent.set(current.in(Units.Amps));
  }

  /**
   * Set the state of the forward limit switch. Set true to indicate that the forward limit switch
   * is set.
   *
   * <p>This method does not have any knowlege of polarity. So if you set the Spark Max limit switch
   * as 'normally open' and set tripped = true, then the limit is concidered closed (tripped). If
   * you set the Spark Max limit switch as 'normally closed' as set tripped = true, then the limit
   * switch is considered open (tripped).
   *
   * @param tripped set true to trip the forward limit
   */
  public void setForwardLimitSwitch(boolean tripped) {
    m_forwardLimit = tripped;
  }

  /**
   * Set the state of the reverse limit switch. Set true to indicate that the reverse limit switch
   * is set.
   *
   * <p>This method does not have any knowlege of polarity. So if you set the Spark Max limit switch
   * as 'normally open' and set tripped = true, then the limit is concidered closed (tripped). If
   * you set the Spark Max limit switch as 'normally closed' as set tripped = true, then the limit
   * switch is considered open (tripped).
   *
   * @param tripped set true to trip the reverse limit
   */
  public void setReverseLimitSwitch(boolean tripped) {
    m_reverseLimit = tripped;
  }

  /**
   * Get the simulation velocity. This should be equivilant to calling CANEncoder().getVelocity()
   * @return Velocity of the SPARK accounting for conversion factor
   */
  public double getVelocity() {
    return m_velocity.get();
  }

  /**
   * Get the simulation position. This should be equivilant to calling CANEncoder().getPosition()
   *
   * @return Velocity of the SPARK MAX
   */
  public double getPosition() {
    return m_position.get();
  }

  /**
   * Get the simulated bus voltage
   * @return Simulated bus voltage
   */
  public double getBusVoltage() {
    return m_busVoltage.get();
  }

  /**
   * Get the simulated motor current in amps. This is equivalant to running
   * sparkmax.getOutputCurrent()
   *
   * @return motor current in amps
   */
  public Measure<Current> getMotorCurrent() {
    return Units.Amps.of(m_motorCurrent.get());
  }

  /**
   * Get the simulated forward limit switch state
   * @return true if tripped. Does not look at whether or not enabled
   */
  public boolean getForwardLimitSwitch() {
    return m_forwardLimit;
  }

  /**
   * Get the simulated reverse limit switch state
   * @return true if tripped. Does not look at whether or not enabled
   */
  public boolean getReverseLimitSwitch() {
    return m_reverseLimit;
  }

  public double getSetpoint() {
    return m_latestOutput.get().value;
  }

  public void enable() {
    m_enable = true;
  }

  public void disable() {
    m_enable = false;
  }

  public void useDriverStationEnable() {
    m_enable = null;
  }
}