package org.lasarobotics.hardware.thethriftybot;

import org.lasarobotics.hardware.LoggableHardware;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class ThriftyNova extends LoggableHardware {

  public static class ID {
    public final String name;
    public final int deviceID;


    public ID(String name, int deviceID) {
      this.name = name;
      this.deviceID = deviceID;
    }
  }

  @Override
  protected void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public LoggableInputs getInputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getInputs'");
  }

  @Override
  public void close() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
}


