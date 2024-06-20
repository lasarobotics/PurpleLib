// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.hardware.pimoroni;

import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class PIM453 {
  private static final int BITRATE = 400000;

  private SPI m_port;

  public PIM453(SPI.Port port) {
    m_port = new SPI(port);

    // Initialize SPI port for communication
    m_port.setClockRate(BITRATE);
    m_port.setMode(SPI.Mode.kMode3);
    m_port.setChipSelectActiveLow();
  }

  private boolean read(byte address, byte[] buffer, int size) {
    boolean success = false;

    byte[] data = new byte[3];
    data[0] = address;
    data[1] = (byte)buffer.length;

    synchronized (this) {
      success = m_port.read(false, buffer, size) == size;
    }

    if (!success) {
      System.err.println("Read error!");
      return false;
    }
    return true;
  }

  private boolean write(byte address, byte value) {
    boolean success = false;

    byte[] data = new byte[2];
    data[0] = (byte)(address | (byte)0x80);
    data[1] = value;

    synchronized (this) {
    	success = m_port.write(data, data.length) == data.length;
    }

    if (!success) {
      System.err.println("Write error!");
      return false;
    }
    return true;
  }


}
