// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project

package org.lasarobotics.hardware;

import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DMA;
import edu.wpi.first.wpilibj.DMASample;

import edu.wpi.first.wpilibj.DMASample.DMAReadStatus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Notifier;

public class DMAService extends SubsystemBase {
  private static DMAService m_dmaService;


  private static final double TIMEOUT = 1e-3;
  private static final int QUEUE_DEPTH = 1024;
  private static final String READ_STATUS_ENTRY = "DMAService/ReadStatus";

  DMA m_dma;
  DMASample m_dmaSample;
  DMAReadStatus m_readStatus;
  Notifier m_dmaThread;

  private DMAService() {
    this.m_dma = new DMA();
    this.m_dmaSample = new DMASample();
    this.m_readStatus = DMAReadStatus.kOk;
    this.m_dmaThread = new Notifier(() -> run());

    m_dma.setTimedTrigger(GlobalConstants.ROBOT_LOOP_PERIOD / 2);
  }

  public static DMAService getInstance() {
    if (m_dmaService == null) m_dmaService = new DMAService();
    return m_dmaService;
  }

  private void run() {
    m_readStatus = m_dmaSample.update(m_dma, TIMEOUT);
    Logger.recordOutput(READ_STATUS_ENTRY, m_readStatus.name());
  }

  public void start() {
    m_dma.start(QUEUE_DEPTH);
    m_dmaThread.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD / 2);
  }
}
