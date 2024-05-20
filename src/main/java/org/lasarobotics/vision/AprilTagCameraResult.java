// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagCameraResult {
  public final EstimatedRobotPose estimatedRobotPose;
  public final Matrix<N3, N1> standardDeviation;

  public AprilTagCameraResult(EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> standardDeviation) {
    this.estimatedRobotPose = estimatedRobotPose;
    this.standardDeviation = standardDeviation;
  }
}
