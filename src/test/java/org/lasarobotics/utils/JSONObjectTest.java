// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.lasarobotics.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class JSONObjectTest {
  private final double DELTA = 1E-3;
  private final String JSON_POSE = "{\"x\":1.0,\"y\":2.0,\"rotation\":90.0}";
  private final String JSON_POINT = "{\"x\":1.0,\"y\":2.0}";
  private final String JSON_POSE_LIST = "[{\"x\":1.0,\"y\":2.0,\"rotation\":90.0},{\"x\":1.0,\"y\":2.0,\"rotation\":90.0},{\"x\":1.0,\"y\":2.0,\"rotation\":90.0}]";
  private final String JSON_POINT_LIST = "[{\"x\":1.0,\"y\":2.0},{\"x\":1.0,\"y\":2.0},{\"x\":1.0,\"y\":2.0}]";

  @Test
  @Order(1)
  @DisplayName("Test if robot can read/write JSON poses")
  public void JSONPose() {
    Pose2d pose = JSONObject.readPose(JSON_POSE);

    assertEquals(pose.getX(), 1.0, DELTA);
    assertEquals(pose.getY(), 2.0, DELTA);
    assertEquals(pose.getRotation().getDegrees(), 90.0, DELTA);

    String json = JSONObject.writePose(pose);
    assertEquals(JSON_POSE, json);
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can read/write JSON points")
  public void JSONPoint() {
    Translation2d point = JSONObject.readPoint(JSON_POINT);

    assertEquals(point.getX(), 1.0, DELTA);
    assertEquals(point.getY(), 2.0, DELTA);

    String json = JSONObject.writePoint(point);
    assertEquals(JSON_POINT, json);
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can read/write JSON pose lists")
  public void JSONPoseList() {
    List<Pose2d> poseList = JSONObject.readPoseList(JSON_POSE_LIST);

    assertEquals(poseList.size(), 3);
    for (var pose : poseList) {
      assertEquals(pose.getX(), 1.0, DELTA);
      assertEquals(pose.getY(), 2.0, DELTA);
    }

    String json = JSONObject.writePoseList(poseList);
    assertEquals(JSON_POSE_LIST, json);
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can read/write JSON point lists")
  public void JSONPointList() {
    List<Translation2d> pointList = JSONObject.readPointList(JSON_POINT_LIST);

    assertEquals(pointList.size(), 3);
    for (var point : pointList) {
      assertEquals(point.getX(), 1.0, DELTA);
      assertEquals(point.getY(), 2.0, DELTA);
    }

    String json = JSONObject.writePointList(pointList);
    assertEquals(JSON_POINT_LIST, json);
  }
}

