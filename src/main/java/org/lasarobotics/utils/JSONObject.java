// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.utils;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Read/write JSON */
public class JSONObject {
  private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();
  private static class JSONPose {
    @JsonProperty("x") double x;
    @JsonProperty("y") double y;
    @JsonProperty("rotation") double rotation;

    private Pose2d toJavaType() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }

    private static JSONPose toJSON(Pose2d pose) {
      JSONPose jsonPose = new JSONPose();

      jsonPose.x = pose.getX();
      jsonPose.y = pose.getY();
      jsonPose.rotation = pose.getRotation().getDegrees();

      return jsonPose;
    }
  }

  private static class JSONPoint {
    @JsonProperty("x") double x;
    @JsonProperty("y") double y;

    private Translation2d toJavaType() {
      return new Translation2d(x, y);
    }

    private static JSONPoint toJSON(Translation2d pose) {
      JSONPoint jsonPoint = new JSONPoint();

      jsonPoint.x = pose.getX();
      jsonPoint.y = pose.getY();

      return jsonPoint;
    }
  }

  private static class JSONPoseList {
    private static List<Pose2d> toJavaType(List<JSONPose> poses) {
      List<Pose2d> poseList = new ArrayList<Pose2d>();
      for (var pose : poses) poseList.add(pose.toJavaType());

      return poseList;
    }

    private static List<JSONPose> toJSON(List<Pose2d> poseList) {
      List<JSONPose> jsonPoseList = new ArrayList<JSONPose>();
      for (var pose : poseList) jsonPoseList.add(JSONPose.toJSON(pose));

      return jsonPoseList;
    }
  }

  private static class JSONPointList {
    private static List<Translation2d> toJavaType( List<JSONPoint> points) {
      List<Translation2d> pointList = new ArrayList<Translation2d>();
      for (var point : points) pointList.add(point.toJavaType());

      return pointList;
    }

    private static List<JSONPoint> toJSON(List<Translation2d> poseList) {
      List<JSONPoint> jsonPointList = new ArrayList<JSONPoint>();
      for (var pose : poseList) jsonPointList.add(JSONPoint.toJSON(pose));

      return jsonPointList;
    }
  }

  /**
   * Convert JSON pose to object
   * @param json JSON strin containing 2D pose
   * @return Object representing pose
   */
  public static Pose2d readPose(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPose.class).toJavaType(); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert JSON point to object
   * @param json JSON string containing point
   * @return Object representing point
   */
  public static Translation2d readPoint(String json) {
    try { return OBJECT_MAPPER.readValue(json, JSONPoint.class).toJavaType(); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert JSON pose list to object list
   * @param json JSON string of poses
   * @return Object list of poses
   */
  public static List<Pose2d> readPoseList(String json) {
    try { return JSONPoseList.toJavaType(OBJECT_MAPPER.readValue(json, new TypeReference<List<JSONPose>>(){})); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert JSON point list to object list
   * @param json JSON string of points
   * @return Object list of points
   */
  public static List<Translation2d> readPointList(String json) {
    try { return JSONPointList.toJavaType(OBJECT_MAPPER.readValue(json, new TypeReference<List<JSONPoint>>(){})); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert pose object to JSON string
   * @param pose Pose object
   * @return JSON string representing pose
   */
  public static String writePose(Pose2d pose) {
    try { return OBJECT_MAPPER.writeValueAsString(JSONPose.toJSON(pose)); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert point object to JSON string
   * @param point Point object
   * @return JSON string representing point
   */
  public static String writePoint(Translation2d point) {
    try { return OBJECT_MAPPER.writeValueAsString(JSONPoint.toJSON(point)); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert list of pose objects to JSON string
   * @param poseList List of pose objects
   * @return JSON string representing pose list
   */
  public static String writePoseList(List<Pose2d> poseList) {
    try { return OBJECT_MAPPER.writeValueAsString(JSONPoseList.toJSON(poseList)); }
    catch (Exception e) { return null; }
  }

  /**
   * Convert list of point objects to JSON string
   * @param pointList List of point objects
   * @return JSON string representing point list
   */
  public static String writePointList(List<Translation2d> pointList) {
    try { return OBJECT_MAPPER.writeValueAsString(JSONPointList.toJSON(pointList)); }
    catch (Exception e) { return null; }
  }
}
