// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package org.lasarobotics.purepursuit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
* This class contains various static methods that are used in the pure pursuit algorithm.
*
* @author Michael Baljet, Team 14470
* @version 1.0
*/
public final class PurePursuitUtil {

  // This class only has static items so the constructor is hidden.
  private PurePursuitUtil() {}

  /**
  * Calculates if a point is further along a line then another point. Useful for determining the best
  * intersection point in pure pursuit. Both points are assumed to lie on the line.
  *
  * @param linePoint1 First point of the line.
  * @param linePoint2 Second point of the line.
  * @param point1     Point to be compared.
  * @param point2     Point that point1 is compared too.
  * @return True if point1 is ahead of point2 on the given line.
  */
  public static boolean isInFront(Translation2d linePoint1, Translation2d linePoint2, Translation2d point1, Translation2d point2) {
    if (linePoint1.getX() < linePoint2.getX() && point1.getX() < point2.getX()) return false;
    if (linePoint1.getY() < linePoint2.getY() && point1.getY() < point2.getY()) return false;
    return true;
  }

  /**
  * Calculates whether or not two points are equal within a margin of error.
  *
  * @param p1     Point 1
  * @param p2     Point 2
  * @param buffer Margin of error.
  * @return True if the point are equal within a margin or error, false otherwise.
  */
  public static boolean positionEqualsWithBuffer(Translation2d p1, Translation2d p2, double buffer) {
    if (p1.getX() - buffer < p2.getX() && p1.getX() + buffer > p2.getX())
    if (p1.getY() - buffer < p2.getY() && p1.getY() + buffer > p2.getY())
    return true;
    return false;
  }

  /**
  * Calculates whether or not two angles are equal within a margin of error.
  *
  * @param a1     Angle 1 (in radians).
  * @param a2     Angle 2 (in radians).
  * @param buffer Margin of error (in radians)
  * @return True if the point are equal within a margin or error, false otherwise.
  */
  public static boolean rotationEqualsWithBuffer(double a1, double a2, double buffer) {
    if (a1 - buffer < a2 && a1 + buffer > a2) return true;
    return false;
  }

  /**
  * This method finds points where a line intersects with a circle.
  *
  * @param circleCenter Center of the circle.
  * @param radius       Radius of the circle.
  * @param linePoint1   One of the line's end points.
  * @param linePoint2   The other end point of the line.
  * @return A list containing all point where the line and circle intersect.
  */
  public static List<Translation2d> lineCircleIntersection(Translation2d circleCenter, Distance radius, Translation2d linePoint1, Translation2d linePoint2) {
    // This method was lifted from Team 11115 Gluten Free's code.

    double baX = linePoint2.getX() - linePoint1.getX();
    double baY = linePoint2.getY() - linePoint1.getY();
    double caX = circleCenter.getX() - linePoint1.getX();
    double caY = circleCenter.getY() - linePoint1.getY();

    double a = baX * baX + baY * baY;
    double bBy2 = baX * caX + baY * caY;
    double c = caX * caX + caY * caY - radius.in(Units.Meters) * radius.in(Units.Meters);

    double pBy2 = bBy2 / a;
    double q = c / a;

    double disc = pBy2 * pBy2 - q;
    if (disc < 0) return Collections.emptyList();

    double tmpSqrt = Math.sqrt(disc);
    double abScalingFactor1 = -pBy2 + tmpSqrt;
    double abScalingFactor2 = -pBy2 - tmpSqrt;

    List<Translation2d> allPoints = null;

    Translation2d p1 = new Translation2d(linePoint1.getX() - baX * abScalingFactor1, linePoint1.getY() - baY * abScalingFactor1);
    if (disc == 0) {
      allPoints = Collections.singletonList(p1);
    }

    if (allPoints == null) {
      Translation2d p2 = new Translation2d(linePoint1.getX() - baX * abScalingFactor2, linePoint1.getY() - baY * abScalingFactor2);
      allPoints = Arrays.asList(p1, p2);
    }

    double maxX = Math.max(linePoint1.getX(), linePoint2.getX());
    double maxY = Math.max(linePoint1.getY(), linePoint2.getY());
    double minX = Math.min(linePoint1.getX(), linePoint2.getX());
    double minY = Math.min(linePoint1.getY(), linePoint2.getY());

    List<Translation2d> boundedPoints = new ArrayList<Translation2d>();

    for (var point : allPoints) {
      if (point.getX() <= maxX && point.getX() >= minX)
      if (point.getY() <= maxY && point.getY() >= minY)
      boundedPoints.add(point);
    }

    return boundedPoints;
  }

}
