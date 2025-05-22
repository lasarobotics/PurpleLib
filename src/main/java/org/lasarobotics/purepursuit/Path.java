package org.lasarobotics.purepursuit;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// Assume Pose2d, Rotation2d, Translation2d are defined as discussed
// or imported from a library like edu.wpi.first.math.geometry.*

public class Path {
  private final List<Pose2d> waypoints;
  private final List<Segment> segments;

  public Path(List<Pose2d> waypoints) {
    if (waypoints == null || waypoints.size() < 2) {
      throw new IllegalArgumentException("Path must contain at least two waypoints.");
    }
    this.waypoints = new ArrayList<>(waypoints);
    this.segments = new ArrayList<>();
    for (int i = 0; i < waypoints.size() - 1; i++) {
      segments.add(new Segment(waypoints.get(i), waypoints.get(i+1)));
    }
  }

  public List<Pose2d> getWaypoints() {
    return waypoints;
  }

  public List<Segment> getSegments() {
    return segments;
  }

  public Pose2d getStartWaypoint() {
    return waypoints.get(0);
  }

  public Pose2d getEndWaypoint() {
    return waypoints.get(waypoints.size() - 1);
  }

  // Helper class for path segments
  public static class Segment {
    public final Pose2d start;
    public final Pose2d end;
    public final Translation2d startTranslation;
    public final Translation2d endTranslation;
    public final double length;
    public final Translation2d segmentVector;


    public Segment(Pose2d start, Pose2d end) {
      this.start = start;
      this.end = end;
      this.startTranslation = start.getTranslation();
      this.endTranslation = end.getTranslation();
      this.segmentVector = endTranslation.minus(startTranslation);
      this.length = segmentVector.getNorm();
    }

    // Finds the closest point on this line segment to a given point.
    public Translation2d findClosestPoint(Translation2d point) {
      Translation2d startToPoint = point.minus(startTranslation);
      double dotProduct = startToPoint.getX() * segmentVector.getX() + startToPoint.getY() * segmentVector.getY();

      // Projection factor: if t is 0, closest is start; if t is 1, closest is end.
      // If t is between 0 and 1, closest is on the segment.
      double t = dotProduct / (length * length);

      if (t < 0.0) {
        return startTranslation;
      } else if (t > 1.0) {
        return endTranslation;
      } else {
        return startTranslation.plus(segmentVector.times(t));
      }
    }

    public List<Translation2d> findCircleIntersections(Translation2d circleCenter, double radius) {
      List<Translation2d> intersections = new ArrayList<>();

      Translation2d d1 = startTranslation.minus(circleCenter);
      Translation2d d2 = endTranslation.minus(circleCenter);

      double dx = d2.getX() - d1.getX();
      double dy = d2.getY() - d1.getY();
      double drSquared = dx * dx + dy * dy;
      double D = d1.getX() * d2.getY() - d2.getX() * d1.getY();
      double discriminant = radius * radius * drSquared - D * D;

      // Negative (with tolerance for floating point)
      if (discriminant < -1e-9) return intersections;

      discriminant = Math.max(0, discriminant); // Ensure non-negative for sqrt

      double sqrtDiscriminant = Math.sqrt(discriminant);
      double sgnDy = (dy < 0) ? -1 : 1;

      // Intersection point 1
      double x1_num = D * dy + sgnDy * dx * sqrtDiscriminant;
      double y1_num = -D * dx + Math.abs(dy) * sqrtDiscriminant;

      // Intersection point 2
      double x2_num = D * dy - sgnDy * dx * sqrtDiscriminant;
      double y2_num = -D * dx - Math.abs(dy) * sqrtDiscriminant;

      if (drSquared < 1e-9) { // p1 and p2 are virtually the same point
        // Check if this point is on the circle (e.g. d1.getNorm() is close to radius)
        if (Math.abs(d1.getNorm() - radius) < 1e-3) {
          intersections.add(startTranslation); // The point itself is an "intersection"
        }
        return intersections;
      }


      Translation2d int1 = new Translation2d(x1_num / drSquared, y1_num / drSquared).plus(circleCenter);
      if (isPointOnSegment(int1)) {
        intersections.add(int1);
      }

      if (discriminant > 1e-9) { // Two distinct points if discriminant is clearly positive
        Translation2d int2 = new Translation2d(x2_num / drSquared, y2_num / drSquared).plus(circleCenter);
        if (isPointOnSegment(int2)) {
          // Avoid adding the same point if numerically very close (e.g. tangent point found twice)
          if (intersections.isEmpty() || int1.getDistance(int2) > 1e-6) {
            intersections.add(int2);
          }
        }
      }
      return intersections;
    }

    public boolean isPointOnSegment(Translation2d point) {
      double epsilon = 1e-3;
      // Check bounding box (inclusive)
      boolean inX = (point.getX() >= Math.min(startTranslation.getX(), endTranslation.getX()) - epsilon) &&
      (point.getX() <= Math.max(startTranslation.getX(), endTranslation.getX()) + epsilon);
      boolean inY = (point.getY() >= Math.min(startTranslation.getY(), endTranslation.getY()) - epsilon) &&
      (point.getY() <= Math.max(startTranslation.getY(), endTranslation.getY()) + epsilon);

      if (!inX || !inY) {
        return false;
      }
      // Check for collinearity: cross product of (pt - segStart) and (segEnd - segStart) should be close to 0.
      // (pt.y - segStart.y) * (segEnd.x - segStart.x) - (pt.x - segStart.x) * (segEnd.y - segStart.y)
      double crossProduct = (point.getY() - startTranslation.getY()) * (endTranslation.getX() - startTranslation.getX()) -
      (point.getX() - startTranslation.getX()) * (endTranslation.getY() - startTranslation.getY());

      return Math.abs(crossProduct) < epsilon * startTranslation.getDistance(endTranslation); // Scale epsilon by segment length for robustness
    }
  }
}
