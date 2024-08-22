package org.lasarobotics.drive;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.dyn4j.collision.narrowphase.Gjk;
import org.dyn4j.collision.narrowphase.NarrowphaseDetector;
import org.dyn4j.collision.narrowphase.Penetration;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.World;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;

public class CollisionDetection {
  private static World<Body> m_field;
  private static List<Body> m_obstacles;

  private static NarrowphaseDetector m_detector;
  private static Penetration m_penetration = new Penetration();

  /**
   * Constructor for CollisionDetection.
   * Initializes the world, collision detector, and penetration objects.
   * Reads obstacles from a JSON file and adds them to the world.
   */
  public CollisionDetection() {
    m_field = new World<>();
    m_detector = new Gjk();
    m_penetration = new Penetration();

    try (InputStream inputStream = getClass().getClassLoader().getResourceAsStream("2024.json")) {
      if (inputStream == null) {
        throw new IllegalArgumentException("Resource not found: " + "2024.json");
      }
      // Parse the JSON resource
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode rootNode = objectMapper.readTree(inputStream);

      // Extract obstacles from the JSON
      JsonNode obstaclesNode = rootNode.get("obstacles");
      for (JsonNode obstacleNode : obstaclesNode) {
        JsonNode verticesNode = obstacleNode.get("vertices");
        Vector2[] vertices = new Vector2[verticesNode.size()];

        for (int i = 0; i < verticesNode.size(); i++) {
          JsonNode vertexNode = verticesNode.get(i);
          double x = vertexNode.get(0).asDouble();
          double y = vertexNode.get(1).asDouble();
          vertices[i] = new Vector2(x, y);
        }

        // Add the obstacle to the list and world
        addObstacles(vertices);
      }

      // Now you can proceed with your collision detection logic
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Adds an obstacle to the world and the list of obstacles.
   * @param vertices An array of vertices representing the shape of the obstacle.
   */
  public void addObstacles(Vector2[] vertices) {
    // Print vertices for debugging
    System.out.println("Received vertices:");
    for (Vector2 vertex : vertices) {
        System.out.println("Vertex: (" + vertex.x + ", " + vertex.y + ")");
    }

    Body obstacle = new Body();
    obstacle.addFixture(Geometry.createPolygon(vertices));
    m_obstacles.add(obstacle);
    m_field.addBody(obstacle);
  }

  /**
   * Checks for collision between a robot and any obstacles in the world.
   * @param robotPose The pose (position and rotation) of the robot.
   * @param robotWidth The width of the robot.
   * @param robotHeight The height of the robot.
   * @return True if the robot collides with any obstacle, false otherwise.
   */
  public boolean checkRobotCollision(Pose2d robotPose, double robotWidth, double robotHeight) {
    Vector2 position = new Vector2(robotPose.getX(), robotPose.getY());
    double rotation = robotPose.getRotation().getRadians();

    Body robotBody = new Body();
    robotBody.addFixture(new BodyFixture(Geometry.createRectangle(robotWidth, robotHeight)));
    robotBody.translate(position);
    robotBody.rotate(rotation, position);
    
    for (Body obstacle : m_obstacles) {
      boolean collision = m_detector.detect(
        robotBody.getFixture(0).getShape(), robotBody.getTransform(),
        obstacle.getFixture(0).getShape(), obstacle.getTransform(),
        m_penetration
      );

      if (collision) {
        return true;
      }
    }

    return false;
  }
}
