//package frc.robot;
package frc.robot;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

import javax.imageio.ImageIO;

public class TriangleInterpolator {
  private int m_outputSize;
  private List<OutputTriangle> m_triangleData;

  public TriangleInterpolator(int outputSize, String pathName, String pointFileName) {
    m_outputSize = outputSize;
    m_triangleData = Output.getJSONTriangles(pathName, pointFileName);
  }

  // public Polygon asPolygon() {
  // PolygonPoint[] points = new PolygonPoint[m_pointSet.getPoints().size()];
  // int i = 0;
  // for (TriangulationPoint point : m_pointSet.getPoints()) {
  // PolygonPoint polygonPoint = new PolygonPoint(point.getX(), point.getY());
  // points[i] = polygonPoint;
  // i++;
  // }
  // Polygon polygon = new Polygon(points);
  // return polygon;
  // }

  private static double getTriangleArea(OutputPoint p1, OutputPoint p2, OutputPoint p3) {
    return ((p2.getX() - p1.getX()) * (p3.getY() - p1.getY()) - (p3.getX() - p1.getX()) * (p2.getY() - p1.getY())) / 2;
  }

  private static boolean isInsideTriangle(double[] weights) {
    // double totalArea = getTriangleArea(triangle.points[0], triangle.points[1],
    // triangle.points[2]);
    // double areaA = getTriangleArea(point, triangle.points[1],
    // triangle.points[2]);
    // double areaB = getTriangleArea(triangle.points[0], point,
    // triangle.points[2]);
    // double areaC = getTriangleArea(triangle.points[0], triangle.points[1],
    // point);

    // if (Math.abs(totalArea - (areaA + areaB + areaC)) < 0.001) {
    // //TODO
    // }

    if (weights[0] < 0 || weights[1] < 0 || weights[2] < 0) {
      return false;
    }
    return true;
  }

  public double[] getWeights(double x, double y, OutputTriangle triangle) {
    OutputPoint v1 = triangle.getPoint1();
    OutputPoint v2 = triangle.getPoint2();
    OutputPoint v3 = triangle.getPoint3();

    // Formula from: https://codeplea.com/triangular-interpolation
    double weight1 = (((v2.getY() - v3.getY()) * (x - v3.getX()))
        + ((v3.getX() - v2.getX()) * (y - v3.getY())))
        / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY())));

    double weight2 = (((v3.getY() - v1.getY()) * (x - v3.getX())
        + ((v1.getX() - v3.getX()) * (y - v3.getY())))
        / (((v2.getY() - v3.getY()) * (v1.getX() - v3.getX())) + ((v3.getX() - v2.getX()) * (v1.getY() - v3.getY()))));

    double weight3 = (1 - weight1 - weight2);

    double[] output = { weight1, weight2, weight3 };
    return output;

  }

  public void draw(String fileName, int width, int height, double minX, double maxX, double maxY, double minY,
      int dataIndex, double dataMin, double dataMax) {
    BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
    double currentX = minX;
    double currentY = minY;
    // System.out.println((maxY-minY)/(double)height);
    for (int i = 0; i < width; i++) {
      currentY = minY;
      for (int j = 0; j < height; j++) {
        // System.out.println(currentX);
        // System.out.println(currentY);
        Optional<double[]> colorData = getTriangulatedOutput(currentX, currentY);
        int color = 0;
        if (colorData.isPresent()) {
          double input = colorData.get()[dataIndex];
          color = (int) (255 * ((input - dataMin) / (dataMax - dataMin)));
          if (color > 255) {
            color = 255;
          }
          if (color < 0) {
            color = 0;
          }
          image.setRGB(i, j, new Color(color, color, color).getRGB());
        }
        currentY += (maxY - minY) / (double) height;
      }
      currentX += (maxX - minX) / (double) width;
    }

    try {
      File outputFile = new File(fileName);
      System.out.println(outputFile.getAbsolutePath());
      ImageIO.write(image, "png", outputFile);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * 
   * @param robotPose
   * @return Optional double array where 0th element is left shooter speed, 1st is
   *         right shooter speed, 2nd is pivot angle, 3rd is heading
   * 
   */
  public Optional<double[]> getTriangulatedOutput(double x, double y) {
    for (OutputTriangle triangle : m_triangleData) {
      double[] weights = getWeights(x, y, triangle);
      if (isInsideTriangle(weights)) {
        // TODO: Get the z from each of the three points, use it to look up calibration
        // points,
        // then use the weights * z (for each point) for each of the four data
        // categories

        double[] output = new double[m_outputSize];

        for (int i = 0; i < m_outputSize; i += 1) {
          output[i] = 0;
          for (int j = 1; j < 4; j += 1) {
            output[i] += triangle.getPoint(j).getOneData(i) * weights[j - 1];
          }
        }
        return Optional.of(output);
      }
    }
    return Optional.empty();
  }

  // public Optional<double[]> getWalkingTriangulatedOutput(double x, double y) {
  // List<DelaunayTriangle> triangles = m_pointSet.getTriangles();
  // DelaunayTriangle currentTriangle = m_lastTriangle;
  // double xyDistanceToQuery = Math.sqrt(Math.pow(m_lastPoint.getX() - x, 2) +
  // Math.pow(m_lastPoint.getY() - y,2));
  // for (DelaunayTriangle neighbor : m_lastTriangle.neighbors) {

  // }
  // return Optional.empty();
  // // double[] weights = getWeights(new Pose2d(robotPose.getX(),
  // robotPose.getY(), new Rotation2d()), );

  // }
}