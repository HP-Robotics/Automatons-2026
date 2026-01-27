// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  Pose2d loadingPose; // This is needed because the april tag field causes a like 6-10 second loop
                      // overrun the first time we see an april tag so we need to run it on boot
  CommandJoystick m_driveJoystick;
  ArrayList<LimelightCamera> m_cameras;

  class LimelightCamera {
    NetworkTable m_table;
    DoubleArraySubscriber m_sub;
    StructPublisher<Pose2d> m_posePublisher;

    public LimelightCamera(String name) {
      m_table = NetworkTableInstance.getDefault().getTable(name);
      m_table.getEntry("imumode_set").setDouble(0);
      m_table.getEntry("imuassistalpha_set").setDouble(LimelightConstants.imuAssist);
      m_sub = m_table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(defaultValues);
      m_posePublisher = m_table.getStructTopic("Pose", Pose2d.struct).publish();
    }
  }

    private double[] defaultValues = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseEstimatorTable = inst.getTable("pose-estimator-table");

  StructPublisher<Pose2d> publisher;

  public class VisionMeasurement {
    public Pose2d m_visionPose;
    public double m_timeStamp;

    public VisionMeasurement(Pose2d visionPose, double timeStamp, int targetAprilTagID, double angleDiff) {
      m_visionPose = visionPose;
      m_timeStamp = timeStamp;
    }
  }

  public LimelightSubsystem() {
    loadingPose = LimelightConstants.aprilTagList[1];

    m_cameras = new ArrayList<LimelightCamera>();
    m_cameras.add(new LimelightCamera("limelight-shpwrte"));
    m_cameras.add(new LimelightCamera("limelight-kite"));
  }

  // TODO: move all geometry code to a geometry util file
  public double getDistanceToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();

    return Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
  }

  public double getAngleToPose(Pose2d robot, Pose2d fieldPose) {
    double distX = fieldPose.getX() - robot.getX();
    double distY = fieldPose.getY() - robot.getY();
    double angleRadians = Math.atan2(distY, distX);

    return Math.toDegrees(angleRadians);
  }

  public void setThrottle(int throttle) {
    for (LimelightCamera camera : m_cameras) {
      camera.m_table.getEntry("throttle_set").setDouble(throttle);
    }
  }

  public static Pose2d[] getFieldTags(AprilTagFieldLayout field) {
    Pose2d[] output = new Pose2d[field.getTags().size() + 1];
    for (int i = 0; i <= field.getTags().size(); i++) {
      output[i] = field.getTagPose(i).isPresent() ? field.getTagPose(i).get().toPose2d() : new Pose2d();
    }
    return output;
  }

  private Optional<VisionMeasurement> getLimelightData(NetworkTable limelightTable, DoubleArraySubscriber botPoseSub) {
    Optional<VisionMeasurement> output = Optional.empty();

    double[] robotOrientationEntries = new double[6];
    robotOrientationEntries[0] = 0;
    robotOrientationEntries[1] = 0;
    robotOrientationEntries[2] = 0;
    robotOrientationEntries[3] = 0;
    robotOrientationEntries[4] = 0;
    robotOrientationEntries[5] = 0;

    limelightTable.getEntry("robot_orientation_set").setDoubleArray(robotOrientationEntries);

    double[] botPose = null;
    double latency = 0;
    double timeStamp = 0;

    for (TimestampedDoubleArray tsValue : botPoseSub.readQueue()) {
      botPose = tsValue.value;
      if ((int) botPose[7] == 0) {
        continue;
      }
      timeStamp = tsValue.timestamp;
      double botPosX = botPose[0];
      double botPosY = botPose[1];
      // double botPosZ = botpose[2];
      // double botRotX = botpose[3];
      // double botRotY = botpose[4]; // TODO: Store botPosZ, botRotX, and botRotY
      // somewhere (Store 3D)
      double botRotZ = botPose[5];
      latency = botPose[6];
      int targetAprilTagID = (int) botPose[11];

      double adjustedTimeStamp = (timeStamp / 1000000.0) - (latency / 1000.0);

      // timeStamp -= latency / 1000;
      Pose2d visionPose = new Pose2d(botPosX, botPosY, new Rotation2d(Math.toRadians(botRotZ)));
      if (0 <= targetAprilTagID
          && targetAprilTagID < LimelightConstants.aprilTagList.length) {
        limelightTable.putValue("botPosX", NetworkTableValue.makeDouble(botPosX));
        limelightTable.putValue("botPosY", NetworkTableValue.makeDouble(botPosY));
        limelightTable.putValue("botPosZ", NetworkTableValue.makeDouble(botRotZ));
        if (botPosX != 0 || botPosY != 0 || botRotZ != 0) {
          Rotation2d aprilTagToRobotAngle = Rotation2d.fromDegrees(
              getAngleToPose(visionPose, LimelightConstants.aprilTagList[targetAprilTagID]) - 180);
          limelightTable.putValue("Angle to pose", NetworkTableValue
              .makeDouble(getAngleToPose(visionPose, LimelightConstants.aprilTagList[targetAprilTagID]) - 180));
          double angleDiff = Math.abs(LimelightConstants.aprilTagList[targetAprilTagID].getRotation()
              .minus(aprilTagToRobotAngle)
              .getRadians());
          // System.out.println("saw apriltag: " + timeStamp + " latency: " + latency);
          output = Optional.of(new VisionMeasurement(visionPose, adjustedTimeStamp, targetAprilTagID, angleDiff));
        }
      }
    }
    return output;
  }

  public ArrayList<VisionMeasurement> getAllLimelightData() {
    ArrayList<VisionMeasurement> allLimelightData = new ArrayList<VisionMeasurement>();
    for (LimelightCamera camera : m_cameras) {
      Optional<VisionMeasurement> data = getLimelightData(camera.m_table, camera.m_sub);
      if (data.isPresent()) {
        allLimelightData.add(data.get());
      }
    }
    return allLimelightData;
  }

  @Override
  public void periodic() {

    }
}