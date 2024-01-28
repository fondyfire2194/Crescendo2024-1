// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public Pose2d visionPoseEstimatedData;

  public double imageCaptureTime;

  public int fiducialId;

  public double[] llHeartbeat = { 0, 0 };

  public double[] llHeartbeatLast = { 0, 0 };

  public boolean allianceBlue;

  public enum pipelinetype {
    retroreflective,
    grip,
    python,
    fiducialmarkers,
    classifier,
    detector;

    public static final pipelinetype values[] = values();
  }

  //
  public enum pipelines {
    APRILTAG(0, pipelinetype.fiducialmarkers),
    APRILTAG3D(1, pipelinetype.fiducialmarkers),
    SPARE_2(2, pipelinetype.fiducialmarkers),
    SPART_3(3, pipelinetype.fiducialmarkers),
    SPARE_4(4, pipelinetype.fiducialmarkers),
    SPARE_5(5, pipelinetype.fiducialmarkers),
    GRIP_6(6, pipelinetype.grip),
    PYTHON_7(7, pipelinetype.python),
    NOTE_DETECT(8, pipelinetype.detector),
    SPARE_DETECT(9, pipelinetype.detector);

    public static final pipelines values[] = values();

    private pipelinetype type;

    public String pipelineTypeName;

    private int number;

    private pipelines(int number, pipelinetype type) {
      this.number = number;
      this.type = type;
    }

  }

  public int currentPipelineIndex;
  public pipelinetype currentPipelineType;
  public pipelines currentPipeline;

  private int[] samples = { 0, 0 };

  public boolean[] limelightExists = { false, false };

  public String limelighttypename = "fiducial";

  private String[] m_name = new String[2];

  private int numCameras = 2;

  public int activeLimelight;

  public String activeName;

  public LimelightVision(String[] name) {
    currentPipeline = pipelines.APRILTAG3D;
    m_name[0] = name[0];
    m_name[1] = name[1];

    activeLimelight = 0;

  }

  public boolean getAllianceBlue() {
    return (DriverStation.getAlliance().get() == Alliance.Blue);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ActLL", activeLimelight);
    activeName = m_name[activeLimelight];
    SmartDashboard.putString("LLAN", activeName);
    if (RobotBase.isReal()) {
      for (int i = 0; i < numCameras; i++) {
        llHeartbeat[i] = LimelightHelpers.getLimelightNTDouble(m_name[i], "hb");
        if (llHeartbeat[i] == llHeartbeatLast[i]) {
          samples[i] += 1;
        } else {
          samples[i] = 0;
          llHeartbeatLast[i] = llHeartbeat[i];
          limelightExists[i] = true;
        }
        if (samples[i] > 5)
          limelightExists[i] = false;
      }
    }
    // SmartDashboard.putBoolean("LLExists", limelightExists);

    if (RobotBase.isReal()) {

      fiducialId = (int) LimelightHelpers.getFiducialID(m_name[activeLimelight]);
      SmartDashboard.putNumber("FIDID", fiducialId);
      currentPipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex(m_name[activeLimelight]);

      currentPipeline = pipelines.values[currentPipelineIndex];

      currentPipelineType = currentPipeline.type;

      limelighttypename = getCurrentPipelineTypeName();
    }
  }

  public void setActiveCamera(int activeCam) {
    activeLimelight = activeCam;
    activeName = m_name[activeLimelight];
  }

  public double round2dp(double number) {
    number = Math.round(number * 100);
    number /= 100;
    return number;
  }

  public String getCurrentPipelineName() {
    return currentPipeline.name();
  }

  public String getCurrentPipelineTypeName() {
    return currentPipeline.type.name();
  }

  public pipelinetype getCurrentPipelineType() {
    return currentPipeline.type;
  }

  public pipelines getCurrentPipeline() {
    return currentPipeline;
  }

  public void setAprilTag_0_Pipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.APRILTAG.ordinal());
  }

  public void setAprilTag3D_Pipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.APRILTAG3D.ordinal());
  }

  public void setNoteDetectorPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.NOTE_DETECT.ordinal());
  }

  public double getSpeakerDistance() {

    //ll3 Field-of-View: 63.3 x 49.7 degrees

    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 56.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

}