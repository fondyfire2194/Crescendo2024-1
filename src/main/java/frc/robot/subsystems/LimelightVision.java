// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.utils.LLPipelines;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */
  boolean showFrontLeft = true;
  boolean showFrontRight = false;
  boolean showRearCamera = true;
  int columnIndex = 0;

  private double llHeartbeatfl;
  private double llHeartbeatLastfl;
  private int samplesfl;
  public boolean limelightExistsfl;

  private double llHeartbeatfr;
  private double llHeartbeatLastfr;
  private int samplesfr;
  public boolean limelightExistsfr;

  private double llHeartbeatr;
  private double llHeartbeatLastr;
  private int samplesr;
  public boolean limelightExistsr;
  private int loopctr;

  public LimelightVision() {

    if (showFrontLeft && CameraConstants.frontLeftCamera.isUsed && limelightExistsfl) {

      Shuffleboard.getTab("VisionSubsystem")
          .addString("LLName FL", () -> CameraConstants.frontLeftCamera.camname)
          .withPosition(columnIndex, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Pipeline # FL",
              () -> LimelightHelpers.getCurrentPipelineIndex(CameraConstants.frontLeftCamera.camname))
          .withPosition(columnIndex + 1, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addBoolean("HasTarget FL", () -> getTV(CameraConstants.frontLeftCamera))
          .withPosition(columnIndex + 2, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("# Tags FL", () -> getNumberTagsSeen(CameraConstants.frontLeftCamera))
          .withPosition(columnIndex + 3, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addString("Tags Seen FL", () -> getTagsSeen(CameraConstants.frontLeftCamera))
          .withPosition(columnIndex, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("TagID FL", () -> getTagId(CameraConstants.frontLeftCamera))
          .withPosition(columnIndex + 1, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Dist To Tag FL", () -> round2dp(getDistanceFromTag(CameraConstants.frontLeftCamera), 2))
          .withPosition(columnIndex + 2, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addString("BluePose FL",
              () -> LimelightHelpers.getBotPose3d_wpiBlue(CameraConstants.frontLeftCamera.camname).toPose2d()
                  .toString())
          .withPosition(columnIndex, 2)
          .withSize(4, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addString("TagPose FL", () -> getTagPose(CameraConstants.frontLeftCamera).toPose2d().toString())
          .withPosition(columnIndex, 3)
          .withSize(4, 1);
    }

    if (showFrontRight && CameraConstants.frontRightCamera.isUsed && limelightExistsfr) {

      // front right camera
      columnIndex = 4;
      Shuffleboard.getTab("VisionSubsystem")
          .addString("LLName FR", () -> CameraConstants.frontRightCamera.camname)
          .withPosition(columnIndex, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Pipeline # FR",
              () -> LimelightHelpers.getCurrentPipelineIndex(CameraConstants.frontRightCamera.camname))
          .withPosition(columnIndex + 1, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addBoolean("HasTarget FR  ", () -> getTV(CameraConstants.frontRightCamera))
          .withPosition(columnIndex + 2, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("# Tags FR", () -> getNumberTagsSeen(CameraConstants.frontRightCamera))
          .withPosition(columnIndex + 3, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addString("Tags Seen FR", () -> getTagsSeen(CameraConstants.frontRightCamera))
          .withPosition(columnIndex, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("TagID FR", () -> getTagId(CameraConstants.frontRightCamera))
          .withPosition(columnIndex + 1, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Dist To Tag FR", () -> round2dp(getDistanceFromTag(CameraConstants.frontRightCamera), 2))
          .withPosition(columnIndex + 2, 1)
          .withSize(1, 1);

    }
    // rear camera

    if (showRearCamera && CameraConstants.rearCamera.isUsed && limelightExistsr) {

      columnIndex = 8;
      Shuffleboard.getTab("VisionSubsystem")
          .addString("LLName R", () -> CameraConstants.rearCamera.camname)
          .withPosition(columnIndex, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Pipeline # R",
              () -> LimelightHelpers.getCurrentPipelineIndex(CameraConstants.rearCamera.camname))
          .withPosition(columnIndex + 1, 0)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addBoolean("HasTarget R  ", () -> getTV(CameraConstants.rearCamera))
          .withPosition(columnIndex, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Ta R",
              () -> round2dp(LimelightHelpers.getTA(CameraConstants.rearCamera.camname), 2))
          .withPosition(columnIndex + 1, 1)
          .withSize(1, 1);

      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Tx R",
              () -> round2dp(LimelightHelpers.getTX(CameraConstants.rearCamera.camname), 2))

          .withPosition(columnIndex, 2)
          .withSize(1, 1);
      Shuffleboard.getTab("VisionSubsystem")
          .addNumber("Ty R",
              () -> round2dp(LimelightHelpers.getTY(CameraConstants.rearCamera.camname), 2))
          .withPosition(columnIndex + 1, 2)
          .withSize(1, 1);
    }

    setNoteDetectorPipeline();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (loopctr >= 2)
      loopctr = 0;
    loopctr++;
    if (RobotBase.isReal()) {
      if (CameraConstants.frontLeftCamera.isUsed && loopctr == 0) {
        llHeartbeatfl = LimelightHelpers.getLimelightNTDouble(CameraConstants.frontLeftCamera.camname, "hb");
        if (llHeartbeatfl == llHeartbeatLastfl) {
          samplesfl += 1;
        } else {
          samplesfl = 0;
          llHeartbeatLastfl = llHeartbeatfl;
          limelightExistsfl = true;
        }
        if (samplesfl > 5)
          limelightExistsfl = false;

        CameraConstants.frontLeftCamera.isActive = limelightExistsfl;
      }
      if (CameraConstants.frontRightCamera.isUsed && loopctr == 1) {
        llHeartbeatfr = LimelightHelpers.getLimelightNTDouble(CameraConstants.frontRightCamera.camname, "hb");
        if (llHeartbeatfr == llHeartbeatLastfr) {
          samplesfr += 1;
        } else {
          samplesfr = 0;
          llHeartbeatLastfr = llHeartbeatfr;
          limelightExistsfr = true;
        }
        if (samplesfr > 5)
          limelightExistsfr = false;

        CameraConstants.frontRightCamera.isActive = limelightExistsfr;
      }

      if (CameraConstants.rearCamera.isUsed && loopctr == 2) {
        llHeartbeatr = LimelightHelpers.getLimelightNTDouble(CameraConstants.rearCamera.camname, "hb");
        if (llHeartbeatr == llHeartbeatLastr) {
          samplesr += 1;
        } else {
          samplesr = 0;
          llHeartbeatLastr = llHeartbeatr;
          limelightExistsr = true;
        }
        if (samplesr > 5)
          limelightExistsr = false;

        CameraConstants.rearCamera.isActive = limelightExistsr;
      }
    }

  }

  public void setAprilTag_ALL_Pipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL.ordinal());
  }

  public void setAprilTagStartPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGSTART.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGSTART.ordinal());
  }

  public void setNoteDetectorPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTE_DETECT.ordinal());
  }

  public void setAlignSpeakerPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALIGN.ordinal());
  }

  public int getNumberTagsSeen(CameraConstants.CameraValues cam) {
    return (int) LimelightHelpers
        .getLatestResults(cam.camname).targetingResults.targets_Fiducials.length;
  }

  public String getTagsSeen(CameraConstants.CameraValues cam) {
    var temp = LimelightHelpers.getLatestResults(cam.camname).targetingResults;

    var temp1 = temp.targets_Fiducials;
    int l = temp1.length;
    if (l <= 0)
      return String.valueOf(0);
    if (l == 1)
      return String.valueOf((int) temp1[0].fiducialID);
    if (l >= 2)
      return String.valueOf((int) temp1[0].fiducialID) + " , " + String.valueOf((int) temp1[1].fiducialID);
    else
      return "Problem";
  }

  public int getNumberNotesSeen() {
    return (int) LimelightHelpers
        .getLatestResults(CameraConstants.rearCamera.camname).targetingResults.targets_Detector.length;
  }

  public boolean getNoteSeen() {
    return LimelightHelpers.getTV(CameraConstants.rearCamera.camname);
  }

  public int getTagId(CameraConstants.CameraValues cam) {
    return (int) LimelightHelpers.getFiducialID(cam.camname);
  }

  public boolean getTV(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getTV(cam.camname);
  }

  public Pose3d getTagPose3d(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public Pose3d getAnyTagPose3d(int tagID) {
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public double getDistanceFromTag(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    return getTranslationFromTag(cam, tagID).getNorm();
  }

  public boolean hasTarget(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getTV(cam.camname);
  }

  public void setCamToRobotOffset(CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

  public double[] getTargetPoseRobotSpaceAsDoubles(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getTargetPose_RobotSpace(cam.camname);
    else
      return temp;
  }

  public double[] getCameraPoseTargetSpaceasDoubles(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getCameraPose_TargetSpace(cam.camname);
    else
      return temp;
  }

  public double[] getTargetPoseCameraSpace(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getTargetPose_CameraSpace(cam.camname);
    else
      return temp;
  }

  public Pose3d getCameraPoseTargetSpace3d(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getCameraPose3d_RobotSpace(cam.camname);
  }

  public double[] getBotPoseWPI_BlueAsDoubles(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose_wpiBlue(cam.camname);
  }

  public double[] getBotPoseWPI_RedAsDoubles(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose_wpiRed(cam.camname);
  }

  public Pose3d getBotPose3d(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose3d(cam.camname);
  }

  public Pose3d getBotPose3dTargetSpace(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose3d_TargetSpace(cam.camname);
  }

  public Pose3d getTagPose(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    // SmartDashboard.putNumber("SHID", tagID);
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public Translation2d getTranslationFromTag(CameraConstants.CameraValues cam, int tagid) {

    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagid);

    if (aprilTagPose.isPresent()) {

      Pose2d tagPose2d = aprilTagPose.get().toPose2d();

      // // get botpose from limelight networktables

      Pose2d botPose2d = LimelightHelpers.getBotPose2d_wpiBlue(cam.camname);

      Translation2d botToTagTranslation2d = botPose2d.getTranslation().minus(tagPose2d.getTranslation());

      return botToTagTranslation2d;

    } else

      return new Translation2d();
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}
