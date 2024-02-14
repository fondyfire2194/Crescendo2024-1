// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CameraConstants;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */
  public LimelightVision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("CPDTS", getCameraPoseTargetSpace3d(CameraConstants.frontRightCamName).toString());
    SmartDashboard.putNumberArray("CPDTSD", getCameraPoseTargetSpaceasDoubles(CameraConstants.frontRightCamName, 8));
    SmartDashboard.putNumberArray("CTPRSD", getTargetPoseRobotSpaceAsDoubles(CameraConstants.frontRightCamName, 8));
    SmartDashboard.putNumber("CTDIST", getTranslationFromTag(CameraConstants.frontRightCamName, 4
    ).getNorm());
    // SmartDashboard.putNumber("CTDIAGDID",
    // getCameraPoseTargetSpaceasDoubles(CameraConstants.frontRightCamName, 8)[2]);
    SmartDashboard.putNumber("CTDIAGDID2", getTargetPoseCameraSpace(CameraConstants.frontRightCamName, 8)[2]);

  }

  public boolean hasTarget(String camname) {
    return LimelightHelpers.getTV(camname);

  }

  public void setCamToRobotOffset(String camname, Transform3d offset) {
    Translation3d tempt = offset.getTranslation();
    Rotation3d tempr = offset.getRotation();
    // LimelightHelpers.setCameraPose_RobotSpace(camname, tempt.getX(),
    // tempt.getY(), tempt.getZ(), tempr.getX(),
    // tempr.getY(), tempr.getZ());
    LimelightHelpers.setCameraPose_RobotSpace(camname, 1, 2, 3, .3,
        .09, .17);
  }

  public double[] getTargetPoseRobotSpaceAsDoubles(String camname, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(camname) && LimelightHelpers.getFiducialID(camname) == id)
      return LimelightHelpers.getTargetPose_RobotSpace(camname);
    else
      return temp;
  }

  public double[] getCameraPoseTargetSpaceasDoubles(String camname, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(camname) && LimelightHelpers.getFiducialID(camname) == id)
      return LimelightHelpers.getCameraPose_TargetSpace(camname);
    else
      return temp;
  }

  public double[] getTargetPoseCameraSpace(String camname, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(camname) && LimelightHelpers.getFiducialID(camname) == id)
      return LimelightHelpers.getTargetPose_CameraSpace(camname);
    else
      return temp;
  }

  public Pose3d getCameraPoseTargetSpace3d(String camname) {
    return LimelightHelpers.getCameraPose3d_RobotSpace(camname);
  }

  public double[] getBotPoseWPI_BlueAsDoubles(String camname) {
    return LimelightHelpers.getBotPose_wpiBlue(camname);
  }

  public double[] getBotPoseWPI_RedAsDoubles(String camname) {
    return LimelightHelpers.getBotPose_wpiRed(camname);
  }

  public Pose3d getBotPose3d(String camname) {
    return LimelightHelpers.getBotPose3d(camname);
  }

  public Pose3d getBotPose3dTargetSpace(String camname) {
    return LimelightHelpers.getBotPose3d_TargetSpace(camname);
  }

  public Pose3d getTagPose(String camname, int tagid) {
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagid);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public Translation2d getTranslationFromTag(String camname, int tagid) {

    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagid);

    if (aprilTagPose.isPresent()) {

      Pose2d tagPose2d = aprilTagPose.get().toPose2d();

      // get botpose from limelight networktables

      Pose2d botPose2d = LimelightHelpers.getBotPose2d_wpiBlue(camname);

      Translation2d botToTagTranslatione2d = botPose2d.getTranslation().minus(tagPose2d.getTranslation());

      return botToTagTranslatione2d;

    } else

      return new Translation2d();
  }

}
