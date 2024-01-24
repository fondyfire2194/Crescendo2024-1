// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionPhoton extends SubsystemBase {

  PhotonCamera leftCam;
  PhotonCamera rightCam;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimatorLeft;
  Pose2d previousposeleft = new Pose2d();
  PhotonPoseEstimator photonPoseEstimatorRight;
  Pose2d previousposeright = new Pose2d();
  private int tst;

  public VisionPhoton() {
    leftCam = new PhotonCamera(CameraConstants.leftCamName);

    rightCam = new PhotonCamera(CameraConstants.rightCamName);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      SmartDashboard.putNumber("No Field Layout", 911);
    }
    aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    photonPoseEstimatorLeft = new PhotonPoseEstimator(
        aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, leftCam, CameraConstants.robotToLeftCam);

    photonPoseEstimatorRight = new PhotonPoseEstimator(
        aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, rightCam, CameraConstants.robotToRightCam);

  }

  Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator ppe, Pose2d prevEstimatedRobotPose) {
    ppe.setReferencePose(prevEstimatedRobotPose);
    return ppe.update();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> est = getEstimatedGlobalPose(photonPoseEstimatorLeft, previousposeleft);
    if (est.isPresent()) {
      EstimatedRobotPose est1 = est.get();
      previousposeleft = est1.estimatedPose.toPose2d();
      double timestampsecondsl = est1.timestampSeconds;
    }
    Optional<EstimatedRobotPose> est2 = getEstimatedGlobalPose(photonPoseEstimatorRight, previousposeright);
    if (est2.isPresent()) {
      EstimatedRobotPose est3 = est2.get();
      previousposeright = est3.estimatedPose.toPose2d();
      double timestampsecondsr = est3.timestampSeconds;
    }

    pose2dtosd("LeftCamPose", previousposeleft);
    pose2dtosd("RightCamPose", previousposeright);
    

    
  }

  void pose2dtosd(String name, Pose2d pose) {
    SmartDashboard.putNumberArray(name,
        new double[] { pose.getX(),
            pose.getY(), pose.getRotation().getDegrees() });

  }
}
