// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackAprilTags3D extends Command {
  /** Creates a new TrackNote. */
  private final LimelightSubsystem m_llv;
  private final SwerveSubsystem m_drive;
  private String m_name = null;

  public TrackAprilTags3D(LimelightSubsystem llv, SwerveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llv = llv;
    m_drive = drive;
    addRequirements(m_llv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_name = m_llv.getLLName();

    m_llv.setAprilTag3D_Pipeline();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(m_name);
    Pose3d robPose3d = new Pose3d();
    Pose2d robPose2d = new Pose2d();
    Pose3d robPose3dTargetSpace = new Pose3d();

    if (hasTarget) {

      robPose3d = LimelightHelpers.getBotPose3d(m_name);

      robPose2d = LimelightHelpers.getBotPose2d(m_name);

      robPose3dTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace(m_name);

      if (m_drive.isStopped())

        m_drive.addVisionMeasurement(robPose2d, Timer.getFPGATimestamp());

    }
    SmartDashboard.putBoolean(m_name + " hasTarget", hasTarget);

    SmartDashboard.putNumber(m_name +" TagID", LimelightHelpers.getFiducialID(m_name));

    SmartDashboard.putNumber(m_name +" #TagsSeen",
     LimelightHelpers.getLatestResults(m_name).targetingResults.targets_Fiducials.length);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
