// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.LimelightVision;

public class TrackAprilTags3D extends Command {
  /** Creates a new TrackNote. */
  private final LimelightVision m_llv;
  private final Swerve m_drive;
  private final String m_llname;

  public TrackAprilTags3D(LimelightVision llv, String llname, Swerve drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llv = llv;
    m_drive=drive;
    m_llname=llname;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_llv.setActiveCamera(0);
    m_llv.setAprilTag3D_Pipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = false;
    Pose3d robPose3d = new Pose3d();
    Pose2d robPose2d = new Pose2d();
    Pose3d robPose3dTargetSpace = new Pose3d();

    if (hasTarget) {

      robPose3d = LimelightHelpers.getBotPose3d(m_llname);

      robPose2d = LimelightHelpers.getBotPose2d(m_llname);

      robPose3dTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace(m_llname);

      m_drive.addVisionMeasurement(robPose2d,Timer.getFPGATimestamp());

    }

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
