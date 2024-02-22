// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathplanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNoteAndCorrectPath extends Command {
  /** Creates a new TrackNoteAndCorrectPath. */
  private final SwerveSubsystem m_swerve;

  public TrackNoteAndCorrectPath(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean notePresent = LimelightHelpers.getTV(CameraConstants.rearCamera.camname);

    double noteSeenAngle = 0;

    if (notePresent)
      LimelightHelpers.getTX(CameraConstants.rearCamera.camname);

    boolean targetNoteSeen = Math.abs(noteSeenAngle) < 1;

    Pose2d currentpose = m_swerve.getPose();

    Pose2d notePose = new Pose2d(0, noteSeenAngle, new Rotation2d());

    m_swerve.getPoseEstimator().addVisionMeasurement(notePose, .1);

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
