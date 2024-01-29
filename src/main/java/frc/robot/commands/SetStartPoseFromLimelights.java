// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SetStartPoseFromLimelights extends Command {
  /** Creates a new SetStartPoseFromLimelights. */
  private final SwerveSubsystem m_drive;
  private final LimelightSubsystem m_llv;
  public SetStartPoseFromLimelights(SwerveSubsystem drive,LimelightSubsystem llv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive=drive;
    m_llv=llv;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
