// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LLPipelines;

public class TrackNote extends Command {
  /** Creates a new TrackNote. */
 
  private final SwerveSubsystem m_drive;
  private final String m_llName;

  public TrackNote(String llName,SwerveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    m_llName=llName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   LimelightHelpers.setPipelineIndex(m_llName, LLPipelines.pipelines.NOTE_DETECT.ordinal());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = false;
    double classid = 999;
    double tx = 0;
    double ty = 0;
    double tarea = 0;
    double latencyms = 0;

     hasTarget = LimelightHelpers.getTV(m_llName);

    if (hasTarget) {

      classid = LimelightHelpers.getNeuralClassID(m_llName);

      if (classid == 0) {

        tx = LimelightHelpers.getTX(m_llName);

        ty = LimelightHelpers.getTY(m_llName);

        tarea = LimelightHelpers.getTA(m_llName);

        latencyms = LimelightHelpers.getLatency_Pipeline(m_llName);
      }
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
