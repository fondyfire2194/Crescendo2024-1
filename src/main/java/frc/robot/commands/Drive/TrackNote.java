// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackNote extends Command {
  /** Creates a new TrackNote. */
  private final LimelightSubsystem m_llv;
  private final SwerveSubsystem m_drive;
  private final String m_llname;

  public TrackNote(LimelightSubsystem llv, String llname,SwerveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llv = llv;
    m_drive = drive;
    m_llname=llname;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_llv.setNoteDetectorPipeline();
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

    hasTarget = LimelightHelpers.getTV(m_llname);

    if (hasTarget) {

      classid = LimelightHelpers.getNeuralClassID(m_llname);

      if (classid == 0) {

        tx = LimelightHelpers.getTX(m_llname);

        ty = LimelightHelpers.getTY(m_llname);

        tarea = LimelightHelpers.getTA(m_llname);

        latencyms = LimelightHelpers.getLatency_Pipeline(m_llname);
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
