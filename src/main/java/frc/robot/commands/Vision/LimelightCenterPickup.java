// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LLPipelines;

public class LimelightCenterPickup extends Command {
  /** Creates a new LimelightSetStartPose. */

  private final String m_llName;
  private final SwerveSubsystem m_swerve;
  private final PathPlannerPath m_path;
  private final Pose2d m_targetPose;
  private boolean redAlliance;
  private boolean blueAlliance;
  private Pose2d useAsStartPose;
  private double startTime;
  private double allowedTime = .75;
  private boolean endIt;
  private int loopctr;
  private int validResults = 3;
  private int notePipeline = 8;

  public LimelightCenterPickup(String llName, SwerveSubsystem swerve, PathPlannerPath path, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llName = llName;
    m_swerve = swerve;
    m_path = path;
    m_targetPose = targetPose;
    startTime = Timer.getFPGATimestamp();
    endIt = false;
    loopctr = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    redAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
    blueAlliance = alliance.isPresent() && alliance.get() == Alliance.Blue;

    LimelightHelpers.setPipelineIndex(m_llName, LLPipelines.pipelines.NOTE_DETECT.ordinal());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LimelightResults llr = LimelightHelpers.getLatestResults(m_llName);

    if (llr.targetingResults.valid) {
      loopctr++;

      boolean llhastarget = LimelightHelpers.getTV(m_llName);

      if (llhastarget) {

        double tx = LimelightHelpers.getTX(m_llName);

        double ty = LimelightHelpers.getTY(m_llName);

        double ta = LimelightHelpers.getTA(m_llName);

        if (m_swerve.getLookForNote()) {

          Pose2d currentPose = m_swerve.getPose();

          double currentPoseY = currentPose.getY();

          double targetPoseY = m_targetPose.getY();

        }
      }

    }

    else
      loopctr = 0;

    if (loopctr > validResults && Timer.getFPGATimestamp() > startTime + allowedTime) {
      m_swerve.setPose(useAsStartPose);
      endIt = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
