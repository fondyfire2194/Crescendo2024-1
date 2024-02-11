// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GeometryUtil;

public class LimelightSetStartPose extends Command {
  /** Creates a new LimelightSetStartPose. */

  private final LimelightSubsystem m_ll;
  private final SwerveSubsystem m_swerve;
  private final Pose2d m_pathStartPose;
  private boolean redAlliance;
  private boolean blueAlliance;
  private Pose2d useAsStartPose;
  private double startTime;
  private double allowedTime = .75;
  private boolean endIt;
  private int loopctr;
  private int validResults = 3;

  public LimelightSetStartPose(LimelightSubsystem ll, SwerveSubsystem swerve, Pose2d pathStartPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ll = ll;
    m_swerve = swerve;
    m_pathStartPose = pathStartPose;
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

    if (redAlliance)
      m_ll.setAprilTagStartPipelineRed();
    if (blueAlliance)
      m_ll.setAprilTagStartPipelineRed();

    useAsStartPose = m_pathStartPose;

    if (redAlliance)
      useAsStartPose = GeometryUtil.flipFieldPose(m_pathStartPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LimelightResults llr = m_ll.getLatestResults();

    if (llr.targetingResults.valid) {
      loopctr++;

      Pose2d botPose2d = llr.targetingResults.getBotPose2d();

      useAsStartPose = botPose2d;

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
