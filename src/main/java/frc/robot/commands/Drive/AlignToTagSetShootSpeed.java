// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants.CameraValues;
import frc.robot.LimelightHelpers;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTagSetShootSpeed extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);

  private final SwerveSubsystem m_swerve;
  private final CameraValues m_camval;
  private final LimelightVision m_llv;
  private final CommandFactory m_cf;

  private double rotationVal;

  public AlignToTagSetShootSpeed(
      SwerveSubsystem swerve,
      LimelightVision llv,
      CommandFactory cf,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      CameraValues camval)

  {
    m_swerve = swerve;
    m_llv = llv;
    m_cf = cf;
    m_camval = camval;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setAlignSpeakerPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    // get horizontal angle

    double angleError = -LimelightHelpers.getTX(m_camval.camname);

    // get distance for shooter speed and angle

    double distanceUsingTag = m_llv.getDistanceFromTag(m_camval);

    double distanceUsingTrig = m_llv.getSpeakerDistance(m_camval);

    double[] shooterSpeeds = m_cf.getShooterSpeedsFromDistance(distanceUsingTag);

    /* Drive */
    s_Swerve.drive(
        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        strafeVal *= Constants.SwerveConstants.kmaxSpeed,
        rotationVal = m_swerve.getAlignPID().calculate(angleError, 0),
        false,
        true,
        false);

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
