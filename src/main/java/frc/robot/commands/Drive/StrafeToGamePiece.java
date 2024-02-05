// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import org.opencv.features2d.FlannBasedMatcher;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class StrafeToGamePiece extends Command {

  private LimelightSubsystem ll;
  private SwerveSubsystem drivetrain;
  private PIDController yController = new PIDController(0.1, 0, 0);

  public StrafeToGamePiece(SwerveSubsystem drivetrain, LimelightSubsystem Limelight) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    ll = Limelight;
  }

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(Constants.SwerveConstants.maxSpeed * 0.01)
      .withRotationalDeadband(Constants.SwerveConstants.maxAngularVelocity * 0.01)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final double thetaOutput = 0;
  private final double xOutput = 0.2; // Speed to drive towards note will increase after testing
  private double yOutput = 0;
  private final double setpoint = 7; // How far the camera is offset from the center in degrees

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yController.reset();
    yController.setTolerance(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ll.hasTarget()) {
      yOutput = yController.calculate(-ll.getTX(), setpoint);
    } else {
      yOutput = 0;
    }

    drivetrain.drive(-xOutput * SwerveConstants.maxSpeed, yOutput * SwerveConstants.maxAngularVelocity,
        thetaOutput * SwerveConstants.maxAngularVelocity, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}