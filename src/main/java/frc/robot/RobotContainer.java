// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TrackAprilTags3D;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  /* Subsystems */
  final SwerveSubsystem m_swerve = new SwerveSubsystem();

  final IntakeSubsystem m_intake = new IntakeSubsystem();

  final ShooterSubsystem m_shooter = new ShooterSubsystem();

  final LimelightSubsystem m_llv1 = new LimelightSubsystem("limelight");

  final LimelightSubsystem m_llv2 = new LimelightSubsystem("limelight_1");

  final LimelightSubsystem m_llv3 = new LimelightSubsystem("limelight_2");

  public final AutoFactory m_cf = new AutoFactory(null, m_swerve, m_intake, m_shooter);

  private BooleanSupplier robotCentric = () -> true;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final CommandXboxController codriver = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    Pref.deleteUnused();

    Pref.addMissing();

    setDefaultCommands();

    registerNamedCommands();

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void setDefaultCommands() {

    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -driver.getLeftX(),
            () -> -driver.getLeftY(),
            () -> -driver.getRightX() / 6,
            () -> robotCentric.getAsBoolean()));

    m_intake.setDefaultCommand(Commands.runOnce(() -> m_intake.stopMotor(), m_intake));
    m_shooter.setDefaultCommand(Commands.runOnce(() -> m_shooter.stopMotors(), m_shooter));
    m_llv1.setDefaultCommand(new TrackAprilTags3D(m_llv1, m_swerve));
    m_llv2.setDefaultCommand(new TrackAprilTags3D(m_llv2, m_swerve));
  }

  private void registerNamedCommands() {

  }

  private void configureBindings() {

    driver.y().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

    codriver.leftBumper().onTrue(m_intake.runIntakeCommand());

    codriver.leftTrigger().onTrue(m_intake.stopIntakeCommand());

    codriver.rightBumper().onTrue(m_shooter.runTopRollerCommand())
        .onTrue(m_shooter.runBottomRollerCommand());

    codriver.rightTrigger().onTrue(m_shooter.stopShootersCommand());

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        2.0));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {

      Pose2d currentPose = m_swerve.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

}