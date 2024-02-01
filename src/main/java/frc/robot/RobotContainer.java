// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CenterStartCommandFactory;
import frc.robot.commands.LoadAndRunPPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final TopShooterRollerSubsystem m_topShooter = new TopShooterRollerSubsystem();

        final BottomShooterRollerSubsystem m_bottomShooter = new BottomShooterRollerSubsystem();

        final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

        final HoldNoteSubsystem m_holdNote = new HoldNoteSubsystem();

        final LimelightSubsystem m_llv1 = new LimelightSubsystem("limelight");

        final LimelightSubsystem m_llv2 = new LimelightSubsystem("limelight_1");

        final LimelightSubsystem m_llv3 = new LimelightSubsystem("limelight_2");

        public final CenterStartCommandFactory m_cf = new CenterStartCommandFactory(m_swerve, m_elevator, m_intake,
                        m_holdNote,
                        m_topShooter, m_bottomShooter);

        public final AutoFactory m_af = new AutoFactory(m_cf, m_swerve);

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        public RobotContainer() {

                Pref.deleteUnused();

                Pref.addMissing();

                setDefaultCommands();

                registerNamedCommands();

                configureBindings();

                // Set the scheduler to log Shuffleboard events for command initialize,
                // interrupt, finish
                CommandScheduler.getInstance()
                                .onCommandInitialize(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command initialized", command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command interrupted", command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandFinish(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command finished", command.getName(),
                                                                EventImportance.kNormal));

        }

        BooleanSupplier fieldCentric = driver.leftBumper();

        private void setDefaultCommands() {

                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_swerve,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> -driver.getRightX(),
                                                fieldCentric));

                // m_intake.setDefaultCommand(Commands.runOnce(() -> m_intake.stopMotor(),
                // m_intake));
                // m_shooter.setDefaultCommand(Commands.runOnce(() -> m_shooter.stopMotors(),
                // m_shooter));
                // m_llv1.setDefaultCommand(new TrackAprilTags3D(m_llv1, m_swerve));
                // m_llv2.setDefaultCommand(new TrackAprilTags3D(m_llv2, m_swerve));
        }

        private void registerNamedCommands() {

        }

        private void configureBindings() {

                SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());

                driver.y().onTrue(m_swerve.setPoseToX0Y0());

                driver.b().onTrue(m_intake.runIntakeCommand())
                                .onFalse(m_intake.stopIntakeCommand());

                driver.x().onTrue(new LoadAndRunPPath(m_swerve, "AutoOneP1", true));

                driver.y().onTrue(new LoadAndRunPPath(m_swerve, "AutoOneP2", false));

                driver.back().onTrue(new LoadAndRunPPath(m_swerve, "AutoOneP3", false));

                codriver.leftBumper().onTrue(m_intake.runIntakeCommand());

                codriver.leftTrigger().onTrue(m_intake.stopIntakeCommand());

                codriver.rightBumper().onTrue(m_topShooter.runTopRollerCommand())
                                .onTrue(m_bottomShooter.runBottomRollerCommand());

                codriver.rightTrigger().onTrue(m_topShooter.stopShootersCommand())
                                .onTrue(m_bottomShooter.stopShootersCommand());

                SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
                                new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(0)),
                                new PathConstraints(
                                                3.0, 3.0,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540)),
                                0,
                                2.0));

                SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
                                new Pose2d(1.15, 1.0, Rotation2d.fromDegrees(180)),
                                new PathConstraints(
                                                3.0, 3.0,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540)),
                                0,
                                0));

                // Add a button to SmartDashboard that will create and follow an on-the-fly path
                // This example will simply move the robot 2m in the +X field direction
                SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {

                        Pose2d currentPose = m_swerve.getPose();

                        // The rotation component in these poses represents the direction of travel
                        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
                        Pose2d endPos = new Pose2d(currentPose.getTranslation()
                                        .plus(new Translation2d(2.0, 0.0)), new Rotation2d());

                        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
                        PathPlannerPath path = new PathPlannerPath(
                                        bezierPoints,
                                        new PathConstraints(
                                                        3.0, 3.0,
                                                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                                        new GoalEndState(0.0, currentPose.getRotation()));

                        path.preventFlipping = true;

                        AutoBuilder.followPath(path).schedule();
                }));
        }

        public Command getSinglePathCommandWithPresetStart(String pathname) {

                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

                Pose2d startPose = path.getPreviewStartingHolonomicPose();
                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.
                return m_swerve.setPose(startPose).andThen(

                                AutoBuilder.followPath(path));
        }

}
