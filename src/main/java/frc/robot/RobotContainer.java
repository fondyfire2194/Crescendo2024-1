// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Pathplanner.LoadAndRunPPath;
import frc.robot.subsystems.LeftShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeedShooterSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RightShooterRollerSubsystem;

public class RobotContainer {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final RightShooterRollerSubsystem m_rightShooter = new RightShooterRollerSubsystem();

        final LeftShooterRollerSubsystem m_leftShooter = new LeftShooterRollerSubsystem();

        final FeedShooterSubsystem m_shooterFeeder = new FeedShooterSubsystem();

        final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

        final HoldNoteSubsystem m_holdNote = new HoldNoteSubsystem();

        final ShooterAngleSubsystem m_shooterAngle = new ShooterAngleSubsystem();

        final CommandFactory m_cf = new CommandFactory(m_swerve, m_intake, m_elevator,
                        m_holdNote, m_shooterAngle, m_rightShooter, m_leftShooter,m_shooterFeeder);

        // final LimelightSubsystem m_llv1 = new LimelightSubsystem("limelight");

        // final LimelightSubsystem m_llv2 = new LimelightSubsystem("limelight_1");

        // final LimelightSubsystem m_llv3 = new LimelightSubsystem("limelight_2");

        public final AutoFactory m_af = new AutoFactory(m_cf, m_swerve, m_elevator, m_intake, m_holdNote,
                        m_rightShooter,
                        m_leftShooter, m_shooterAngle);

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
                                                () -> -driver.getRightX() / 4,
                                                fieldCentric));

                m_elevator.setDefaultCommand(m_elevator.positionHold());

                m_shooterAngle.setDefaultCommand(m_shooterAngle.positionHold());

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

                // SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());

                driver.y().onTrue(m_swerve.setPoseToX0Y0());

                driver.x().onTrue(new LoadAndRunPPath(m_swerve, "TestPath1", true));

                driver.a().onTrue(new LoadAndRunPPath(m_swerve, "TestPath2", true));

                driver.b().onTrue(new LoadAndRunPPath(m_swerve, "TestPath3", true));

                driver.back().onTrue(m_intake.runIntakeCommand())
                                .onFalse(m_intake.stopIntakeCommand());

                // driver.start()

                codriver.leftBumper().onTrue(m_intake.runIntakeCommand());

                codriver.leftTrigger().onTrue(m_intake.stopIntakeCommand());

                //codriver.rightBumper()

                codriver.rightTrigger().onTrue(m_rightShooter.stopShooterCommand())
                                .onTrue(m_leftShooter.stopShooterCommand());

                codriver.y().whileTrue(m_elevator.jogCommand(.2))
                                .onFalse(Commands.runOnce(() -> m_elevator.stopMotor()));

                codriver.a().whileTrue(m_shooterAngle.jogCommand(-.2))
                                .onFalse(Commands.runOnce(() -> m_shooterAngle.stopMotor()));

                codriver.x().whileTrue(m_shooterAngle.jogCommand(.2))
                                .onFalse(Commands.runOnce(() -> m_shooterAngle.stopMotor()));

                codriver.b().whileTrue(m_shooterAngle.jogCommand(-.2))
                                .onFalse(Commands.runOnce(() -> m_elevator.stopMotor()));

        }

}
