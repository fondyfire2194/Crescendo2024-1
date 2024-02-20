// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Drive.AlignToTagSetShootSpeed;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final ShooterSubsystem m_shooter = new ShooterSubsystem();

        public final PathFactory m_pf = new PathFactory(m_swerve);

        final ShooterAngleSubsystem m_shooterAngle = new ShooterAngleSubsystem();
        public final LimelightVision m_llv = new LimelightVision();
        public final AutoFactory m_af = new AutoFactory(m_pf, m_swerve);
        final CommandFactory m_cf = new CommandFactory(m_pf, m_af, "ss", m_swerve, m_shooter, m_shooterAngle, m_intake,
                        m_llv);

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        final CommandJoystick tstjs = new CommandJoystick(2);

        public RobotContainer() {

                Pref.deleteUnused();

                Pref.addMissing();

                setDefaultCommands();

                registerNamedCommands();

                showOnShuffleboard();

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

        BooleanSupplier keepAngle = driver.rightBumper();

        private void setDefaultCommands() {

                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_swerve,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> -driver.getRawAxis(4),
                                                fieldCentric,
                                                keepAngle));

                // m_elevator.setDefaultCommand(m_elevator.positionHold());

                // m_shooterAngle.setDefaultCommand(m_shooterAngle.positionHold());

        }

        private void showOnShuffleboard() {

                m_swerve.showSwerve = false;
                m_shooter.showShooter = false;
                m_shooterAngle.showShooterAngle = false;
                m_intake.showIntake = false;
                m_llv.showFrontLeft = true;
                m_llv.showFrontRight = false;
                m_llv.showRearCamera = false;

        }

        private void registerNamedCommands() {
                // Register Named Commands

                NamedCommands.registerCommand("RunIntake",
                                Commands.runOnce(() -> SmartDashboard.putString("RunIntake", "")));

        }

        private void configureBindings() {

                driver.leftTrigger().whileTrue(
                                new AlignToTagSetShootSpeed(
                                                m_swerve,
                                                m_shooter,
                                                m_shooterAngle,
                                                m_llv,
                                                m_cf,
                                                () -> -driver.getLeftY(),
                                                () -> driver.getLeftX(),
                                                () -> driver.getRightX(),
                                                CameraConstants.frontLeftCamera));

                SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());

                tstjs.button(5).onTrue(m_shooter.testRunRollerCommand());

                tstjs.button(3).onTrue(Commands.runOnce(() -> m_shooter.stopMotors(), m_shooter));

                tstjs.button(2).onTrue(m_intake.intakeToSensorCommand());

                tstjs.button(1).onTrue(m_intake.feedShooterCommand()
                                .onlyIf(() -> m_shooter.leftAtSpeed() && m_shooter.rightAtSpeed()));

                // tstjs.povUp().onTrue(m_shooterAngle.jogCommand(.1))
                // .onFalse(Commands.runOnce(() -> m_shooterAngle.stopMotor()));

                // tstjs.povDown().onTrue(m_shooterAngle.jogCommand(.1))
                // .onFalse(Commands.runOnce(() -> m_shooterAngle.stopMotor()));

                // tstjs.button(6).onTrue(m_shooterAngle
                // .smartPositionShooterAngleCommandToAngle(ShooterAngleConstants.shooterangleMaxDegrees));

                // tstjs.button(4).onTrue(m_shooterAngle
                // .smartPositionShooterAngleCommandToAngle(ShooterAngleConstants.shooterangleMinDegrees));

                // tstjs.button(8).onTrue(Commands.runOnce(() ->
                // m_shooterAngle.incrementShooterAngle())
                // .andThen(m_shooterAngle.smartPositionShooterAngleCommand()));

                // tstjs.button(7).onTrue(Commands.runOnce(() ->
                // m_shooterAngle.decrementShooterAngle())
                // .andThen(m_shooterAngle.smartPositionShooterAngleCommand()));

                tstjs.button(10).onTrue(m_intake.runIntakeCommand());

                tstjs.button(9).onTrue(Commands.runOnce(() -> m_intake.stopMotor(), m_intake));

                tstjs.button(12).onTrue(Commands.runOnce(() -> m_intake.incrementIntakeRPM()));

                tstjs.button(11).onTrue(Commands.runOnce(() -> m_intake.decrementIntakeRPM()));

                // driver.x().onTrue(

                // driver.povUp().

                // driver.povLeft().onTrue(

                // driver.povRight().onTrue(

                // driver.start()

                // codriver.leftBumper()

                // codriver.leftTrigger()

                // codriver.rightBumper()

                // codriver.rightTrigger()

                // codriver.y()

                // codriver.a()
                // codriver.x()
                // codriver.b()

        }

}
