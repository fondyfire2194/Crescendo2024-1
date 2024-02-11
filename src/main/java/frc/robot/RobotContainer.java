// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Pathplanner.SetStartByAlliance;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem();

        // final IntakeSubsystem m_intake = new IntakeSubsystem();

        // final RightShooterSubsystem m_rightShooter = new RightShooterSubsystem();

        // final LeftRollerSubsystem m_leftShooter = new LeftShooterSubsystem();

        // final FeedShooterSubsystem m_shooterFeeder = new FeedShooterSubsystem();

        // final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

        // final HoldNoteSubsystem m_holdNote = new HoldNoteSubsystem();

        // final ShooterAngleSubsystem m_shooterAngle = new ShooterAngleSubsystem();

        // final CommandFactory m_cf = new CommandFactory(m_swerve, m_intake,
        // m_elevator,
        // m_holdNote, m_shooterAngle, m_rightShooter, m_leftShooter,m_shooterFeeder);

         final LimelightSubsystem m_llv1 = new LimelightSubsystem("limelight");

        // final LimelightSubsystem m_llv2 = new LimelightSubsystem("limelight_1");

        // final LimelightSubsystem m_llv3 = new LimelightSubsystem("limelight_2");
        public final PathFactory m_pf = new PathFactory(m_swerve);

        public final AutoFactory m_af = new AutoFactory(m_pf);

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        private final CommandXboxController setupCommandXboxController = new CommandXboxController(2);

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                Pref.deleteUnused();

                Pref.addMissing();

                setDefaultCommands();

                registerNamedCommands();

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

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
                                                () -> -driver.getRawAxis(2),
                                                fieldCentric));

                // m_elevator.setDefaultCommand(m_elevator.positionHold());

                // m_shooterAngle.setDefaultCommand(m_shooterAngle.positionHold());

                // m_intake.setDefaultCommand(Commands.runOnce(() -> m_intake.stopMotor(),
                // m_intake));
                // m_shooter.setDefaultCommand(Commands.runOnce(() -> m_shooter.stopMotors(),
                // m_shooter));
                // m_llv1.setDefaultCommand(new TrackAprilTags3D(m_llv1, m_swerve));
                // m_llv2.setDefaultCommand(new TrackAprilTags3D(m_llv2, m_swerve));
        }

        private void registerNamedCommands() {
                // Register Named Commands
                NamedCommands.registerCommand("LimelightSetStartPose1",
                                new SetStartByAlliance(m_swerve, "CentOneP1"));
                NamedCommands.registerCommand("SetAngleSpeed1",
                                Commands.runOnce(() -> SmartDashboard.putString("AngleSpeed1", "")));
                NamedCommands.registerCommand("SetAngleSpeed2",
                                Commands.runOnce(() -> SmartDashboard.putString("AngleSpeed2", "")));
                NamedCommands.registerCommand("SetAngleSpeed3",
                                Commands.runOnce(() -> SmartDashboard.putString("AngleSpeed3", "")));
                NamedCommands.registerCommand("SetAngleSpeed4",
                                Commands.runOnce(() -> SmartDashboard.putString("AngleSpeed4", "")));
                NamedCommands.registerCommand("ShootNote",
                                Commands.runOnce(() -> SmartDashboard.putString("ShootNote", "")));
                NamedCommands.registerCommand("RunIntake",
                                Commands.runOnce(() -> SmartDashboard.putString("RunIntake", "")));
                NamedCommands.registerCommand("StopShooter",
                                Commands.runOnce(() -> SmartDashboard.putString("StopShooter", "")));

        }

        private void configureBindings() {

                // SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());

                driver.y().onTrue(m_swerve.setPoseToX0Y0());

                // driver.a()

                // driver.x().

                // driver.b()

                // driver.back()

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

        public Command getTestPathCommand() {
                return autoChooser.getSelected();
        }

        
}
