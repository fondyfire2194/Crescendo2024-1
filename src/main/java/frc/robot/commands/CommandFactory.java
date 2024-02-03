// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LeftShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RightShooterRollerSubsystem;

/** Add your docs here. */
public class CommandFactory {
        private final SwerveSubsystem m_swerve;

        private final ElevatorSubsystem m_elevator;

        private final IntakeSubsystem m_intake;

        private final RightShooterRollerSubsystem m_rightshooter;

        private final LeftShooterRollerSubsystem m_leftshooter;

        private final ShooterAngleSubsystem m_shooterangle;

        private final HoldNoteSubsystem m_holdnote;

        public CommandFactory(SwerveSubsystem swerve, IntakeSubsystem intake, ElevatorSubsystem elevator,
                        HoldNoteSubsystem holdNote,
                        ShooterAngleSubsystem shooterAngle, RightShooterRollerSubsystem rightShooter,
                        LeftShooterRollerSubsystem leftShooter) {
                m_swerve = swerve;
                m_elevator = elevator;
                m_intake = intake;
                m_holdnote = holdNote;
                m_rightshooter = rightShooter;
                m_leftshooter = leftShooter;
                m_shooterangle = shooterAngle;
        }

        public Command setStartPosebyAlliance(PathPlannerPath path, Pose2d redstart) {

                Pose2d temp = new Pose2d();

                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)

                return Commands.runOnce(() -> m_swerve.resetPoseEstimator(redstart));     

                else
                         
                return Commands.runOnce(() -> m_swerve.resetPoseEstimator(path.getPreviewStartingHolonomicPose()));

        }

        public Command moveAndPickup(PathPlannerPath path) {

                return new ParallelCommandGroup(

                                new RunPPath(m_swerve, path, true)
                                                .asProxy(),

                                m_holdnote.feedShooterCommand()
                                                .andThen(m_holdnote
                                                                .stopHoldNoteCommand()),

                                m_intake.runIntakeCommand().withTimeout(.5)
                                                .andThen(m_intake.stopIntakeCommand()));
        }

        public Command shootNote() {
                return new SequentialCommandGroup(
                                m_holdnote.feedShooterCommand().withTimeout(.25)
                                                .andThen(m_holdnote.stopHoldNoteCommand()),
                                new WaitCommand(1));
        }

        public Command runShooters(double shooterAngle, double rightRPM, double leftRPM) {

                return new ParallelCommandGroup(

                                m_shooterangle.positionCommand(
                                                shooterAngle)
                                                .asProxy(),
                                m_elevator.positionToIntakeCommand().asProxy(),
                                m_rightshooter.setShooterSpeed(
                                                rightRPM),

                                m_leftshooter.setShooterSpeed(
                                                leftRPM),

                                m_leftshooter.runLeftRollerCommand(),
                                m_rightshooter.runRightRollerCommand())

                                .withName("RunShooters");
        }

}
