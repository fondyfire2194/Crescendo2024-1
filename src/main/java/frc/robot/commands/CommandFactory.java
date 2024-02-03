// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;

/** Add your docs here. */
public class CommandFactory {
        private final SwerveSubsystem m_swerve;

        private final ElevatorSubsystem m_elevator;

        private final IntakeSubsystem m_intake;

        private final TopShooterRollerSubsystem m_topshooter;

        private final BottomShooterRollerSubsystem m_bottomshooter;

        private final ShooterAngleSubsystem m_shooterangle;

        private final HoldNoteSubsystem m_holdnote;

        public CommandFactory(SwerveSubsystem swerve, IntakeSubsystem intake, ElevatorSubsystem elevator,
                        HoldNoteSubsystem holdNote,
                        ShooterAngleSubsystem shooterAngle, TopShooterRollerSubsystem topShooter,
                        BottomShooterRollerSubsystem bottomShooter) {
                m_swerve = swerve;
                m_elevator = elevator;
                m_intake = intake;
                m_holdnote = holdNote;
                m_topshooter = topShooter;
                m_bottomshooter = bottomShooter;
                m_shooterangle = shooterAngle;
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

        public Command runShooters(double shooterAngle, double topRPM, double bottomRPM) {

                return new ParallelCommandGroup(

                                m_shooterangle.positionCommand(
                                                shooterAngle)
                                                .asProxy(),
                                m_elevator.positionToIntakeCommand().asProxy(),
                                m_topshooter.setShooterSpeed(
                                                topRPM),

                                m_bottomshooter.setShooterSpeed(
                                                bottomRPM),

                                m_bottomshooter.runBottomRollerCommand(),
                                m_topshooter.runTopRollerCommand())

                                .withName("RunShooters");
        }

}
