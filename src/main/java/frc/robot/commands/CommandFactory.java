// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LeftShooterSubsystem;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeedShooterSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.InterpolatingTable;
import frc.robot.utils.ShotParameter;
import frc.robot.subsystems.RightShooterSubsystem;

/** Add your docs here. */
public class CommandFactory {
        private final SwerveSubsystem m_swerve;

        private final ElevatorSubsystem m_elevator;

        private final IntakeSubsystem m_intake;

        private final RightShooterSubsystem m_rightshooter;

        private final LeftShooterSubsystem m_leftshooter;

        private final FeedShooterSubsystem m_shooterFeed;

        private final ShooterAngleSubsystem m_shooterangle;

        private final HoldNoteSubsystem m_holdnote;

        public CommandFactory(SwerveSubsystem swerve, IntakeSubsystem intake, ElevatorSubsystem elevator,
                        HoldNoteSubsystem holdNote,
                        ShooterAngleSubsystem shooterAngle, RightShooterSubsystem rightShooter,
                        LeftShooterSubsystem leftShooter, FeedShooterSubsystem shooterFeed) {
                m_swerve = swerve;
                m_elevator = elevator;
                m_intake = intake;
                m_holdnote = holdNote;
                m_rightshooter = rightShooter;
                m_leftshooter = leftShooter;
                m_shooterangle = shooterAngle;
                m_shooterFeed = shooterFeed;
        }

        public static Rotation2d flipFieldRotation(Rotation2d rotation) {
                return new Rotation2d(Math.PI).minus(rotation);
        }

        public static Pose2d flipPose(Pose2d pose) {
                return GeometryUtil.flipFieldPose(pose);
        }

        public Command setStartPosebyAlliance(PathPlannerPath path) {

                Pose2d temp = path.getPreviewStartingHolonomicPose();

                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)

                {
                        return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
                } else
                        return new DoNothing(); // Commands.runOnce(
                // () -> m_swerve.resetPoseEstimator(temp));
        }

        public Command moveAndPickup(PathPlannerPath path) {

                return new ParallelCommandGroup(

                                new RunPPath(m_swerve, path, true)
                                                .asProxy(),

                                m_holdnote.intakeToNoteSeenCommand(),

                                m_intake.runIntakeCommand().withTimeout(.5)
                                                .andThen(m_intake.stopIntakeCommand()));
        }

        public Command shootNote() {
                return new SequentialCommandGroup(
                                m_holdnote.feedShooterCommand().withTimeout(.1)
                                                .andThen(m_holdnote.stopHoldNoteCommand()),
                                new WaitCommand(1));
        }

        public Command runShooters(double distance, double feedRPM) {

                return new SequentialCommandGroup(

                                distanceShot(distance),

                                new ParallelCommandGroup(

                                                m_elevator.positionToIntakeCommand().asProxy(),

                                                m_shooterFeed.runFeedBeltsCommand(feedRPM)))

                                .withName("RunShooters");
        }

        private Command distanceShot(double distance) {
                ShotParameter shot = InterpolatingTable.get(distance);
                return m_leftshooter.runLeftRollerCommand(shot.rpm)
                                .alongWith(m_rightshooter.runRightRollerCommand(shot.rpm),
                                                m_shooterangle.runOnce(
                                                                () -> m_shooterangle.positionShooterAngle(shot.angle)));

        }
}