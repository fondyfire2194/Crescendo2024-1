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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LeftShooterSubsystem;
import frc.robot.AutoFactory;
import frc.robot.PathFactory;
import frc.robot.commands.CenterStart.CenterStartCommand1;
import frc.robot.commands.CenterStart.CenterStartCommand1Paths;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.commands.Vision.LimelightSetStartPose;
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

    private final String m_llName;

    private final AutoFactory m_af;

    private final PathFactory m_pf;

    private boolean runAll = false;

    public CommandFactory(PathFactory pf, AutoFactory af, String llName, SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ElevatorSubsystem elevator,
            HoldNoteSubsystem holdNote,
            ShooterAngleSubsystem shooterAngle, RightShooterSubsystem rightShooter,
            LeftShooterSubsystem leftShooter, FeedShooterSubsystem shooterFeed) {
        m_swerve = swerve;
        m_pf = pf;
        m_elevator = elevator;
        m_intake = intake;
        m_holdnote = holdNote;
        m_rightshooter = rightShooter;
        m_leftshooter = leftShooter;
        m_shooterangle = shooterAngle;
        m_shooterFeed = shooterFeed;
        m_llName = llName;
        m_af = af;
    }

    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }

    public static Pose2d flipPose(Pose2d pose) {
        return GeometryUtil.flipFieldPose(pose);
    }

    public Command finalCommand(int choice) {

        switch (choice) {
            case 1:
                return new DoNothing();
            case 2:
                return new DoNothing();
            case 3:
                return new DoNothing();
            case 11:
                return new CenterStartCommand1(this, m_pf, m_llName, m_swerve, m_elevator, m_intake, m_holdnote,
                        m_rightshooter,
                        m_leftshooter, m_shooterangle).withName("CC1");
            case 12:
                return new CenterStartCommand1Paths(m_pf, m_swerve).withName("CC2");
            case 13:
                return new DoNothing();
            case 21:
                return new DoNothing();
            case 22:
                return new DoNothing();
            case 23:
                return new DoNothing();
            default:
                return new DoNothing();

        }
    }

    public Command setStartPoseWithLimeLight() {

        return new LimelightSetStartPose(
            m_llName, m_swerve, m_af.activePaths.get(0).getPreviewStartingHolonomicPose());

    }

    public Command setStartPosebyAlliance(PathPlannerPath path) {

        Pose2d temp = path.getPreviewStartingHolonomicPose();

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)

        {
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
        } else
            return new DoNothing();

    }

    public Command moveAndPickup(PathPlannerPath path) {

        return new ConditionalCommand(

                new ParallelCommandGroup(
                        new RunPPath(m_swerve, path, true)
                                .asProxy(),
                        m_holdnote.intakeToNoteSeenCommand(),
                        m_intake.runIntakeCommand().withTimeout(.5)
                                .andThen(m_intake.stopIntakeCommand())),
                new DoNothing(),
                () -> runAll);
    }

    public Command shootNote() {

        return new ConditionalCommand(

                new SequentialCommandGroup(
                        m_holdnote.feedShooterCommand().withTimeout(.1)
                                .andThen(m_holdnote.stopHoldNoteCommand()),
                        new WaitCommand(1)),
                new DoNothing(), () -> runAll);
    }

    public Command runShooters(double distance, double feedRPM) {

        return new ConditionalCommand(

                new SequentialCommandGroup(

                        distanceShot(distance),

                        new ParallelCommandGroup(

                                m_elevator.positionToIntakeCommand().asProxy(),

                                m_shooterFeed.runFeedBeltsCommand(feedRPM)))

                        .withName("RunShooters"),

                new DoNothing(), () -> runAll);
    }

    private Command distanceShot(double distance) {
        ShotParameter shot = InterpolatingTable.get(distance);
        return m_leftshooter.runLeftRollerCommand(shot.leftrpm)
                .alongWith(m_rightshooter.runRightRollerCommand(shot.rightrpm),
                        m_shooterangle.runOnce(
                                () -> m_shooterangle.positionShooterAngle(shot.angle)));

    }

    public Command getAutonomusCommand() {
        return finalCommand(m_af.finalChoice);
    }
}