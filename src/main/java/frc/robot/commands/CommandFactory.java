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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AutoFactory;
import frc.robot.LimelightHelpers;
import frc.robot.PathFactory;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CameraConstants.CameraValues;
import frc.robot.commands.AmpStart.LeaveZone;
import frc.robot.commands.CenterStart.CenterStartCommand1;
import frc.robot.commands.SourceStart.SourceShootThenCenter;
import frc.robot.commands.Drive.AlignToTagSetShootSpeed;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.commands.Vision.LimelightSetStartPose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeedShooterSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.InterpolatingTable;
import frc.robot.utils.ShotParameter;

/** Add your docs here. */
public class CommandFactory {
    private final SwerveSubsystem m_swerve;

    private final ElevatorSubsystem m_elevator;

    private final IntakeSubsystem m_intake;

    private final ShooterSubsystem m_shooter;

    private final FeedShooterSubsystem m_shooterFeed;

    private final ShooterAngleSubsystem m_shooterangle;

    private final HoldNoteSubsystem m_holdnote;

    private final String m_llName;

    private final AutoFactory m_af;

    private final PathFactory m_pf;

    private final LimelightVision m_llv;

    private boolean runAll = false;

    public CommandFactory(PathFactory pf, AutoFactory af, String llName, SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ElevatorSubsystem elevator,
            HoldNoteSubsystem holdNote,
            ShooterAngleSubsystem shooterAngle,
            ShooterSubsystem shooter,
            FeedShooterSubsystem shooterFeed,
            LimelightVision llv) {
        m_swerve = swerve;
        m_pf = pf;
        m_elevator = elevator;
        m_intake = intake;
        m_holdnote = holdNote;
        m_shooter = shooter;
        m_llv = llv;
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

            // amp side starts
            case 1:
                return new LeaveZone(this, m_af, m_swerve);
            case 2:
                return new DoNothing();
            case 3:
                return new DoNothing();

            // center starts
            case 11:
                return new CenterStartCommand1(this, m_pf, m_llName, m_swerve, m_elevator, m_intake, m_holdnote,
                        m_shooter,
                        m_shooterangle).withName("CC1");
            case 12:
                return new DoNothing();

            case 13:
                return new DoNothing();

            // source side starts

            case 21:
                return new LeaveZone(this, m_af, m_swerve);
            case 22:
                return new SourceShootThenCenter(this, m_pf, m_swerve, m_elevator, m_intake, m_holdnote,
                        m_shooter, m_shooterangle).withName("SourceCenter");
            case 23:
                return new SourceShootThenCenter(this, m_pf, m_swerve, m_elevator, m_intake, m_holdnote,
                        m_shooter, m_shooterangle).withName("SourceCenter");
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
        return m_shooter.runBothRollersCommand(shot.leftrpm, shot.rightrpm)
                .alongWith(m_shooterangle.runOnce(
                        () -> m_shooterangle.smartPositionShooterAngle(shot.angle)));

    }

    public double[] getShooterSpeedsFromDistance(double distance) {
        ShotParameter shot = InterpolatingTable.get(distance);
        double[] temp = { 0, 0 };
        temp[0] = shot.leftrpm;
        temp[1] = shot.rightrpm;
        return temp;
    }

    public double getShooterAngleByDistance(double distance) {
        ShotParameter shot = InterpolatingTable.get(distance);
        return shot.angle;
    }

    public Command teleopAlignSpeaker(CameraValues camval, CommandXboxController driver) {
        return new ConditionalCommand(
                new AlignToTagSetShootSpeed(m_swerve, m_llv, this, () -> -driver.getLeftY(), () -> driver.getLeftX(),
                        camval),
                new TeleopSwerve(
                        m_swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRawAxis(4),
                        () -> false,
                        () -> false),
                () -> LimelightHelpers.getTV(CameraConstants.frontLeftCamera.camname));
    }

    public void decideNextPickup() {

        LimelightHelpers.getTV(CameraConstants.rearCamera.camname);

        if (m_swerve.getRearLeftSensorInches() < 20 || m_swerve.getRearRightSensorInches() < 20)
            CommandScheduler.getInstance().cancel(getAutonomusCommand());

    }

    public Command getAutonomusCommand() {
        return finalCommand(m_af.finalChoice);
    }
}