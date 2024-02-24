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
import frc.robot.AutoFactory;
import frc.robot.Constants.CameraConstants;
import frc.robot.PathFactory.amppaths;
import frc.robot.LimelightHelpers;
import frc.robot.PathFactory;
import frc.robot.commands.Autos.CenterStart.CenterStartShoot4FromSubwoofer;
import frc.robot.commands.Autos.SourceStart.SourceShootThenCenter;
import frc.robot.commands.Autos.AmpStart.AmpShootOuterInner;
import frc.robot.commands.Drive.DriveToPosition;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.commands.Vision.LimelightSetStartPose;
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

    private final IntakeSubsystem m_intake;

    private final ShooterSubsystem m_shooter;

    private final ShooterAngleSubsystem m_shooterAngle;

    private final String m_llName;

    private final AutoFactory m_af;

    private final PathFactory m_pf;

    private final LimelightVision m_llv;

    private boolean runAll = false;

    public CommandFactory(PathFactory pf,
            AutoFactory af,
            String llName,
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            ShooterAngleSubsystem shooterAngle,
            IntakeSubsystem intake,
            LimelightVision llv) {
        m_swerve = swerve;
        m_pf = pf;

        m_intake = intake;

        m_shooter = shooter;
        m_shooterAngle = shooterAngle;
        m_llv = llv;
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
                return new DriveToPosition(m_swerve, 1);
            case 2:
            case 3:
                return new AmpShootOuterInner(
                        this,
                        m_af,
                        m_pf,
                        m_swerve,
                        m_intake,
                        m_shooter,
                        m_shooterAngle);

            // center starts
            case 11:
                return new CenterStartShoot4FromSubwoofer(
                        this,
                        m_af,
                        m_pf,
                        m_swerve,
                        m_intake,
                        m_shooter,
                        m_shooterAngle).withName("CC1");

            // source side starts

            case 21:
                return new DriveToPosition(m_swerve, 1);
            case 22:
            case 23:
                return new SourceShootThenCenter(
                        this,
                        m_af,
                        m_pf,
                        m_swerve,
                        m_intake,
                        m_shooter,
                        m_shooterAngle);

            default:
                return Commands.none();

        }
    }

    public Command setStartPoseWithLimeLight() {
        return new LimelightSetStartPose(
                m_llName, m_swerve, m_pf.pathMaps.get(amppaths.A_S2N1.name()).getPreviewStartingHolonomicPose());
    }

    public Command setStartPosebyAlliance(PathPlannerPath path) {

        Pose2d temp = path.getPreviewStartingHolonomicPose();

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)

        {
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
        } else
            return Commands.none();

    }

    public Command moveAndPickup(PathPlannerPath path) {

        return new ConditionalCommand(

                new ParallelCommandGroup(
                        new RunPPath(m_swerve, path, true)
                                .asProxy(),

                        m_intake.runIntakeCommand().withTimeout(.5)
                                .andThen(m_intake.stopIntakeCommand())),
                Commands.none(),
                () -> runAll);
    }

    public Command moveAndPickup2() {

        return Commands.none();

    }

    public Command moveAndPickupSetShooter(PathPlannerPath path, double distance) {

        return new ConditionalCommand(

                new ParallelCommandGroup(
                        new RunPPath(m_swerve, path, true)
                                .asProxy(),

                        m_intake.runIntakeCommand().withTimeout(.5)
                                .andThen(m_intake.stopIntakeCommand())),
                Commands.none(),
                () -> runAll);
    }

    public Command shootNote() {

        return new ConditionalCommand(

                new SequentialCommandGroup(
                        Commands.none(),
                        new WaitCommand(1)),
                Commands.none(), () -> runAll);
    }

    public Command setShooters(double distance) {

        return Commands.none();
    }

    private Command distanceShot(double distance) {
        ShotParameter shot = InterpolatingTable.get(distance);
        return m_shooter.runBothRollersCommand(shot.leftrpm, shot.rightrpm)
                .alongWith(m_shooterAngle.runOnce(
                        () -> m_shooterAngle.smartPositionShooterAngle(shot.angle)));

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

    public Command decideNextPickup(int location) {

        boolean notePresent = LimelightHelpers.getTV(CameraConstants.rearCamera.camname);

        double noteSeenAngle = 0;

        if (notePresent)
            LimelightHelpers.getTX(CameraConstants.rearCamera.camname);

        boolean targetNoteSeen = Math.abs(noteSeenAngle) < 1;

        if (m_swerve.getRearLeftSensorInches() < 20 || m_swerve.getRearRightSensorInches() < 20 || !targetNoteSeen) {

            switch (location) {

                // location 0 is the amp outer so next command moves to inner and tries pickup
                // there
                case 0:
                    return Commands.none();

                // location 1 is the amp inner so next command moves to outer and tries pickup
                // there
                case 1:
                    return Commands.none(); // location 2 is the source outer so next command moves to inner and tries
                                            // pickup there
                case 2:
                    return Commands.none();

                // location 3 is the source inner so next command moves to outer and tries
                // pickup there
                case 3:
                    return Commands.none();
                // location 4 is the center so next command moves to amp inner and tries
                // pickup there
                case 4:
                    return Commands.none();

                default:
                    return Commands.none();

            }
        } else
            return Commands.none();
    }

    public Command getAutonomusCommand() {
        return finalCommand(m_af.finalChoice);
    }

}