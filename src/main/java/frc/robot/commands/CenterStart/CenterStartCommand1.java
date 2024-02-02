// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CenterStart;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoFactory;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RunPPath;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterStartCommand1 extends SequentialCommandGroup {

        public PathPlannerPath getPath(String pathname) {
                return PathPlannerPath.fromPathFile(pathname);
        }

        PathConstraints pathConstraints = new PathConstraints(
                        3.0, 4.0,
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(540));

        public Command getPathToPose(Pose2d pose, PathConstraints constraints) {
                return AutoBuilder.pathfindToPose(pose, constraints, 0, 2);
        }

        public CenterStartCommand1(AutoFactory af, SwerveSubsystem swerve, ElevatorSubsystem elevator,
                        IntakeSubsystem intake, HoldNoteSubsystem holdNote, TopShooterRollerSubsystem topShooter,
                        BottomShooterRollerSubsystem bottomShooter, ShooterAngleSubsystem shooterAngle) {

                addCommands(

                                new SequentialCommandGroup(

                                                Commands.runOnce(() -> swerve.resetPoseEstimator(
                                                                af.activePaths.get(0)
                                                                                .getPreviewStartingHolonomicPose())),
                                                // shoot loaded note to speaker
                                                new ParallelCommandGroup(
                                                                shooterAngle.positionCommand(
                                                                                ShooterAngleConstants.shooterangleMinDegrees)
                                                                                .asProxy(),
                                                                elevator.positionToIntakeCommand().asProxy(),
                                                                topShooter.setShooterSpeed(
                                                                                ShooterConstants.closeShootSpeed)
                                                                                .asProxy(),
                                                                bottomShooter.setShooterSpeed(
                                                                                ShooterConstants.closeShootSpeed)
                                                                                .asProxy(),
                                                                bottomShooter.runBottomRollerCommand().asProxy(),
                                                                topShooter.runTopRollerCommand()).asProxy(),

                                                holdNote.feedShooterCommand().withTimeout(.25)
                                                                .andThen(holdNote.stopHoldNoteCommand()),
                                                new WaitCommand(1),

                                                // pick up note directly behinE

                                                new ParallelCommandGroup(

                                                                new RunPPath(swerve, af.activePaths.get(0), true)
                                                                                .asProxy(),
                                                                holdNote.runHoldNoteCommand().asProxy().withTimeout(.5)
                                                                                .andThen(holdNote
                                                                                                .stopHoldNoteCommand()),

                                                                intake.runIntakeCommand().asProxy().withTimeout(.5)
                                                                                .andThen(intake.stopIntakeCommand())),

                                                new RunPPath(swerve, af.activePaths.get(1), false).asProxy(),

                                                holdNote.feedShooterCommand().withTimeout(.25),
                                                new WaitCommand(1),

                                                new ParallelCommandGroup(
                                                                new RunPPath(swerve, af.activePaths.get(2), false)
                                                                                .asProxy(),
                                                                holdNote.runHoldNoteCommand().asProxy().withTimeout(.5)
                                                                                .andThen(holdNote
                                                                                                .stopHoldNoteCommand()),

                                                                intake.runIntakeCommand().asProxy().withTimeout(.5)
                                                                                .andThen(intake.stopIntakeCommand())),

                                                new RunPPath(swerve, af.activePaths.get(3), false).asProxy(),

                                                holdNote.feedShooterCommand().withTimeout(.25)
                                                                .andThen(intake.stopIntakeCommand()),
                                                new WaitCommand(1),

                                                new ParallelCommandGroup(

                                                                new RunPPath(swerve, af.activePaths.get(4), false)
                                                                                .asProxy(),
                                                                holdNote.runHoldNoteCommand().asProxy().withTimeout(.5)
                                                                                .andThen(holdNote
                                                                                                .stopHoldNoteCommand()),

                                                                intake.runIntakeCommand().asProxy().withTimeout(.5)
                                                                                .andThen(intake.stopIntakeCommand())),

                                                new RunPPath(swerve, af.activePaths.get(5), false).asProxy(),

                                                intake.feedShooterCommand().withTimeout(.25)
                                                                .andThen(intake.stopIntakeCommand()),
                                                new WaitCommand(1),
                                                topShooter.stopShootersCommand(),
                                                bottomShooter.stopShootersCommand()

                                ));

        }

}
