// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterStartCommandFactory {

        PathPlannerPath tempPath = this.getPath("AutoOneP1");
        ArrayList<PathPlannerPath> activePaths = new ArrayList<PathPlannerPath>(5);

        public void loadPathFiles(List<String> fileNames) {
                activePaths.clear();
                for (String i : fileNames) {
                        activePaths.add(getPath(i));
                }
        }

        public SequentialCommandGroup centerChoice1;

        public PathPlannerPath getPath(String pathname) {
                return PathPlannerPath.fromPathFile(pathname);
        }

        PathConstraints pathConstraints = new PathConstraints(
                        3.0, 4.0,
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(540));

        public Command getSinglePathCommand(String pathname) {

                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);
                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.
                return AutoBuilder.followPath(path);
        }

        public Command getPathToPose(Pose2d pose, PathConstraints constraints) {
                return AutoBuilder.pathfindToPose(pose, constraints, 0, 2);
        }

        public CenterStartCommandFactory(SwerveSubsystem swerve, ElevatorSubsystem elevator,
                        IntakeSubsystem intake, HoldNoteSubsystem holdNote, TopShooterRollerSubsystem topShooter,
                        BottomShooterRollerSubsystem bottomShooter, ShooterAngleSubsystem shooterAngle) {

                activePaths.add(tempPath);
                activePaths.add(tempPath);
                activePaths.add(tempPath);
                activePaths.add(tempPath);
                activePaths.add(tempPath);
                activePaths.add(tempPath);

                centerChoice1 = new SequentialCommandGroup(
                                // shoot loaded note to speaker
                                new ParallelCommandGroup(
                                                shooterAngle.positionShooterAngleCommand(0),
                                                elevator.positionToIntakeCommand(),                                                                
                                                topShooter.setShooterSpeed(ShooterConstants.closeShootSpeed),
                                                bottomShooter.setShooterSpeed(ShooterConstants.closeShootSpeed)),

                                holdNote.feedShooterCommand(),
                                new WaitCommand(1),

                                // pick up note directly behinE

                                new ParallelCommandGroup(

                                                new RunPPath(swerve, activePaths.get(0), false),
                                                holdNote.runHoldNoteCommand().asProxy(),
                                                intake.runIntakeCommand().asProxy()),

                                new RunPPath(swerve, activePaths.get(1), false),

                                holdNote.feedShooterCommand(),
                                new WaitCommand(1),

                                new ParallelCommandGroup(
                                                new RunPPath(swerve, activePaths.get(2), false),
                                                holdNote.runHoldNoteCommand(),
                                                intake.runIntakeCommand()),

                                new RunPPath(swerve, activePaths.get(3), false),

                                holdNote.feedShooterCommand(),
                                new WaitCommand(1),

                                new ParallelCommandGroup(

                                                new RunPPath(swerve, activePaths.get(4), false),
                                                intake.runIntakeCommand(),
                                                holdNote.runHoldNoteCommand()),

                                new RunPPath(swerve, activePaths.get(5), false),

                                intake.feedShooterCommand(),
                                new WaitCommand(1)

                );

        }

}
