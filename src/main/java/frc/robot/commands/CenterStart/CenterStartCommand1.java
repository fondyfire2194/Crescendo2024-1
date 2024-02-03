// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CenterStart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoFactory;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.RunPPath;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;

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

        public CenterStartCommand1(AutoFactory af, CommandFactory cf, SwerveSubsystem swerve,
                        ElevatorSubsystem elevator,
                        IntakeSubsystem intake, HoldNoteSubsystem holdNote, TopShooterRollerSubsystem topShooter,
                        BottomShooterRollerSubsystem bottomShooter, ShooterAngleSubsystem shooterAngle) {

                addCommands(

                                new SequentialCommandGroup(

                                                Commands.runOnce(() -> swerve.resetPoseEstimator(
                                                                af.activePaths.get(0)
                                                                                .getPreviewStartingHolonomicPose())),

                                                cf.runShooters(60,2500,2500),

                                                 cf.shootNote(),

                                                cf.moveAndPickup(af.activePaths.get(0)),

                                                new RunPPath(swerve, af.activePaths.get(1), false).asProxy(),

                                                cf.shootNote(),

                                                cf.moveAndPickup(af.activePaths.get(4)),

                                                new RunPPath(swerve, af.activePaths.get(5), false).asProxy(),

                                                cf.shootNote(),

                                                cf.moveAndPickup(af.activePaths.get(2)),

                                                new RunPPath(swerve, af.activePaths.get(3), false).asProxy(),

                                               cf.shootNote(),

                                                topShooter.stopShootersCommand(),
                                                bottomShooter.stopShootersCommand()

                                ));

        }

}
