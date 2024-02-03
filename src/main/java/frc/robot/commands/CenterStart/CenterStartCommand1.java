// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CenterStart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoFactory;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.RunPPath;
import frc.robot.subsystems.LeftShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RightShooterRollerSubsystem;

/** Add your docs here. */
public class CenterStartCommand1 extends SequentialCommandGroup {

        Pose2d redStart = new Pose2d(15.69, 5.5, Rotation2d.fromDegrees(180));

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
                        IntakeSubsystem intake, HoldNoteSubsystem holdNote, RightShooterRollerSubsystem rightshooter,
                        LeftShooterRollerSubsystem leftshooter, ShooterAngleSubsystem shooterAngle) {

                addCommands(

                                new SequentialCommandGroup(

                                                cf.setStartPosebyAlliance(af.activePaths.get(0),
                                                                redStart),

                                                cf.runShooters(60, 2500, 2500),

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

                                                rightshooter.stopShooterCommand(),
                                                leftshooter.stopShooterCommand()

                                ));

        }

}
