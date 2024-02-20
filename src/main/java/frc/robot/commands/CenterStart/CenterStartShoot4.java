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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoFactory;
import frc.robot.PathFactory;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;

import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterStartShoot4 extends SequentialCommandGroup {

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

        public CenterStartShoot4(
                        CommandFactory cf, 
                        AutoFactory af,   
                        PathFactory pf,                   
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ShooterAngleSubsystem shooterAngle ,
                        String llName) {

                addCommands(

                                new SequentialCommandGroup(

                                                cf.setStartPoseWithLimeLight(),

                                                // cf.setStartPosebyAlliance(af.activePaths.get(0)),

                                               // cf.runShooters(2.2, 50),

                                                cf.shootNote(),

                                                cf.moveAndPickup(pf.activePaths.get(0)),

                                                new RunPPath(swerve, pf.activePaths.get(1), false).asProxy(),

                                                cf.shootNote(),

                                                cf.moveAndPickup(pf.activePaths.get(4)),

                                                new RunPPath(swerve, pf.activePaths.get(5), false).asProxy(),

                                                cf.shootNote(),

                                                cf.moveAndPickup(pf.activePaths.get(2)),

                                                new RunPPath(swerve, pf.activePaths.get(3), false).asProxy(),

                                                cf.shootNote(),

                                                shooter.stopShooterCommand()

                                ));

        }

}
