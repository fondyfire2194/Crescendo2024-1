// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.CenterStart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoFactory;
import frc.robot.PathFactory;
import frc.robot.PathFactory.centerpaths;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterStartShoot4FromSubwoofer extends SequentialCommandGroup {

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

        public CenterStartShoot4FromSubwoofer(
                        CommandFactory cf,
                        AutoFactory af,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ShooterAngleSubsystem shooterAngle) {

                addCommands(

                                new SequentialCommandGroup(

                                                // cf.setStartPoseWithLimeLight(),

                                                cf.setStartPosebyAlliance(
                                                                pf.pathMaps.get(centerpaths.C_SToN2.toString())),

                                                 cf.setShooters(2.2),

                                                new WaitCommand(2),

                                                 cf.moveAndPickup(pf.pathMaps.get(centerpaths.C_SToN2.toString())),

                                               
                                                new RunPPath(swerve, pf.pathMaps.get(centerpaths.C_N2ToS.toString()),
                                                                false),

                                              //  cf.shootNote(),

                                                cf.moveAndPickup(pf.pathMaps.get(centerpaths.C_SToN3.toString())),

                                                new RunPPath(swerve, pf.pathMaps.get(centerpaths.C_N3ToS.toString()),
                                                                false),

                                               // cf.shootNote(),

                                                cf.moveAndPickup(pf.pathMaps.get(centerpaths.C_SToN1.toString())),

                                                new RunPPath(swerve, pf.pathMaps.get(centerpaths.C_N1ToS.toString()),
                                                                false),

                                                cf.shootNote(),

                                                shooter.stopShooterCommand()

                                ));

        }

}
