package frc.robot.commands.Autos.AmpStart;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoFactory;
import frc.robot.PathFactory;
import frc.robot.PathFactory.amppaths;
import frc.robot.PathFactory.decisionpoints;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AmpShootInnerOuter extends SequentialCommandGroup {

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

        public AmpShootInnerOuter(
                        CommandFactory cf,
                        AutoFactory af,
                        PathFactory pf,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ShooterAngleSubsystem shooterAngle) {

                addCommands(

                                new SequentialCommandGroup(

                                                cf.setStartPoseWithLimeLight(),

                                                cf.setStartPosebyAlliance(
                                                                pf.pathMaps.get(amppaths.A_S2N1.toString())),

                                                cf.setShooters(2.2),

                                                cf.shootNote(),

                                                cf.moveAndPickup(pf.pathMaps.get(amppaths.A_S2N1.toString())),

                                                cf.setShooters(2.2),

                                                new RunPPath(swerve, pf.pathMaps.get(amppaths.A_N1ToCN1.toString()),
                                                                false),

                                                cf.decideNextPickup(decisionpoints.CN1.ordinal()),

                                                cf.moveAndPickup(pf.pathMaps.get(amppaths.A_CN1ToPU.toString())),

                                                cf.setShooters(2.2),

                                                new RunPPath(swerve, pf.pathMaps.get(amppaths.A_CN1ToShoot.toString()),
                                                                false),
                                                cf.shootNote(),
                                                
                                                cf.moveAndPickup(pf.pathMaps.get(amppaths.A_N1ToCN2.toString())),

                                                cf.decideNextPickup(decisionpoints.CN2.ordinal()),

                                                cf.moveAndPickup(pf.pathMaps.get(amppaths.A_CN2ToPU.toString())),

                                                cf.setShooters(2.2),

                                                new RunPPath(swerve, pf.pathMaps.get(amppaths.A_CN2ToShoot.toString()),
                                                                false),
                                                cf.shootNote())



                );
        }

}
