// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.block.MatrixMult_MT_DDRB;
import org.ejml.dense.block.MatrixMult_MT_FDRB;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CommandFactory {

        public SequentialCommandGroup centerChoice2;

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

        public CommandFactory(SwerveSubsystem drive, ArmSubsystem arm,
                        IntakeSubsystem intake, ShooterSubsystem shooter) {

                centerChoice2 = new SequentialCommandGroup(
                                // shoot loaded note to speaker
                                new ParallelCommandGroup(
                                                arm.positionArmCommand(
                                                                Constants.ArmConstants.armPositionToShootClose),
                                                shooter.setShooterSpeed(ShooterConstants.closeShootSpeed)),

                                intake.feedShooterCommand(),
                                new WaitCommand(1),

                                // pick up note directly behind

                                new ParallelCommandGroup(

                                                arm.positionArmCommand(
                                                                Constants.ArmConstants.armPositionToIntakeDegrees)
                                                                .asProxy(),
                                                shooter.setShooterSpeed(
                                                                ShooterConstants.dist1ShootSpeed)
                                                                .asProxy(),

                                                new ParallelRaceGroup(

                                                                getPathToPose(FieldConstants.blueNote1,
                                                                                pathConstraints),
                                                                intake.runIntakeCommand().asProxy())),

                                intake.feedShooterCommand(),
                                new WaitCommand(1),

                                new ParallelCommandGroup(

                                                arm.positionArmCommand(
                                                                Constants.ArmConstants.armPositionToIntakeDegrees)
                                                                .asProxy(),
                                                shooter.setShooterSpeed(
                                                                ShooterConstants.dist3ShootSpeed)
                                                                .asProxy(),

                                                new ParallelRaceGroup(
                                                                getPathToPose(FieldConstants.blueNote3,
                                                                                pathConstraints),
                                                                new DoNothing(),
                                                                // getSinglePathCommand("AutoOneP2"),
                                                                intake.runIntakeCommand())),

                                intake.feedShooterCommand(),
                                new WaitCommand(1),

                                new ParallelCommandGroup(

                                                arm.positionArmCommand(
                                                                Constants.ArmConstants.armPositionToIntakeDegrees),
                                                shooter.setShooterSpeed(
                                                                ShooterConstants.dist1ShootSpeed),

                                                new ParallelRaceGroup(
                                                                new DoNothing(),
                                                                getPathToPose(FieldConstants.blueNote3,
                                                                                pathConstraints),
                                                                // getSinglePathCommand("AutoOneP3"),
                                                                intake.runIntakeCommand())),

                                intake.feedShooterCommand(),
                                new WaitCommand(1)

                );

        }

}
