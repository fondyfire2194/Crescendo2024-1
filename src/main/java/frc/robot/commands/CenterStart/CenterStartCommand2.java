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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoFactory;
import frc.robot.commands.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterStartCommand2 extends SequentialCommandGroup {

 

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

  public CenterStartCommand2(AutoFactory af,SwerveSubsystem swerve) {

  

    addCommands(

        new SequentialCommandGroup(
            // shoot loaded note to speaker

            // pick up note directly behinE
                                               

             new RunPPath(swerve, af.activePaths.get(0), true),

            new WaitCommand(.5),

             new RunPPath(swerve,af. activePaths.get(1), false),

            new WaitCommand(1),

            new RunPPath(swerve, af.activePaths.get(2), false),

            new WaitCommand(.5),

            new RunPPath(swerve, af.activePaths.get(3), false),

            new WaitCommand(1),

            new RunPPath(swerve, af.activePaths.get(4), false),

            new WaitCommand(.5),

            new RunPPath(swerve, af.activePaths.get(5), false),

            new WaitCommand(1)

        ));

  }
}
