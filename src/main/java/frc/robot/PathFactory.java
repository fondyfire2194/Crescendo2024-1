// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PathFactory {

    private final SwerveSubsystem m_swerve;
    int finalChoice = 0;

    int ampChoice;
    int ampChoiceLast;

    int centerChoice;
    int centerChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public final SendableChooser<Integer> m_pathOnlyChooser = new SendableChooser<Integer>();

    List<String> usedPathFiles = new ArrayList<>();

    public ArrayList<PathPlannerPath> activePaths = new ArrayList<PathPlannerPath>(5);

    public PathFactory(SwerveSubsystem swerve) {

        PathPlannerPath tempPath = getPath("BasicLeave");
        m_swerve = swerve;
        for (int i = 0; i < 9; i++)
            activePaths.add(tempPath);

        m_pathOnlyChooser.setDefaultOption("Not Used", 10);
        m_pathOnlyChooser.addOption("Score 4", 11);
        m_pathOnlyChooser.addOption("Not Used", 12);

    }

    public int selectAndLoadPathFiles(int choice) {

        usedPathFiles.clear();

        setFilenames(choice);

        SmartDashboard.putNumber("AUPL", usedPathFiles.size());

        loadPathFiles(usedPathFiles);

        SmartDashboard.putNumber("APAths", activePaths.size());

        return choice;

    }

    public List<String> setFilenames(int index) {

        usedPathFiles.clear();

        switch (index) {

            case 11:

                usedPathFiles.add("CentOneP1");
                usedPathFiles.add("CentOneP1R");
                usedPathFiles.add("CentOneP2");
                usedPathFiles.add("CentOneP2R");
                usedPathFiles.add("CentOneP3");
                usedPathFiles.add("CentOneP3R");

                return usedPathFiles;

            case 12:

            default:

                return usedPathFiles;

        }

    }

    public PathPlannerPath getPath(String pathname) {
        return PathPlannerPath.fromPathFile(pathname);
    }

    public void loadPathFiles(List<String> fileNames) {
        activePaths.clear();
        for (String i : fileNames) {
            activePaths.add(getPath(i));

        }

    }

    public Command setStartPosebyAlliance(PathPlannerPath path) {

        Pose2d temp = path.getPreviewStartingHolonomicPose();

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)

        {
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
        } else
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(temp));

    }

    public static Pose2d flipPose(Pose2d pose) {
        return GeometryUtil.flipFieldPose(pose);
    }

}