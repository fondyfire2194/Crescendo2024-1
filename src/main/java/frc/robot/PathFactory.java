// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import java.util.HashMap;
import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PathFactory {

    private final SwerveSubsystem m_swerve;

    int ampChoice;
    int ampChoiceLast;

    int centerChoice;
    int centerChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public boolean ampFilesOK;
    public boolean centerFilesOK;
    public boolean sourceFilesOK;

    public HashMap<String, PathPlannerPath> pathMaps = new HashMap<String, PathPlannerPath>();

    public PathFactory(SwerveSubsystem swerve) {
        m_swerve = swerve;
    }

    public enum amppaths {
        A_CN1ToCN2,
        A_CN1ToPU,
        A_CN1ToShoot,
        A_CN2ToCN1,
        A_CN2ToPU,
        A_CN2ToShoot,
        A_CN3ToPU,
        A_N1ToCN1,
        A_N1ToCN2,
        A_S2N1,
        A_SToCN3;
    }

    public boolean checkAmpFilesExist() {
        int valid = 0;
        for (amppaths a : amppaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.name() + ".path").isFile())
                valid++;
            SmartDashboard.putNumber("Valid", valid);
        }
        return valid == amppaths.values().length;
    }

    public void linkAmpPaths() {
        pathMaps.clear();
        for (amppaths a : amppaths.values()) {
            pathMaps.put(a.name(), getPath(a.name()));
        }
    }

    public enum centerpaths {
        C_N1ToS,
        C_N2ToN1,
        C_N2ToS,
        C_N3ToN2,
        C_N3ToS,
        C_SToN1,
        C_SToN2,
        C_SToN3;
    }

    public boolean checkCenterFilesExist() {
        int valid = 0;
        for (centerpaths a : centerpaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.name() + ".path").isFile())
                valid++;
        }
        return valid == centerpaths.values().length;
    }

    public void linkCenterPaths() {
        pathMaps.clear();
        for (centerpaths c : centerpaths.values()) {
            // if (DriverStation.getAlliance().isPresent()
            // && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            // pathMaps.put(c.name(), getPath(c.name()).flipPath());
            // else
            pathMaps.put(c.name(), getPath(c.name()));
        }
    }

    public enum sourcepaths {
        SourceInner1Pickup,
        SourceInner1ToShoot,
        SourceOuterPickup,
        SourceOuterPickupToShoot,
        SourceOuterToInnerDecision,
        SourceToOuterDecision;
    }

    public boolean checkSourceFilesExist() {
        int valid = 0;
        for (sourcepaths a : sourcepaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.toString() + ".path").isFile())
                valid++;
        }
        return valid == sourcepaths.values().length;
    }

    public void linkSourcePaths() {
        pathMaps.clear();
        for (sourcepaths s : sourcepaths.values()) {
            pathMaps.put(s.toString(), getPath(s.toString()));
        }
    }

    public enum decisionpoints {
        CN1,
        CN2, CN3,
        CN4,
        CN5;
    }

    public PathPlannerPath getPath(String pathname) {
        return PathPlannerPath.fromPathFile(pathname);
    }

    public Command setStartPosebyAlliance(PathPlannerPath path) {
        Pose2d temp = path.getPreviewStartingHolonomicPose();
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
        } else
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(temp));
    }

    public static Pose2d flipPose(Pose2d pose) {
        return GeometryUtil.flipFieldPose(pose);
    }

}