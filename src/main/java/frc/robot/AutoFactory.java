// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoFactory {

        private final PathFactory m_pf;

        private final SwerveSubsystem m_swerve;

        public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_centerStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

        public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

        public int finalChoice = 0;

        int ampChoice;
        int ampChoiceLast;

        int centerChoice;
        int centerChoiceLast;

        int sourceChoice;
        int sourceChoiceLast;

        public int validStartChoice = 0;

        public AutoFactory(PathFactory pf, SwerveSubsystem swerve) {

                m_pf = pf;
                m_swerve = swerve;

                m_startDelayChooser.setDefaultOption("0 sec", 0.);
                m_startDelayChooser.addOption("1 sec", 1.);
                m_startDelayChooser.addOption("2 sec", 2.);
                m_startDelayChooser.addOption("3 sec", 3.);
                m_startDelayChooser.addOption("4 sec", 4.);
                m_startDelayChooser.addOption("5 sec", 5.);

                m_ampStartChooser.setDefaultOption("Not Used", 0);
                m_ampStartChooser.addOption("Leave Zone", 1);
                m_ampStartChooser.addOption("ShootOuterInner", 2);
                m_ampStartChooser.addOption("ShootInnerOuter", 3);
                

                m_centerStartChooser.setDefaultOption("Not Used", 10);
                m_centerStartChooser.addOption("Score 4", 11);

                m_sourceStartChooser.setDefaultOption("Not Used", 20);
                m_sourceStartChooser.addOption("LeaveZone", 21);
                m_sourceStartChooser.addOption("ShootThenCenter", 22);
                m_sourceStartChooser.addOption("ShootThenInnerOne", 23);

                Shuffleboard.getTab("Autonomous").add("DelayChooser", m_startDelayChooser)
                                .withSize(1, 1).withPosition(9, 0);
                Shuffleboard.getTab("Autonomous").add("AmpStart", m_ampStartChooser)
                                .withSize(2, 1).withPosition(7, 0);
                Shuffleboard.getTab("Autonomous").add("CenterStart", m_centerStartChooser)
                                .withSize(2, 1).withPosition(7, 1);
                Shuffleboard.getTab("Autonomous").add("SourceStart", m_sourceStartChooser)
                                .withSize(2, 1).withPosition(7, 2);

                Shuffleboard.getTab("Autonomous").addBoolean("Valid Choice", () -> finalChoice != 0)
                                .withSize(1, 1).withPosition(9, 1)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("Cameras", BuiltInLayouts.kList).withPosition(9, 2)
                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));

                camLayout.addBoolean("FrontLeftCamera", () -> CameraConstants.frontLeftCamera.isActive)
                                // .withSize(1, 1).withPosition(8, 0)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("FrontRightCamera", () -> CameraConstants.frontRightCamera.isActive)
                                // .withSize(1, 1).withPosition(8, 1)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("RearCamera", () -> CameraConstants.rearCamera.isActive)
                                // .withSize(1, 1).withPosition(8, 2)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout rearLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("RearValues", BuiltInLayouts.kList).withPosition(0, 0)
                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));

                rearLayout.addBoolean("NoteSeen", () -> LimelightHelpers.getTV(CameraConstants.rearCamera.camname));
                rearLayout.addNumber("LeftSensor", () -> m_swerve.getRearLeftSensorInches());
                rearLayout.addNumber("RightSensor", () -> m_swerve.getRearRightSensorInches());

            //    rearLayout.addNumber("BatteryVolts", () -> RobotController.getBatteryVoltage());

        }

        public boolean checkChoiceChange() {

                ampChoice = m_ampStartChooser.getSelected();// 0 start
                centerChoice = m_centerStartChooser.getSelected();// 10 start
                sourceChoice = m_sourceStartChooser.getSelected();// 20 start

                boolean temp = ampChoice != ampChoiceLast || centerChoice != centerChoiceLast
                                || sourceChoice != sourceChoiceLast;

                ampChoiceLast = ampChoice;
                centerChoiceLast = centerChoice;
                sourceChoiceLast = sourceChoice;

                return temp;
        }

        public int selectAndLoadPathFiles() {
                finalChoice = 0;
                m_pf.usedPathFiles.clear();
                if (ampChoice != 0 && centerChoice == 10 && sourceChoice == 20)
                        finalChoice = ampChoice;

                if (ampChoice == 0 && centerChoice != 10 && sourceChoice == 20)
                        finalChoice = centerChoice;

                if (ampChoice == 0 && centerChoice == 10 && sourceChoice != 20)
                        finalChoice = sourceChoice;

                        SmartDashboard.putNumber("FC",finalChoice);

                if (finalChoice != 0) {

                        m_pf.setFilenames(finalChoice);

                        m_pf.loadPathFiles(m_pf.usedPathFiles);

                }

                return finalChoice;

        }

}