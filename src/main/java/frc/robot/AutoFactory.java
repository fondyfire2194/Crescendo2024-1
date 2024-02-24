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
                m_ampStartChooser.addOption("Score 3 Center OuterInner", 2);
                m_ampStartChooser.addOption("Score 3 CenterInnerOuter", 3);
                m_ampStartChooser.addOption("Score 2 Cemter Middle Via Stage", 4);

                m_centerStartChooser.setDefaultOption("Not Used", 10);
                m_centerStartChooser.addOption("Score 4 Shoot from Subwoofer", 11);
                m_centerStartChooser.addOption("Score 4 Shoot From Pickup", 12);
                m_centerStartChooser.addOption("Score 3 Through Stage", 13);

                m_sourceStartChooser.setDefaultOption("Not Used", 20);
                m_sourceStartChooser.addOption("LeaveZone", 21);
                m_sourceStartChooser.addOption("Score 3 Center InnerOuter", 22);
                m_sourceStartChooser.addOption("Score 3 Center OuterInner", 23);

                Shuffleboard.getTab("Autonomous").add("AmpStart", m_ampStartChooser)
                                .withSize(3, 1).withPosition(0, 0);

                Shuffleboard.getTab("Autonomous").add("CenterStart", m_centerStartChooser)
                                .withSize(3, 1).withPosition(3, 0);

                Shuffleboard.getTab("Autonomous").add("SourceStart", m_sourceStartChooser)
                                .withSize(3, 1).withPosition(6, 0);

                Shuffleboard.getTab("Autonomous").add("DelayChooser", m_startDelayChooser)
                                .withSize(1, 1).withPosition(9, 0);

                Shuffleboard.getTab("Autonomous").addBoolean("Valid Choice", () -> finalChoice != 0)
                                .withSize(10, 1).withPosition(0, 1)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("Cameras", BuiltInLayouts.kList).withPosition(0, 2)
                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));

                camLayout.addBoolean("FrontLeftCamera", () -> CameraConstants.frontLeftCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("FrontRightCamera", () -> CameraConstants.frontRightCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                camLayout.addBoolean("RearCamera", () -> CameraConstants.rearCamera.isActive)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                       
                ShuffleboardLayout fileCheckLayout = Shuffleboard.getTab("Autonomous")
                .getLayout("PathFilesOK", BuiltInLayouts.kList).withPosition(1, 2)
                .withSize(1, 2).withProperties(Map.of("Label position", "TOP"));         

               fileCheckLayout.addBoolean("AmpFiles", ()->m_pf.ampFilesOK);
               fileCheckLayout.addBoolean("CenterFiles", ()->m_pf.centerFilesOK);
               fileCheckLayout.addBoolean("SourceFiles", ()->m_pf.sourceFilesOK);

                // rearLayout.addNumber("BatteryVolts", () ->
                // RobotController.getBatteryVoltage());

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
                if (ampChoice != 0 && centerChoice == 10 && sourceChoice == 20)
                        finalChoice = ampChoice;

                if (ampChoice == 0 && centerChoice != 10 && sourceChoice == 20)
                        finalChoice = centerChoice;

                if (ampChoice == 0 && centerChoice == 10 && sourceChoice != 20)
                        finalChoice = sourceChoice;

                SmartDashboard.putNumber("FC", finalChoice);

                if (finalChoice > 0 && finalChoice < 10) {
                        m_pf.linkAmpPaths();
                        SmartDashboard.putNumber("LENGTHAmp", m_pf.pathMaps.size());
                
                }

                if (finalChoice > 10 && finalChoice < 20) {
                        m_pf.linkCenterPaths();
                  
        
                }
                if (finalChoice > 20 && finalChoice < 30) {
                        m_pf.linkSourcePaths();
                        SmartDashboard.putNumber("LENGTHSource", m_pf.pathMaps.size());
                }

                return finalChoice;
        }

}