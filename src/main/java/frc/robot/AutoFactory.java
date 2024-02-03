// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DoNothing;
import frc.robot.commands.CenterStart.CenterStartCommand1;
import frc.robot.commands.CenterStart.CenterStartCommand2;
import frc.robot.subsystems.BottomShooterRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoldNoteSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopShooterRollerSubsystem;

/** Add your docs here. */
public class AutoFactory {

    private final SwerveSubsystem m_swerve;

    private final ElevatorSubsystem m_elevator;

    private final IntakeSubsystem m_intake;

    private final TopShooterRollerSubsystem m_topshooter;

    private final BottomShooterRollerSubsystem m_bottomshooter;

    private final ShooterAngleSubsystem m_shooterangle;

    private final HoldNoteSubsystem m_holdnote;

    private final CommandFactory m_cf;

    public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_centerStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    int finalChoice = 0;

    int ampChoice;
    int ampChoiceLast;

    int centerChoice;
    int centerChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public int validStartChoice = 0;

    List<String> usedPathFiles = new ArrayList<>();

    public ArrayList<PathPlannerPath> activePaths = new ArrayList<PathPlannerPath>(5);

    public AutoFactory(CommandFactory cf, SwerveSubsystem swerve, ElevatorSubsystem elevator,
            IntakeSubsystem intake, HoldNoteSubsystem holdNote, TopShooterRollerSubsystem topShooter,
            BottomShooterRollerSubsystem bottomShooter, ShooterAngleSubsystem shooterAngle) {
        m_cf = cf;
        m_swerve = swerve;
        m_elevator = elevator;
        m_intake = intake;
        m_holdnote = holdNote;
        m_topshooter = topShooter;
        m_bottomshooter = bottomShooter;
        m_shooterangle = shooterAngle;

        m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
        m_startDelayChooser.addOption("One Second", 1.);
        m_startDelayChooser.addOption("Two Seconds", 2.);
        m_startDelayChooser.addOption("Three Second", 3.);
        m_startDelayChooser.addOption("Four Seconds", 4.);
        m_startDelayChooser.addOption("Five Seconds", 4.);

        m_ampStartChooser.setDefaultOption("Not Used", 0);
        m_ampStartChooser.addOption("Not Assigned", 1);
        m_ampStartChooser.addOption("Not Assigned", 2);

        m_centerStartChooser.setDefaultOption("Not Used", 10);
        m_centerStartChooser.addOption("Score 4", 11);
        m_centerStartChooser.addOption("Paths Test", 12);

        m_sourceStartChooser.setDefaultOption("Not Used", 20);
        m_sourceStartChooser.addOption("Not Assigned", 21);
        m_sourceStartChooser.addOption("Not Assigned", 22);

        Shuffleboard.getTab("Autonomous").add("DelayChooser", m_startDelayChooser)
                .withSize(2, 1).withPosition(0, 0);
        Shuffleboard.getTab("Autonomous").add("AmpStart", m_ampStartChooser)
                .withSize(2, 1).withPosition(2, 0);
        Shuffleboard.getTab("Autonomous").add("CenterStart", m_centerStartChooser)
                .withSize(2, 1).withPosition(4, 0);
        Shuffleboard.getTab("Autonomous").add("SourceStart", m_sourceStartChooser)
                .withSize(2, 1).withPosition(6, 0);
        Shuffleboard.getTab("Autonomous").addNumber("Choice Must Not Be Zero", () -> validStartChoice)
                .withSize(2, 1).withPosition(2, 1);
        Shuffleboard.getTab("Autonomous").addBoolean("Valid Choice", () -> validStartChoice != 0)
                .withSize(2, 1).withPosition(4, 1)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

        Shuffleboard.getTab("Autonomous").addNumber("Number Files", () -> usedPathFiles.size())
                .withSize(2, 1).withPosition(6, 1);

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

        usedPathFiles.clear();

        if (ampChoice != 0 && centerChoice == 10 && sourceChoice == 20)
            finalChoice = ampChoice;

        if (ampChoice == 0 && centerChoice != 10 && sourceChoice == 20)
            finalChoice = centerChoice;

        if (ampChoice == 0 && centerChoice == 10 && sourceChoice != 20)
            finalChoice = sourceChoice;

        if (finalChoice != 0 && finalChoice == centerChoice) {

            setFilenames(finalChoice);

            loadPathFiles(usedPathFiles);

        }

        return finalChoice;

    }

    private List<String> setFilenames(int index) {

        usedPathFiles.clear();

        switch (finalChoice) {

            case 11:

                usedPathFiles.add("CentOneP1");
                usedPathFiles.add("CentOneP1R");
                usedPathFiles.add("CentOneP2");
                usedPathFiles.add("CentOneP2R");
                usedPathFiles.add("CentOneP3");
                usedPathFiles.add("CentOneP3R");

                return usedPathFiles;

            case 12:

                usedPathFiles.add("CentOneP1");
                usedPathFiles.add("CentOneP1R");
                usedPathFiles.add("CentOneP2");
                usedPathFiles.add("CentOneP2R");
                usedPathFiles.add("CentOneP3");
                usedPathFiles.add("CentOneP3R");

                return usedPathFiles;

            default:
                return usedPathFiles;

        }

    }

    private Command finalCommand(int choice) {

        switch (choice) {
            case 1:
                return new DoNothing();
            case 2:
                return new DoNothing();
            case 3:
                return new DoNothing();
            case 11:
                return new CenterStartCommand1(this,m_cf, m_swerve, m_elevator, m_intake, m_holdnote, m_topshooter,
                        m_bottomshooter, m_shooterangle).withName("CC1");
            case 12:
                return new CenterStartCommand2(this, m_swerve).withName("CC2");
            case 13:
                return new DoNothing();
            case 21:
                return new DoNothing();
            case 22:
                return new DoNothing();
            case 23:
                return new DoNothing();
            default:
                return new DoNothing();

        }
    }

    public Command getAutonomousCommand() {
        return finalCommand(finalChoice);
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

}