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
import frc.robot.commands.CenterStartCommandFactory;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoFactory {

    private final SwerveSubsystem m_swerve;

    private final IntakeSubsystem m_intake;

    private final ShooterSubsystem m_shooter;

    private final CenterStartCommandFactory m_csf;

    public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_centerStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    private Command ampChoice1 = new DoNothing();
    private Command ampChoice2 = new DoNothing();
    private Command ampChoice3 = new DoNothing();

    private Command centerChoice1 = new DoNothing();
    private Command centerChoice2 = new DoNothing();
    private Command centerChoice3 = new DoNothing();

    private Command sourceChoice1 = new DoNothing();
    private Command sourceChoice2 = new DoNothing();
    private Command sourceChoice3 = new DoNothing();

    int finalChoice = 0;

    int ampChoice;
    int ampChoiceLast;

    int centerChoice;
    int centerChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public int validStartChoice = 0;

    List<String> usedPathFiles = new ArrayList<>();

    public AutoFactory(CenterStartCommandFactory csf, SwerveSubsystem swerve, IntakeSubsystem intake,
            ShooterSubsystem shooter) {
        m_swerve = swerve;
        m_intake = intake;
        m_shooter = shooter;
        m_csf = csf;

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
        m_centerStartChooser.addOption("Not Assigned", 12);

        m_sourceStartChooser.setDefaultOption("Not Used", 20);
        m_sourceStartChooser.addOption("Not Assigned", 21);
        m_sourceStartChooser.addOption("Not Assigned", 22);

        centerChoice1 = m_csf.centerChoice1;

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

        if (finalChoice != 0)

            setFilenames(finalChoice);

        return finalChoice;
    }

    private List<String> setFilenames(int index) {

        usedPathFiles.clear();

        switch (finalChoice) {

            case 11:
                usedPathFiles.add("AutoPath1");
                usedPathFiles.add("AutoPath2");
                usedPathFiles.add("AutoPath3");
                return usedPathFiles;

            default:
                return usedPathFiles;

        }

    }

    private Command finalCommand(int choice) {

        switch (choice) {
            case 1:
                return ampChoice1;
            case 2:
                return ampChoice2;
            case 3:
                return ampChoice3;
            case 11:
                return centerChoice1;
            case 12:
                return centerChoice2;
            case 13:
                return centerChoice3;
            case 21:
                return sourceChoice1;
            case 22:
                return sourceChoice2;
            case 23:
                return sourceChoice3;
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

    public void followPath(String pathName) {
        m_swerve.followPathCommand(pathName);
    }

}