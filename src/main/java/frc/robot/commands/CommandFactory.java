// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CommandFactory {

    private final SwerveSubsystem m_swerve;

    private final IntakeSubsystem m_intake;

    private final ShooterSubsystem m_shooter;

    public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_centerStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    int finalChoice = 0;

    public CommandFactory(SwerveSubsystem swerve, IntakeSubsystem intake, ShooterSubsystem shooter) {
        m_swerve = swerve;
        m_intake = intake;
        m_shooter = shooter;

        m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
        m_startDelayChooser.addOption("One Second", 1.);
        m_startDelayChooser.addOption("Two Seconds", 2.);
        m_startDelayChooser.addOption("Three Second", 3.);
        m_startDelayChooser.addOption("Four Seconds", 4.);
        m_startDelayChooser.addOption("Five Seconds", 4.);

        m_ampStartChooser.setDefaultOption("", 0);
        m_ampStartChooser.addOption("Choice 1", 1);
        m_ampStartChooser.addOption("Choice 2", 2);

        m_centerStartChooser.setDefaultOption("", 10);
        m_centerStartChooser.addOption("Choice 1", 11);
        m_centerStartChooser.addOption("Choice 2", 12);

        m_sourceStartChooser.setDefaultOption("", 20);
        m_sourceStartChooser.addOption("Choice 1", 21);
        m_sourceStartChooser.addOption("Choice 2", 22);

    }

    private Command selectAndBuildAuto() {

        int ampChoice = m_ampStartChooser.getSelected();

        int centerChoice = m_centerStartChooser.getSelected();

        int sourceChoice = m_sourceStartChooser.getSelected();

        if (centerChoice == 0 && sourceChoice == 0 && ampChoice != 0)
            finalChoice = ampChoice;

        if (centerChoice == 0 && ampChoice == 0 && sourceChoice != 0)
            finalChoice = sourceChoice;

        if (sourceChoice == 0 && ampChoice == 0 && centerChoice != 0)
            finalChoice = centerChoice;

        return new DoNothing();
    }

    private SequentialCommandGroup ampChoice1 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup ampChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup ampChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    private SequentialCommandGroup centerChoice1 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup centerChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup centerChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    private SequentialCommandGroup sourceChoice1 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup sourceChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup sourceChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    private Command finalCommand() {

        switch (finalChoice) {
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
        selectAndBuildAuto();
        return finalCommand();
    }

    public PathPlannerPath getPath(String pathname){
        return PathPlannerPath.fromPathFile(pathname);
    }

    public void followPath(String pathName){
        m_swerve.followPathCommand(pathName);
    }

}