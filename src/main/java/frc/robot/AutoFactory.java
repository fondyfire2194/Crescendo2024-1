// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.AuthenticationNotSupportedException;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoFactory {

    private final SwerveSubsystem m_swerve;

    private final IntakeSubsystem m_intake;

    private final ShooterSubsystem m_shooter;

    private final CommandFactory m_cf;

    public final SendableChooser<Integer> m_ampStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_centerStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_sourceStartChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    private SequentialCommandGroup ampChoice1 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup ampChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup ampChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    private Command centerChoice1 = new DoNothing();
    private SequentialCommandGroup centerChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup centerChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    private SequentialCommandGroup sourceChoice1 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup sourceChoice2 = new SequentialCommandGroup(new DoNothing(), new DoNothing());
    private SequentialCommandGroup sourceChoice3 = new SequentialCommandGroup(new DoNothing(), new DoNothing());

    int finalChoice = 0;

    public AutoFactory(CommandFactory cf, SwerveSubsystem swerve, IntakeSubsystem intake, ShooterSubsystem shooter) {
        m_swerve = swerve;
        m_intake = intake;
        m_shooter = shooter;
        m_cf = cf;

        m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
        m_startDelayChooser.addOption("One Second", 1.);
        m_startDelayChooser.addOption("Two Seconds", 2.);
        m_startDelayChooser.addOption("Three Second", 3.);
        m_startDelayChooser.addOption("Four Seconds", 4.);
        m_startDelayChooser.addOption("Five Seconds", 4.);

        m_ampStartChooser.setDefaultOption("Not Used", 0);
        m_ampStartChooser.addOption("Choice 1", 1);
        m_ampStartChooser.addOption("Choice 2", 2);

        m_centerStartChooser.setDefaultOption("Not Used", 10);
        m_centerStartChooser.addOption("Score 4", 11);
        m_centerStartChooser.addOption("Choice 2", 12);

        m_sourceStartChooser.setDefaultOption("Not Used", 20);
        m_sourceStartChooser.addOption("Choice 1", 21);
        m_sourceStartChooser.addOption("Choice 2", 22);

        centerChoice1 = m_cf.centerChoice2;

        Shuffleboard.getTab("Autonomous").add("DelayChooser", m_startDelayChooser)
                .withSize(2, 1).withPosition(0, 0);
        Shuffleboard.getTab("Autonomous").add("AmpStart", m_ampStartChooser)
                .withSize(2, 1).withPosition(2, 0);
        Shuffleboard.getTab("Autonomous").add("CenterStart", m_centerStartChooser)
                .withSize(2, 1).withPosition(4, 0);
        Shuffleboard.getTab("Autonomous").add("SourceStart", m_sourceStartChooser)
                .withSize(2, 1).withPosition(6, 0);

    }

    private int selectAndBuildAuto() {

        int ampChoice = m_ampStartChooser.getSelected();// 0 start

        int centerChoice = m_centerStartChooser.getSelected();// 10 start

        int sourceChoice = m_sourceStartChooser.getSelected();// 20 start

        if (ampChoice != 0 && centerChoice == 10 && sourceChoice == 20)
            finalChoice = ampChoice;

        if (ampChoice == 0 && centerChoice != 10 && sourceChoice == 20)
            finalChoice = centerChoice;

        if (ampChoice == 0 && centerChoice == 10 && sourceChoice != 20)
            finalChoice = sourceChoice;

        return finalChoice;
    }

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
        SmartDashboard.putNumber("FinChoice", finalChoice);
        SmartDashboard.putData("FC", finalCommand());
        return finalCommand();
    }

    public PathPlannerPath getPath(String pathname) {
        return PathPlannerPath.fromPathFile(pathname);
    }

    public void followPath(String pathName) {
        m_swerve.followPathCommand(pathName);
    }

}