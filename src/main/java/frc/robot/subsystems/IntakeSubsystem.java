// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Pref;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intakeMotor;
  SparkPIDController intakeController;
  RelativeEncoder intakeEncoder;
  double intakeRPM = 1000;
  double feedShooterRPM = 1500;

  public boolean showIntake = false;

  private final TimeOfFlight m_detectNoteSensor = new TimeOfFlight(CANIDConstants.intakeDistanceSensorID);
  private boolean lookForNote;
  private double lookForNoteStartTime;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_detectNoteSensor.setRangingMode(RangingMode.Short, 40);
    intakeMotor = new CANSparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, intakeController, true);
    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(intakeMotor, 3, 5600);
  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.IntakeConstants.intakeContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.IntakeConstants.intakeIdleMode);
    encoder.setVelocityConversionFactor(Constants.IntakeConstants.intakeConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.IntakeConstants.intakeConversionPositionFactor);
    controller.setP(Constants.IntakeConstants.intakeKP);
    controller.setI(Constants.IntakeConstants.intakeKI);
    controller.setD(Constants.IntakeConstants.intakeKD);
    controller.setFF(Constants.IntakeConstants.intakeKFF);
    motor.enableVoltageCompensation(Constants.IntakeConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);

    setIntakeKp();

    if (showIntake) {

      Shuffleboard.getTab("IntakeSubsystem").add(this)
          .withSize(2, 1)
          .withPosition(0, 0);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("ActualRPM",
          () -> round2dp(getRPM(), 0))
          .withSize(1, 1)
          .withPosition(1, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("CommandRPM",
          () -> intakeRPM)
          .withSize(1, 1)
          .withPosition(0, 1);

      Shuffleboard.getTab("IntakeSubsystem").addNumber("NoteSensorInches",
          () -> round2dp(getSensorDistanceInches(), 1))
          .withSize(1, 1)
          .withPosition(0, 2);

      Shuffleboard.getTab("IntakeSubsystem").addBoolean("NoteSensed", () -> noteAtIntake())
          .withSize(1, 1)
          .withPosition(1, 2)
          .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

      Shuffleboard.getTab("IntakeSubsystem").add("SetKpKd", setIntakeKpKdCommand())
          .withSize(1, 1)
          .withPosition(0, 3)
          .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

      Shuffleboard.getTab("IntakeSubsystem").addNumber("IntakeAmps",
          () -> round2dp(getAmps(), 1))
          .withSize(1, 1)
          .withPosition(1, 3);

    }
  }

  public double getSensorDistanceInches() {
    return Units.metersToInches(m_detectNoteSensor.getRange() / 1000);
  }

  public void stopMotor() {
    if (RobotBase.isReal())
      intakeController.setReference(0, ControlType.kVelocity);
    intakeMotor.stopMotor();
  }

  public double getRPM() {
    return intakeEncoder.getVelocity();
  }

  public void runIntake(double rpm) {
    if (RobotBase.isReal()) {
      intakeRPM = rpm;
      // intakeController.setReference(intakeRPM, ControlType.kVelocity);
      intakeController.setReference(rpm, ControlType.kVelocity);
    } else
      intakeMotor.setVoltage(rpm * 12 / IntakeConstants.maxIntakeMotorRPM);
  }

  public boolean noteAtIntake() {
    return getSensorDistanceInches() < IntakeConstants.noteSensedInches
        || lookForNote && Timer.getFPGATimestamp() > lookForNoteStartTime + 5;
  }

  public Command intakeToSensorCommand() {
    lookForNote = true;
    lookForNoteStartTime = Timer.getFPGATimestamp();
    return Commands.run(() -> runIntake(Pref.getPref("Intk2SensorRPM")), this)
        .until(() -> noteAtIntake())
        .andThen(new SequentialCommandGroup(new WaitCommand(.3), stopIntakeCommand()));
  }

  public Command feedShooterCommand() {
    return Commands.runOnce(() -> runIntake(Pref.getPref("Intk2ShtrRPM")), this)
        .andThen(new SequentialCommandGroup(new WaitCommand(1), stopIntakeCommand()));
  }

  public Command runIntakeCommand() {
    return Commands.runOnce(() -> runIntake(intakeRPM), this);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public void setIntakeKp() {
    intakeController.setP(Pref.getPref("IntakeKp"));
  }

  public void setIntakeKd() {
    intakeController.setP(Pref.getPref("IntakeKd"));
  }

  public Command setIntakeKpKdCommand() {
    return Commands.runOnce(() -> setIntakeKp()).alongWith(Commands.runOnce(() -> setIntakeKd()));
  }

  public double getIntakeKp() {
    return intakeController.getP();
  }

  public double getIntakeKd() {
    return intakeController.getD();
  }

  public void incrementIntakeRPM() {
    if (intakeRPM < IntakeConstants.maxIntakeMotorRPM)
      intakeRPM += 100;
  }

  public void decrementIntakeRPM() {
    if (intakeRPM > 500)
      intakeRPM -= 100;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public double getAmps() {
    return intakeMotor.getOutputCurrent();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void jog(double speed) {
    intakeMotor.setVoltage(speed * 12);
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}
