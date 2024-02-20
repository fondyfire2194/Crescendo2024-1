// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Pref;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax rightRoller;

  SparkPIDController rightController;
  RelativeEncoder rightEncoder;

  CANSparkMax leftRoller;

  SparkPIDController leftController;

  RelativeEncoder leftEncoder;

  double leftRollerCommandRPM = 500;

  double rightRollerCommandRPM = 500;

  double commandRPM = 500;

  public double testjs;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    rightRoller = new CANSparkMax(Constants.CANIDConstants.rightShooterID, MotorType.kBrushless);
    rightController = rightRoller.getPIDController();
    rightEncoder = rightRoller.getEncoder();
    configMotor(rightRoller, rightEncoder, rightController, true);

    leftRoller = new CANSparkMax(Constants.CANIDConstants.leftShooterID, MotorType.kBrushless);
    leftController = leftRoller.getPIDController();
    leftEncoder = leftRoller.getEncoder();
    configMotor(leftRoller, leftEncoder, leftController, true);

    // Shuffleboard.getTab("ShooterSubsystem").add(this)
    //     .withPosition(0, 0).withSize(2, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addNumber("TestRPMSet", () -> testjs * ShooterConstants.maxShooterMotorRPM)
    //     .withPosition(0, 1).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addNumber("LeftRPM",
    //     () -> round2dp(getRPMLeft(), 0))
    //     .withPosition(1, 1).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addNumber("RightRPM",
    //     () -> round2dp(getRPMRight(), 0))
    //     .withPosition(2, 1).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").add("SetLeftKpKd", setLeftKpKdCommand())
    //     .withPosition(0, 2).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").add("SetRightKpKd", setRightKpKdCommand())
    //     .withPosition(0, 3).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addNumber("LeftAmps", () -> getLeftAmps())
    //     .withPosition(2, 2).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addNumber("RightAmps", () -> getRightAmps())
    //     .withPosition(2, 3).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addBoolean("LeftAtSpeed", () -> leftAtSpeed())
    //     .withPosition(1, 2).withSize(1, 1);

    // Shuffleboard.getTab("ShooterSubsystem").addBoolean("RightAtSpeed", () -> rightAtSpeed())
    //     .withPosition(1, 3).withSize(1, 1);

    setLeftKpKd();
    setRightKpKd();

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ShooterConstants.shooterContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ShooterConstants.shooterIdleMode);
    encoder.setVelocityConversionFactor(Constants.ShooterConstants.shooterConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ShooterConstants.shooterConversionPositionFactor);
    controller.setP(Constants.ShooterConstants.shooterKP);
    controller.setI(Constants.ShooterConstants.shooterKI);
    controller.setD(Constants.ShooterConstants.shooterKD);
    controller.setFF(Constants.ShooterConstants.shooterKFF);

    motor.enableVoltageCompensation(Constants.ShooterConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void runRightRoller(double rpm) {
    commandRPM = rpm;
    if (RobotBase.isReal())
      rightController.setReference(commandRPM, ControlType.kVelocity);
    else
      rightRoller.setVoltage(commandRPM * 12 / ShooterConstants.maxShooterMotorRPM);

  }

  public void stopMotors() {
    if (RobotBase.isReal()) {
      rightController.setReference(0, ControlType.kVelocity);
      leftController.setReference(0, ControlType.kVelocity);

      leftRoller.stopMotor();
      rightRoller.stopMotor();
    }
  }

  public Command stopShooterCommand() {
    return Commands.runOnce(() -> stopMotors(), this);
  }

  public Command runRightRollerCommand(double rpm) {
    commandRPM = rpm;
    if (RobotBase.isReal())
      return this.runOnce(() -> runRightRoller(commandRPM));
    else
      return Commands.runOnce(() -> rightRoller.setVoltage(.5));
  }

  public double getRPMRight() {
    return rightEncoder.getVelocity();
  }

  public void runLeftRoller() {
    if (RobotBase.isReal())
      leftController.setReference(commandRPM, ControlType.kVelocity);
    else
      leftRoller.setVoltage(commandRPM * 12 / ShooterConstants.maxShooterMotorRPM);

  }

  private void testRunRollers() {
    leftController.setReference(testjs * ShooterConstants.maxShooterMotorRPM * Pref.getPref("LeftMult"),
        ControlType.kVelocity);
    rightController.setReference(testjs * ShooterConstants.maxShooterMotorRPM, ControlType.kVelocity);
  }

  public Command testRunRollerCommand() {
    return Commands.runOnce(() -> testRunRollers(), this);
  }

  public Command runLeftRollerCommand(double rpm) {
    commandRPM = rpm;
    return this.runOnce(() -> runLeftRoller());
  }

  public Command runBothRollersCommand(double leftRPM, double rightRPM) {
    return runLeftRollerCommand(leftRPM).alongWith(runRightRollerCommand(rightRPM));
  }

  public double getRPMLeft() {
    return leftEncoder.getVelocity();
  }

  public Command setLeftKpKdCommand() {
    return Commands.runOnce(() -> setLeftKpKd());
  }

  public Command setRightKpKdCommand() {
    return Commands.runOnce(() -> setRightKpKd());
  }

  public void setLeftKpKd() {
    leftController.setP(Pref.getPref("ShooterLeftKp"));
    leftController.setP(Pref.getPref("ShooterLeftKd"));
  }

  public void setRightKpKd() {
    rightController.setP(Pref.getPref("ShooterRightKp"));
    rightController.setD(Pref.getPref("ShooterRightKd"));
  }

  public double getShooterKp() {
    return rightController.getP();
  }

  public double getShooterKd() {
    return rightController.getD();
  }

  public boolean leftAtSpeed() {
    return commandRPM != 0 && Math.abs(commandRPM - getRPMLeft()) < commandRPM / 20;
  }

  public boolean rightAtSpeed() {
    return commandRPM != 0 && Math.abs(commandRPM - getRPMRight()) < commandRPM / 20;
  }

  public boolean bothAtSpeed() {
    return leftAtSpeed() && rightAtSpeed();
  }

  public double getLeftAmps() {
    return leftRoller.getOutputCurrent();
  }

  public double getRightAmps() {
    return rightRoller.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TESTJS", testjs * ShooterConstants.maxShooterMotorRPM);

  }

  @Override
  public void simulationPeriodic() {

  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}
