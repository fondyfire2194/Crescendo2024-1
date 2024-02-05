// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Pref;

public class RightShooterRollerSubsystem extends SubsystemBase {

  CANSparkMax rightRoller;

  SparkPIDController rightController;
  RelativeEncoder rightEncoder;

  double rightRollerCommandRPM = 500;

  double commandRPM = 500;

  /** Creates a new Shooter. */
  public RightShooterRollerSubsystem() {
    rightRoller = new CANSparkMax(Constants.CANIDConstants.rightShooterID, MotorType.kBrushless);
    rightController = rightRoller.getPIDController();
    rightEncoder = rightRoller.getEncoder();

    configMotor(rightRoller, rightEncoder, rightController, false);

    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(rightRoller, 3, 5600);

    Shuffleboard.getTab("ShooterSubsystem").add(this)
        .withPosition(2, 0).withSize(2, 1);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("RightRPMSet",
        () -> Pref.getPref("RightRPM"))
        .withPosition(3, 1).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").add("StartRight",
        this.setShooterSpeed(Pref.getPref("RightRPM"))
            .andThen(this.runRightRollerCommand()))
        .withPosition(2, 1).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").add("SetRightKp", setRightKpCommand())
        .withPosition(2, 2).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("RightKpSet",
        () -> Pref.getPref("RightShooterKp"))
        .withPosition(3, 2).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("RightRPM",
        () -> round2dp(getRPMRight(), 0))
        .withPosition(2, 3).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("RightKp", () -> getRightShooterKp())
        .withPosition(3, 3).withSize(1, 1);

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
    controller.setFF(Constants.ShooterConstants.shooterKFF / ShooterConstants.maxShooterMotorRPM);
    motor.enableVoltageCompensation(Constants.ShooterConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public Command setShooterSpeed(double rpm) {
    return Commands.runOnce(() -> commandRPM = rpm);
  }

  public void runRoller() {
    if (RobotBase.isReal())
      rightController.setReference(commandRPM, ControlType.kVelocity);
    else
      rightRoller.setVoltage(commandRPM * 12 / ShooterConstants.maxShooterMotorRPM);

  }

  public void stopMotor() {
    if (RobotBase.isReal())
      rightController.setReference(0, ControlType.kVelocity);
    rightRoller.stopMotor();
  }

  public Command stopShooterCommand() {
    return this.runOnce(() -> stopMotor());
  }

  public Command runRightRollerCommand() {
    if (RobotBase.isReal())
      return this.runOnce(() -> runRoller());
    else
      return Commands.runOnce(() -> rightRoller.setVoltage(.5));
  }

  public double getRPMRight() {
    return rightEncoder.getVelocity();
  }

  public Command setRightKpCommand() {
    return Commands.runOnce(() -> setRightShooterKp());
  }

  public void setRightShooterKp() {
    rightController.setP(Pref.getPref("ShooterKp"));
  }

  public double getRightShooterKp() {
    return rightController.getP();
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}
