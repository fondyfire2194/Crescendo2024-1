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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;

public class ShooterAngleSubsystem extends SubsystemBase {

  CANSparkMax shooterangleMotor;
  SparkPIDController shooterangleController;
  RelativeEncoder shooterangleEncoder;
  private double simPosition;
  private double commandDegrees;
  private double velocitySet;

  /** Creates a new ShooterAngle. */
  public ShooterAngleSubsystem() {

    // shooterangleMotor = new CANSparkMax(Constants.CANIDConstants.shooterangleID, MotorType.kBrushless);
    // shooterangleController = shooterangleMotor.getPIDController();
    // shooterangleEncoder = shooterangleMotor.getEncoder();
    // configMotor(shooterangleMotor, shooterangleEncoder, shooterangleController, true);

//     Shuffleboard.getTab("ShooterSubsystem").add(this).withSize(3, 1)
//         .withPosition(6, 0);

//     Shuffleboard.getTab("ShooterSubsystem").addNumber("ActlDegrees", () -> round2dp(getPosition(), 2))
//         .withSize(1, 1)
//         .withPosition(6, 1);

//     Shuffleboard.getTab("ShooterSubsystem").addNumber("CmdDegrees", () -> round2dp(commandDegrees, 2))
//         .withSize(1, 1)
//         .withPosition(7, 1);

//     Shuffleboard.getTab("ShooterSubsystem").add("ShooterToMaxAngle",
//         positionCommand(Constants.ShooterAngleConstants.shooterangleMaxDegrees))
//         .withSize(2, 1)
//         .withPosition(6, 2);

//     Shuffleboard.getTab("ShooterSubsystem").add("ShooterToIntake",
//         positionToIntakeCommand())
//         .withSize(2, 1)
//         .withPosition(6, 3);

//  Shuffleboard.getTab("ShooterSubsystem").add("SetShooterAngleKp", setShooterAngleKpCommand())
//         .withPosition(8, 1).withSize(1, 1);

//     Shuffleboard.getTab("ShooterSubsystem").addNumber("ShooterAngleKpSet",
//         () -> Pref.getPref("ShooterAngleKp"))
//         .withPosition(8, 2).withSize(1, 1);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("ShooterAngleKp", () -> getShooterAngleKp())
        .withPosition(8, 3).withSize(1, 1);


    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(shooterangleMotor, 3, 5600);
      shooterangleEncoder.setVelocityConversionFactor(.001 / 60);
      shooterangleEncoder.setPositionConversionFactor(.001);
    }

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ShooterAngleConstants.shooterangleContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ShooterAngleConstants.shooterangleIdleMode);
    encoder.setVelocityConversionFactor(Constants.ShooterAngleConstants.shooterangleConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ShooterAngleConstants.shooterangleConversionPositionFactor);
    controller.setP(Constants.ShooterAngleConstants.shooterangleKP);
    controller.setI(Constants.ShooterAngleConstants.shooterangleKI);
    controller.setD(Constants.ShooterAngleConstants.shooterangleKD);
    controller.setFF(Constants.ShooterAngleConstants.shooterangleKFF);
    motor.enableVoltageCompensation(Constants.ShooterAngleConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotor() {
    shooterangleMotor.stopMotor();
    shooterangleMotor.setVoltage(0);

  }

  public void jog(double speed) {
    shooterangleMotor.setVoltage(speed * 12);
    commandDegrees = getPosition();
  }

  public Command jogCommand(double speed) {
    commandDegrees = getPosition();
    return Commands.runEnd(() -> jog(speed), () -> stopMotor(), this)
        .withName("Jog");
  }

  public double getPosition() {
    if (RobotBase.isReal())
      return shooterangleEncoder.getPosition();
    else {
      simPosition += getVelocity() / 100;
      return simPosition;
    }
  }

  public double getVelocity() {
    if (RobotBase.isReal())
      return shooterangleEncoder.getVelocity();
    else
      return velocitySet;
  }

  public void setVelocity(double speed) {
    velocitySet = speed;
    shooterangleMotor.setVoltage(speed * 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void positionShooterAngle(double degrees) {
    commandDegrees = degrees;
    if (RobotBase.isReal())
      shooterangleController.setReference(commandDegrees, ControlType.kPosition);
    else
      setVelocity(commandDegrees - getPosition());
  }

  public Command positionHold() {
    return Commands.run(() -> positionShooterAngle(commandDegrees), this).withName("Hold");
  }

  public Command positionCommand(double angle) {
    commandDegrees = angle;
    return Commands.runOnce(() -> positionShooterAngle(angle), this)
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public Command positionToIntakeCommand() {
    return Commands
        .runOnce(() -> positionShooterAngle(Constants.ShooterAngleConstants.shooteranglePositionToIntake), this)
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public double getPositionError() {
    return commandDegrees - getPosition();
  }

  public Command setShooterAngleKpCommand() {
    return Commands.runOnce(() -> setShooterAngleKp());
  }

  public void setShooterAngleKp() {
    shooterangleController.setP(Pref.getPref("ShooterAngleKp"));
  }

  public double getShooterAngleKp() {
    return shooterangleController.getP();
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
