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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.Constants.ShooterAngleConstants;

public class ShooterAngleSubsystem extends SubsystemBase {

  CANSparkMax shooterangleMotor;
  SparkPIDController shooterangleController;
  RelativeEncoder shooterangleEncoder;

  private double simPosition;
  private double commandDegrees;
  private double velocitySet;
  public double distanceAngle;
  public boolean showShooterAngle = false;

  /** Creates a new ShooterAngle. */
  public ShooterAngleSubsystem() {

    shooterangleMotor = new CANSparkMax(Constants.CANIDConstants.shooterangleID, MotorType.kBrushless);
    shooterangleController = shooterangleMotor.getPIDController();
    shooterangleEncoder = shooterangleMotor.getEncoder();
    configMotor(shooterangleMotor, shooterangleEncoder, shooterangleController, false);

    if (showShooterAngle) {

      Shuffleboard.getTab("ShooterSubsystem").add(this).withSize(3, 1)
          .withPosition(6, 0);

      Shuffleboard.getTab("ShooterSubsystem").addNumber("ActlDegrees", () -> round2dp(getPosition(), 2))
          .withSize(1, 1)
          .withPosition(6, 1);

      Shuffleboard.getTab("ShooterSubsystem").addNumber("CmdDegrees", () -> round2dp(commandDegrees, 2))
          .withSize(1, 1)
          .withPosition(7, 1);

      Shuffleboard.getTab("ShooterSubsystem").addNumber("Velocitu", () -> round2dp(getVelocity(), 2))
          .withSize(1, 1)
          .withPosition(8, 1);

      Shuffleboard.getTab("ShooterSubsystem").add("ShooterToMaxAngle",
          smartPositionShooterAngleCommandToAngle(Constants.ShooterAngleConstants.shooterangleMaxDegrees))
          .withSize(2, 1)
          .withPosition(6, 2);

      Shuffleboard.getTab("ShooterSubsystem").add("SetShooterAngleKp", setShooterAngleKpCommand())
          .withPosition(8, 2).withSize(1, 1);

      Shuffleboard.getTab("ShooterSubsystem").addNumber("ShooterAngleKp", () -> getShooterAngleKp())
          .withPosition(8, 3).withSize(1, 1);

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
    controller.setOutputRange(ShooterAngleConstants.shooteranglekMinOutput,
        ShooterAngleConstants.shooteranglekMaxOutput);
    motor.enableVoltageCompensation(Constants.ShooterAngleConstants.voltageComp);

    controller.setSmartMotionMaxVelocity(Constants.ShooterAngleConstants.maxVelocity, 0);
    controller.setSmartMotionMinOutputVelocity(Constants.ShooterAngleConstants.minVelocity, 0);
    controller.setSmartMotionMaxAccel(Constants.ShooterAngleConstants.maxAcceleration, 0);
    controller.setSmartMotionAllowedClosedLoopError(Constants.ShooterAngleConstants.allowedError, 0);
    controller.setSmartMotionMinOutputVelocity(Constants.ShooterAngleConstants.minVelocity, 0);

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

  public void smartPositionShooterAngle(double degrees) {
    commandDegrees = degrees;
    shooterangleController.setReference(degrees, ControlType.kSmartMotion);
  }

  public Command smartPositionShooterAngleCommandToAngle(double degrees) {
    return Commands.run(() -> smartPositionShooterAngle(degrees), this)
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public Command smartPositionShooterAngleCommand() {
    return Commands.run(() -> smartPositionShooterAngle(commandDegrees), this)
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public Command positionholdCommand() {
    double holdPosition = getPosition();
    return Commands.run(() -> smartPositionShooterAngle(holdPosition));
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

  public void incrementShooterAngle() {
    if (commandDegrees < ShooterAngleConstants.shooterangleMaxDegrees)
      commandDegrees += 1;
  }

  public void decrementShooterAngle() {
    if (commandDegrees < ShooterAngleConstants.shooterangleMinDegrees)
      commandDegrees = 1;
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
