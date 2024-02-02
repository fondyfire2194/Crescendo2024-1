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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.commands.DoNothing;

public class ShooterAngleSubsystem extends SubsystemBase {

  CANSparkMax shooterangleMotor;
  SparkPIDController shooterangleController;
  RelativeEncoder shooterangleEncoder;
  private double simPosition;
  private double commandDegrees;

  /** Creates a new ShooterAngle. */
  public ShooterAngleSubsystem() {

    shooterangleMotor = new CANSparkMax(Constants.CANIDConstants.shooterangleID, MotorType.kBrushless);
    shooterangleController = shooterangleMotor.getPIDController();
    shooterangleEncoder = shooterangleMotor.getEncoder();
    configMotor(shooterangleMotor, shooterangleEncoder, shooterangleController, true);

    Shuffleboard.getTab("ShooterSubsystem").add(this).withSize(3, 1)
        .withPosition(6, 0);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("ShooterAngleDegrees", () -> round2dp(getPosition(), 2))
        .withSize(1, 1)
        .withPosition(6, 1);

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

  public void jog(double speed) {
    shooterangleMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public Command jogCommand(Double speed) {
    return Commands.run(() -> jog(speed), this).withName("Jog");
  }

  public double getPosition() {
    if (RobotBase.isReal())
      return shooterangleEncoder.getPosition();
    else {
      simPosition += shooterangleEncoder.getVelocity() / 50;
      return simPosition;
    }
  }

  public double getVelocity() {
    return shooterangleEncoder.getVelocity();
  }

  public void setVelocity(double speed) {
    shooterangleMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void positionShooterAngle(double degrees) {
    commandDegrees = degrees;
    if (RobotBase.isReal())
      shooterangleController.setReference(degrees, ControlType.kPosition);
    else
      setVelocity(degrees - getPosition());
  }

  public Command positionShooterAngleCommand(double degrees) {
    return new DoNothing();
  }

  public Command positionHold() {
    return Commands.run(() -> positionShooterAngle(commandDegrees), this).withName("Hold");
  }

  public Command positionToHomeCommand() {
    return Commands.run(() -> positionShooterAngle(Constants.ElevatorConstants.elevatorPositionToIntake), this)
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public double getPositionError() {
    return commandDegrees - getPosition();
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
