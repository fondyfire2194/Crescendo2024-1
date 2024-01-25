// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  CANSparkMax bottomRoller;
  CANSparkMax topRoller;
  SparkPIDController bottomController;
  SparkPIDController topController;
  RelativeEncoder topEncoder;
  RelativeEncoder bottomEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    bottomRoller = new CANSparkMax(Constants.Shooter.bottomShooterID, MotorType.kBrushless);
    topRoller = new CANSparkMax(Constants.Shooter.topShooterID, MotorType.kBrushless);
    bottomController = bottomRoller.getPIDController();
    topController = topRoller.getPIDController();
    topEncoder = topRoller.getEncoder();
    bottomEncoder = bottomRoller.getEncoder();
    configMotor(bottomRoller, bottomEncoder, bottomController, true);
    configMotor(topRoller, topEncoder, topController, false);
  }

private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.Shooter.shooterContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.Shooter.shooterIdleMode);
    encoder.setVelocityConversionFactor(Constants.Shooter.shooterConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.Shooter.shooterConversionPositionFactor);
    controller.setP(Constants.Shooter.shooterKP);
    controller.setI(Constants.Shooter.shooterKI);
    controller.setD(Constants.Shooter.shooterKD);
    controller.setFF(Constants.Shooter.shooterKFF);
    motor.enableVoltageCompensation(Constants.Shooter.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void setRPM(double rpm) {
    bottomController.setReference(rpm, ControlType.kVelocity);
    topController.setReference(rpm, ControlType.kVelocity);
  }

  public void stopMotors() {
    topRoller.stopMotor();
    bottomRoller.stopMotor();
    bottomController.setReference(0, ControlType.kVelocity);
    topController.setReference(0, ControlType.kVelocity);
  }

  public double getRPMBottom() {
    return bottomEncoder.getVelocity();
  }

  public double getRPMTop() {
    return topEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
