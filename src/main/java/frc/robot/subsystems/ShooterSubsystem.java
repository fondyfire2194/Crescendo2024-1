// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax bottomRoller;
  CANSparkMax topRoller;
  SparkPIDController bottomController;
  SparkPIDController topController;
  RelativeEncoder topEncoder;
  RelativeEncoder bottomEncoder;
  double topRollerCommandRPM = 500;
  double bottomRollerCommandRPM = 500;
  double commandRPM = 500;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    bottomRoller = new CANSparkMax(Constants.CANIDConstants.bottomShooterID, MotorType.kBrushless);
    topRoller = new CANSparkMax(Constants.CANIDConstants.topShooterID, MotorType.kBrushless);
    bottomController = bottomRoller.getPIDController();
    topController = topRoller.getPIDController();
    topEncoder = topRoller.getEncoder();
    bottomEncoder = bottomRoller.getEncoder();
    configMotor(bottomRoller, bottomEncoder, bottomController, true);
    configMotor(topRoller, topEncoder, topController, false);
 
    Shuffleboard.getTab("ShooterSubsystem").add(this).withSize(2, 1);
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

  public void setShooterRPM(double rpm) {
    bottomController.setReference(rpm, ControlType.kVelocity);
    topController.setReference(rpm, ControlType.kVelocity);
  }

  public Command setShooterSpeed(double rpm) {
    return Commands.runOnce(() -> commandRPM = rpm);
  }

  public void stopMotors() {

    bottomController.setReference(0, ControlType.kVelocity);
    topController.setReference(0, ControlType.kVelocity);
    topRoller.stopMotor();
    bottomRoller.stopMotor();
  }

  public Command runTopRollerCommand() {
    return this.run(() -> topController.setReference(Pref.getPref("ShooterRPM"), ControlType.kVelocity));
  }

  public Command runBottomRollerCommand() {
    return this.run(() -> bottomController.setReference(Pref.getPref("ShooterRPM"), ControlType.kVelocity));
  }

  public Command stopShootersCommand() {
    return this.runOnce(() -> stopMotors());
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
    SmartDashboard.putNumber("CANTopRoller", topRoller.getFirmwareVersion());
  }
}
