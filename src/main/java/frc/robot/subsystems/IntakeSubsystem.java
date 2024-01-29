// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intakeMotor;
  SparkPIDController intakeController;
  RelativeEncoder intakeEncoder;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeID, MotorType.kBrushless);
    intakeController = intakeMotor.getPIDController();
    intakeEncoder = intakeMotor.getEncoder();
    configMotor(intakeMotor, intakeEncoder, intakeController, true);
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
  }

  public void stopMotor() {
    intakeController.setReference(0, ControlType.kVelocity);
    intakeMotor.stopMotor();
  }

  public double getRPM() {
    return intakeEncoder.getVelocity();
  }

  public Command runIntakeCommand() {
    return this.run(() -> intakeController.setReference(Pref.getPref("IntakeRPM"), ControlType.kVelocity));
  }

  public Command feedShooterCommand() {
    return this.run(() -> intakeController.setReference(Pref.getPref("IntakeRPM"), ControlType.kVelocity));
  }

  public Command stopIntakeCommand() {
    return this.runOnce(() -> stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeRPM", getRPM());
    SmartDashboard.putNumber("IntakeCANVer", intakeMotor.getFirmwareVersion());

  }

  public void jog(double speed) {
    intakeMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }
}
