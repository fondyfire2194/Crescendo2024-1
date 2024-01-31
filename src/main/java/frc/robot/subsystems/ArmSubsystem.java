// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.commands.DoNothing;

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax armMotor;
  SparkPIDController armController;
  RelativeEncoder armEncoder;
  private double simPosition;

  /** Creates a new Arm. */
  public ArmSubsystem() {

    armMotor = new CANSparkMax(Constants.CANIDConstants.armID, MotorType.kBrushless);
    armController = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();
    configMotor(armMotor, armEncoder, armController, true);

    Shuffleboard.getTab("ArmSubsystem").add(this).withSize(2, 1);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ArmConstants.armContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ArmConstants.armIdleMode);
    encoder.setVelocityConversionFactor(Constants.ArmConstants.armConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ArmConstants.armConversionPositionFactor);
    controller.setP(Constants.ArmConstants.armKP);
    controller.setI(Constants.ArmConstants.armKI);
    controller.setD(Constants.ArmConstants.armKD);
    controller.setFF(Constants.ArmConstants.armKFF);
    motor.enableVoltageCompensation(Constants.ArmConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public double getPosition() {
    if (RobotBase.isReal())
      return armEncoder.getPosition();
    else {
      simPosition += armEncoder.getVelocity() / 50;
      return simPosition;
    }
  }

  public double getVelocity() {
    return armEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command positionArmCommand(double degrees) {
    return new DoNothing();
  }
}
