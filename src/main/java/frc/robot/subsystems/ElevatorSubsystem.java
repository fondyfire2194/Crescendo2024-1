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

public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax elevatorMotor;
  SparkPIDController elevatorController;
  RelativeEncoder elevatorEncoder;
  private double simPosition;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {

    elevatorMotor = new CANSparkMax(Constants.CANIDConstants.elevatorID, MotorType.kBrushless);
    elevatorController = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();
    configMotor(elevatorMotor, elevatorEncoder, elevatorController, true);

    Shuffleboard.getTab("ElevatorSubsystem").add(this).withSize(2, 1);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ElevatorConstants.elevatorContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ElevatorConstants.elevatorIdleMode);
    encoder.setVelocityConversionFactor(Constants.ElevatorConstants.elevatorConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ElevatorConstants.elevatorConversionPositionFactor);
    controller.setP(Constants.ElevatorConstants.elevatorKP);
    controller.setI(Constants.ElevatorConstants.elevatorKI);
    controller.setD(Constants.ElevatorConstants.elevatorKD);
    controller.setFF(Constants.ElevatorConstants.elevatorKFF);
    motor.enableVoltageCompensation(Constants.ElevatorConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public double getPosition() {
    if (RobotBase.isReal())
      return elevatorEncoder.getPosition();
    else {
      simPosition += elevatorEncoder.getVelocity() / 50;
      return simPosition;
    }
  }

  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command positionElevatorCommand(double degrees) {
    return new DoNothing();
  }
}
