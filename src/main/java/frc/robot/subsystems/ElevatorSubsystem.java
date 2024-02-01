// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  CANSparkMax elevatorMotor;
  SparkPIDController elevatorController;
  RelativeEncoder elevatorEncoder;
  private double simPosition;
  private double commandInches;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {

    elevatorMotor = new CANSparkMax(Constants.CANIDConstants.elevatorID, MotorType.kBrushless);
    elevatorController = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();
    configMotor(elevatorMotor, elevatorEncoder, elevatorController, true);

    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(elevatorMotor, 3, 5600);

    Shuffleboard.getTab("IntakeSubsystem").add(this)
        .withSize(2, 1)
        .withPosition(2, 0);

    Shuffleboard.getTab("IntakeSubsystem").addNumber("ElevtorPositionInches", () -> getPosition())
        .withSize(2, 1)
        .withPosition(2, 1);

    Shuffleboard.getTab("IntakeSubsystem").add("ElevatorToIntake", positionToIntakeCommand())
        .withSize(2, 1)
        .withPosition(2, 2);

    Shuffleboard.getTab("IntakeSubsystem").add("ElaevatorToAmp", positionToAmpCommand())
        .withSize(2, 1)
        .withPosition(2, 3);

    if (RobotBase.isSimulation()) {

      elevatorEncoder.setVelocityConversionFactor(.001 / 60);
      elevatorEncoder.setPositionConversionFactor(.001);
    }

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
      simPosition += elevatorEncoder.getVelocity() / 50000;
      return simPosition;
    }
  }

  public void stopMotor() {
    elevatorMotor.stopMotor();
    elevatorMotor.setVoltage(0);
  }

  public void jog(double speed) {
    elevatorMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public Command jogCommand(Double speed) {
    return Commands.run(() -> jog(speed));
  }

  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  public void setVelocity(double speed) {
    elevatorMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void positionElevator(double inches) {
    commandInches = inches;
    if (RobotBase.isReal())
      elevatorController.setReference(inches, ControlType.kPosition);
    else
      setVelocity(inches - getPosition());
  }

  public Command positionToIntakeCommand() {
    return Commands.run(() -> positionElevator(Constants.ElevatorConstants.elevatorPositionToIntake))
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public Command positionToAmpCommand() {
    return Commands.run(() -> positionElevator(Constants.ElevatorConstants.elevatorPositionToAmp))
        .until(() -> Math.abs(getPositionError()) < .5);
  }

  public double getPositionError() {
    return commandInches - getPosition();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
