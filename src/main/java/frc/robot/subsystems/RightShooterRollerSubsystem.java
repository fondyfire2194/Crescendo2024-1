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
        .withSize(2, 1)
        .withPosition(2, 0);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("RightRPM", () -> getRPMRight())
        .withSize(1, 1)
        .withPosition(2, 1);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("CANRightRoller", rightRoller.getFirmwareVersion());
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
