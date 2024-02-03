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
import frc.robot.Constants.ShooterConstants;

public class BottomShooterRollerSubsystem extends SubsystemBase {

  CANSparkMax bottomRoller;

  SparkPIDController bottomController;

  RelativeEncoder bottomEncoder;

  double bottomRollerCommandRPM = 500;
  double commandRPM = 500;

  /** Creates a new Shooter. */
  public BottomShooterRollerSubsystem() {
    bottomRoller = new CANSparkMax(Constants.CANIDConstants.bottomShooterID, MotorType.kBrushless);

    bottomController = bottomRoller.getPIDController();

    bottomEncoder = bottomRoller.getEncoder();
    configMotor(bottomRoller, bottomEncoder, bottomController, true);

    Shuffleboard.getTab("ShooterSubsystem").add(this)
        .withSize(3, 1)
        .withPosition(3, 0);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("BottomRPM", () -> getRPMBottom())
        .withSize(1, 1)
        .withPosition(3, 1);

    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(bottomRoller, 3, 5600);
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
      bottomController.setReference(commandRPM, ControlType.kVelocity);
    else
      bottomRoller.setVoltage(commandRPM * 12 / ShooterConstants.maxShooterMotorRPM);

  }

  public void stopMotors() {
    if (RobotBase.isReal())
      bottomController.setReference(0, ControlType.kVelocity);
    bottomRoller.stopMotor();
  }

  public Command runBottomRollerCommand() {

    return this.runOnce(() -> runRoller());

  }

  public Command stopShootersCommand() {
    return this.runOnce(() -> stopMotors());
  }

  public double getRPMBottom() {
    return bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("CANTopRoller", topRoller.getFirmwareVersion());
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
