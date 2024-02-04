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
import frc.robot.Constants.ShooterFeederConstants;

public class FeedShooterSubsystem extends SubsystemBase {

  CANSparkMax feedBelts;

  SparkPIDController feedController;
  RelativeEncoder feedEncoder;

  double feedBeltsCommandRPM = 500;

  double commandRPM = 500;

  /** Creates a new Shooter feeder. */
  public FeedShooterSubsystem() {
    feedBelts = new CANSparkMax(Constants.CANIDConstants.feedShooterID, MotorType.kBrushless);
    feedController = feedBelts.getPIDController();
    feedEncoder = feedBelts.getEncoder();

    configMotor(feedBelts, feedEncoder, feedController, false);

    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(feedBelts, 3, 5600);

    Shuffleboard.getTab("ShooterSubsystem").add(this)
        .withSize(2, 1)
        .withPosition(4, 0);

    Shuffleboard.getTab("ShooterSubsystem").addNumber("FeederRPM", () -> getRPMBelts())
        .withSize(1, 1)
        .withPosition(4, 1);
  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller, boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ShooterFeederConstants.shooterfeederContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ShooterFeederConstants.shooterfeederIdleMode);
    encoder.setVelocityConversionFactor(Constants.ShooterFeederConstants.shooterfeederConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ShooterFeederConstants.shooterfeederConversionPositionFactor);
    controller.setP(Constants.ShooterFeederConstants.shooterfeederKP);
    controller.setI(Constants.ShooterFeederConstants.shooterfeederKI);
    controller.setD(Constants.ShooterFeederConstants.shooterfeederKD);
    controller.setFF(Constants.ShooterFeederConstants.shooterfeederKFF);
    motor.enableVoltageCompensation(Constants.ShooterFeederConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public Command setFeedShooterSpeed(double rpm) {
    return Commands.runOnce(() -> commandRPM = rpm);
  }

  public void runBelts() {
    if (RobotBase.isReal())
      feedController.setReference(commandRPM, ControlType.kVelocity);
    else
      feedBelts.setVoltage(commandRPM * 12 / ShooterFeederConstants.maxShooterFeederMotorRPM);

  }

  public void stopMotor() {
    if (RobotBase.isReal())
      feedController.setReference(0, ControlType.kVelocity);
    feedBelts.stopMotor();
  }

  public Command stopShooterCommand() {
    return this.runOnce(() -> stopMotor());
  }

  public Command runFeedBeltsCommand() {
    if (RobotBase.isReal())
      return this.runOnce(() -> runBelts());
    else
      return Commands.runOnce(() -> feedBelts.setVoltage(.5));
  }

  public double getRPMBelts() {
    return feedEncoder.getVelocity();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

}
