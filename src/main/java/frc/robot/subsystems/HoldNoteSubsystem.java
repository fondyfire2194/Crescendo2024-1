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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Pref;

public class HoldNoteSubsystem extends SubsystemBase {

  CANSparkMax holdnoteMotor;
  SparkPIDController holdnoteController;
  RelativeEncoder holdnoteEncoder;

  /** Creates a new HoldNote. */
  public HoldNoteSubsystem() {
    holdnoteMotor = new CANSparkMax(Constants.CANIDConstants.holdNoteID, MotorType.kBrushless);
    holdnoteController = holdnoteMotor.getPIDController();
    holdnoteEncoder = holdnoteMotor.getEncoder();
    configMotor(holdnoteMotor, holdnoteEncoder, holdnoteController, true);
    if (RobotBase.isSimulation())
      REVPhysicsSim.getInstance().addSparkMax(holdnoteMotor, 3, 5600);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder, SparkPIDController controller,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.HoldNoteConstants.holdnoteContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.HoldNoteConstants.holdnoteIdleMode);
    encoder.setVelocityConversionFactor(Constants.HoldNoteConstants.holdnoteConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.HoldNoteConstants.holdnoteConversionPositionFactor);
    controller.setP(Constants.HoldNoteConstants.holdnoteKP);
    controller.setI(Constants.HoldNoteConstants.holdnoteKI);
    controller.setD(Constants.HoldNoteConstants.holdnoteKD);
    controller.setFF(Constants.HoldNoteConstants.holdnoteKFF);
    motor.enableVoltageCompensation(Constants.HoldNoteConstants.voltageComp);
    motor.burnFlash();
    encoder.setPosition(0.0);

    Shuffleboard.getTab("IntakeSubsystem").add(this)
        .withSize(2, 1)
        .withPosition(4, 0);

    Shuffleboard.getTab("IntakeSubsystem").addNumber("HoldNoteRPM", () -> getRPM())
        .withSize(2, 1)
        .withPosition(4, 1);

    Shuffleboard.getTab("IntakeSubsystem").add("StartHoldNote", runHoldNoteCommand())
        .withSize(2, 1)
        .withPosition(4, 2);

    Shuffleboard.getTab("IntakeSubsystem").add("StopHoldNote", stopHoldNoteCommand())
        .withSize(2, 1)
        .withPosition(4, 3);

  }

  public void stopMotor() {
    holdnoteController.setReference(0, ControlType.kVelocity);
    holdnoteMotor.stopMotor();
  }

  public double getRPM() {
    return holdnoteEncoder.getVelocity();
  }

  public Command runHoldNoteCommand() {
    return this.runOnce(() -> holdnoteController.setReference(2500, ControlType.kVelocity));
  }

  public Command feedShooterCommand() {
    return this.runOnce(() -> holdnoteController.setReference(Pref.getPref("HoldNoteRPM"), ControlType.kVelocity));
  }

  public Command stopHoldNoteCommand() {
    return this.runOnce(() -> stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  public void jog(double speed) {
    holdnoteMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }
}
