package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Pref;

public class SwerveModule extends SubsystemBase {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;

  private final CANcoder m_turnCancoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private SwerveModuleState currentDesiredState = new SwerveModuleState();
  private double angleDegrees;
  private double m_simDrivePosition;
  private double m_simRotatePosition;
  private boolean driveReversed;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    driveReversed = moduleConstants.driveReversed;
    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();
    this.m_simDrivePosition = 0.0;
    this.m_simRotatePosition = 0.0;
    m_turnCancoder = new CANcoder(moduleConstants.canCoderID, "CV1");

    // var can_config = new CANcoderConfiguration();
    // can_config.MagnetSensor.AbsoluteSensorRange =
    // AbsoluteSensorRangeValue.Unsigned_0To1;
    // m_turnCancoder.getConfigurator().apply(can_config);

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    // try this for faster updates from CAN check can usage befor and after
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);
    angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    currentDesiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public SwerveModuleState getDesiredState() {
    return currentDesiredState;
  }

  public void resetAngleToAbsolute() {
    double angle = ((m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360)); // -angleOffset
    integratedAngleEncoder.setPosition(angle);
  }

  public void resetAngleEncoder(double angle) {
    integratedAngleEncoder.setPosition(angle);
    this.m_simRotatePosition = 0.0;
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);

    // ******************************************************** */

    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);

    // ******************************************************** */

    angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    angleMotor.burnFlash();

  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(driveReversed);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.SwerveConstants.driveKP);
    driveController.setI(Constants.SwerveConstants.driveKI);
    driveController.setD(Constants.SwerveConstants.driveKD);
    driveController.setFF(Constants.SwerveConstants.driveKFF);// 3.24=max speed
    driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public void setDriveKp() {
    driveController.setP(Pref.getPref("DriveKp"));
  }

  public void setDriveFF() {
    driveController.setFF(Pref.getPref("DriveFF") / Constants.SwerveConstants.kmaxTheoreticalSpeed);
  }

  public double getDriveKp() {
    return driveController.getP();
  }

  public double getDriveFF() {
    return driveController.getFF();
  }

  public void setAngleKp() {
    angleController.setP(Pref.getPref("AngleKp"));
  }

  public double getAngleKp() {
    return angleController.getP();
  }

  public boolean driveIsBraked() {
    return driveMotor.getIdleMode() == IdleMode.kBrake;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.kmaxSpeed;
      driveMotor.setVoltage(percentOutput * 12);
    } else {
      driveController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }
    if (RobotBase.isSimulation())
      m_simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD;
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.kmaxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
    angleDegrees = angle.getDegrees();
    if (RobotBase.isSimulation())
      m_simRotatePosition = angle.getDegrees();
  }

  private Rotation2d getAngle() {
    if (RobotBase.isReal())
      return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    else
      return Rotation2d.fromDegrees(m_simRotatePosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
  }

  public double getDriveVelocity() {
    if (RobotBase.isReal())
      return driveEncoder.getVelocity();
    else {
      return currentDesiredState.speedMetersPerSecond;
    }
  }

  public double getDrivePosition() {
    if (RobotBase.isReal())
      return driveEncoder.getPosition();

    else {

      return m_simDrivePosition;

    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " DRIVEVEL",
    // getDriveVelocity());
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " SETVEL",
    // currentDesiredState.speedMetersPerSecond);
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " ACTPOS",
    // getDrivePosition());
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " CC",
    // m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360);

    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " ABS POS",
    // m_turnCancoder.getAbsolutePosition().getValueAsDouble());

    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " setptangdeg",
    // angleDegrees);
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " actualangdeg",
    // getAngle().getDegrees());
  }

  public boolean isStopped() {
    return Math.abs(getDriveVelocity()) < .1;
  }

  public void setIdleMode(boolean brake) {
    if (brake) {
      driveMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setIdleMode(IdleMode.kBrake);
    } else {
      driveMotor.setIdleMode(IdleMode.kCoast);
      angleMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void simulationPeriodic() {

  }

  public static double round2dp(double number, int dp) {
    number = Math.round(number * dp);
    number /= 100;
    return number;
  }

}
