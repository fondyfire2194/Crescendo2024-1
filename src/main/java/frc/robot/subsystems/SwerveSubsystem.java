package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.commands.CenterStart.CenterStartCommand1Paths;
import frc.robot.Pref;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase {
  // The gyro sensor

  private final AHRS gyro;

  private SwerveDrivePoseEstimator swervePoseEstimator;

  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private Pose2d simOdometryPose = new Pose2d();

  private final TimeOfFlight m_rearLeftSensor = new TimeOfFlight(CANIDConstants.rearLeftSensor);

  private final TimeOfFlight m_rearRightSensor = new TimeOfFlight(CANIDConstants.rearRightSensor);

  private boolean allowVisionCorrection;

  private boolean lookForNote;

  public SwerveSubsystem() {

    if (RobotBase.isSimulation()) {

      // thetaPID.setP(0);

      // xPID.setP(1.0);

      // yPID.setP(0);

    }

    gyro = new AHRS(SPI.Port.kMXP, (byte) 100);

    // Configure time of flight sensor for short ranging mode and sample
    // distance every 40 ms
    m_rearLeftSensor.setRangingMode(RangingMode.Short, 40);

    m_rearRightSensor.setRangingMode(RangingMode.Short, 40);

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    swervePoseEstimator = new SwerveDrivePoseEstimator(
        Constants.SwerveConstants.swerveKinematics,
        getYaw(),
        getPositions(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    simOdometryPose = swervePoseEstimator.getEstimatedPosition();

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    if (Robot.isReal())
      resetModuleEncoders();
    else
      resetAngleEncoders();

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPoseEstimator,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.SwerveConstants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    zeroGyro();
    resetPoseEstimator(new Pose2d());

    Shuffleboard.getTab("Drivetrain").add(this)
        .withSize(2, 1).withPosition(0, 0);

    Shuffleboard.getTab("Drivetrain").add("SetDriveKp", setDriveKp())
        .withSize(1, 1).withPosition(5, 0);

    Shuffleboard.getTab("Drivetrain").addNumber("DriveKp", () -> getDriveKp())
        .withSize(1, 1).withPosition(5, 1);

    Shuffleboard.getTab("Drivetrain").addNumber("DriveKpSet1", () -> Pref.getPref("DriveKp"))
        .withSize(1, 1).withPosition(5, 2);

    Shuffleboard.getTab("Drivetrain").add("SetDriveFF", setDriveFF())
        .withSize(1, 1).withPosition(6, 0);

    Shuffleboard.getTab("Drivetrain").addNumber("DriveFF%",
        () -> getDriveFF() * Constants.SwerveConstants.kmaxTheoreticalSpeed)
        .withSize(1, 1).withPosition(6, 1);

    Shuffleboard.getTab("Drivetrain").addNumber("DriveFFSet",
        () -> Pref.getPref("DriveFF"))
        .withSize(1, 1).withPosition(6, 2);

    Shuffleboard.getTab("Drivetrain").add("SetAngleKp", setAngleKp())
        .withSize(1, 1).withPosition(7, 0);

    Shuffleboard.getTab("Drivetrain").addNumber("AngleKp", () -> getAngleKp())
        .withSize(1, 1).withPosition(7, 1);

    Shuffleboard.getTab("Drivetrain").addNumber("AngleKpSet", () -> Pref.getPref("AngleKp"))
        .withSize(1, 1).withPosition(7, 2);

    Shuffleboard.getTab("Drivetrain").add("ResetPose", this.setPoseToX0Y0())
        .withSize(1, 1).withPosition(2, 0);

    Shuffleboard.getTab("Drivetrain").add("PathfindPickup",

        AutoBuilder.pathfindToPose(
            new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(0)),
            new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            2.0))
        .withSize(1, 1).withPosition(0, 1);

    Shuffleboard.getTab("Drivetrain").add("PathfindScore",

        AutoBuilder.pathfindToPose(
            new Pose2d(1.15, 1.0, Rotation2d.fromDegrees(180)),
            new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            0))
        .withSize(1, 1).withPosition(1, 1);

    Shuffleboard.getTab("Drivetrain").add("On-the-fly path", Commands.runOnce(() -> {

      Pose2d currentPose = this.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation()
          .plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              3, 3,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }))
        .withSize(1, 1).withPosition(2, 1);


    setModuleDriveFF();
    setModuleDriveKp();
    setModuleAngleKp();

  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void drive(double translation, double strafe, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(

        fieldRelative
            ? fieldRelativeByAlliance(translation, strafe, rotation)
            : new ChassisSpeeds(translation, strafe, rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.kmaxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }

  }

  ChassisSpeeds fieldRelativeByAlliance(double translation, double strafe, double rotation) {
    Rotation2d temp = getYaw();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
      temp = getYaw().plus(new Rotation2d(Math.PI));
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        translation, strafe, rotation, temp);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.kmaxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetModuleEncoders() {
    mSwerveMods[0].resetAngleToAbsolute();
    mSwerveMods[1].resetAngleToAbsolute();
    mSwerveMods[2].resetAngleToAbsolute();
    mSwerveMods[3].resetAngleToAbsolute();
  }

  public void resetAngleEncoders() {
    mSwerveMods[0].resetAngleEncoder(0);
    mSwerveMods[1].resetAngleEncoder(180);
    mSwerveMods[2].resetAngleEncoder(0);
    mSwerveMods[3].resetAngleEncoder(180);

  }

 
  public double getHeadingDegrees() {
    return Math.IEEEremainder((gyro.getAngle()), 360);
  }

  public Pose2d getPose() {
    if (RobotBase.isReal())
      return swervePoseEstimator.getEstimatedPosition();
    else
      return simOdometryPose;
  }

  public double getX() {
    return getPose().getX();
  }

  public double getY() {
    return getPose().getY();
  }

  public void setModuleDriveKp() {
    mSwerveMods[0].setDriveKp();
    mSwerveMods[1].setDriveKp();
    mSwerveMods[2].setDriveKp();
    mSwerveMods[3].setDriveKp();
  }

  public void setModuleDriveFF() {
    mSwerveMods[0].setDriveFF();
    mSwerveMods[1].setDriveFF();
    mSwerveMods[2].setDriveFF();
    mSwerveMods[3].setDriveFF();
  }

  public void setModuleAngleKp() {
    mSwerveMods[0].setAngleKp();
    mSwerveMods[1].setAngleKp();
    mSwerveMods[2].setAngleKp();
    mSwerveMods[3].setAngleKp();
  }

  public double getDriveKp() {
    return mSwerveMods[0].getDriveKp();
  }

  public double getDriveFF() {
    return mSwerveMods[0].getDriveFF();
  }

  public Command setDriveKp() {
    return Commands.runOnce(() -> setModuleDriveKp());
  }

  public Command setDriveFF() {
    return Commands.runOnce(() -> setModuleDriveFF());
  }

  public Command setAngleKp() {
    return Commands.runOnce(() -> setModuleAngleKp());
  }

  public double getAngleKp() {
    return mSwerveMods[0].getAngleKp();
  }

  public boolean driveIsBraked() {
    return mSwerveMods[0].driveIsBraked();
  }

  public boolean getIsRotating(){
    return gyro.isRotating();
  }

  public void setIdleMode(boolean brake) {
    mSwerveMods[0].setIdleMode(brake);
    mSwerveMods[1].setIdleMode(brake);
    mSwerveMods[2].setIdleMode(brake);
    mSwerveMods[3].setIdleMode(brake);
  }

  public double getPoseHeading() {
    return getPose().getRotation().getDegrees();
  }

  public void resetPoseEstimator(Pose2d pose) {
    zeroGyro();
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
    simOdometryPose = pose;
  }

  public Command setPose(Pose2d pose) {
    return Commands.runOnce(() -> resetPoseEstimator(pose));
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swervePoseEstimator;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.reset();

  }

  public Rotation2d getYaw() {
    if (RobotBase.isReal())
      return gyro.getRotation2d();
    else
      return simOdometryPose.getRotation();
  }

  public float getPitch() {
    return gyro.getPitch();
  }

  public float getRoll() {
    return gyro.getRoll();
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    if (allowVisionCorrection)
      swervePoseEstimator.addVisionMeasurement(pose, timestamp);
  }

  public double getRearLeftSensorMM() {
    return m_rearLeftSensor.getRange();
  }

  public double getRearRightSensorMM() {
    return m_rearRightSensor.getRange();
  }

  public double getRearLeftSensorStdDevMM() {
    return m_rearLeftSensor.getRangeSigma();
  }

  public double getRearRightSensorStdDevMM() {
    return m_rearRightSensor.getRangeSigma();
  }

  public void setLookForNote(){
    lookForNote=true;
  }


  public void resetLookForNote(){
    lookForNote=false;
  }

  public boolean getLookForNote(){
    return lookForNote;
  }

  

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("LeftInches",
    // round2dp(Units.metersToInches(getRearLeftSensorMM() / 1000), 1));
    // SmartDashboard.putNumber("RightInches",
    // round2dp(Units.metersToInches(getRearRightSensorMM() / 1000), 1));
    // SmartDashboard.putBoolean("Note Seen", getRearRightSensorMM() < 50);
    swervePoseEstimator.update(getYaw(), getPositions());
    // swervePoseEstimator.addVisionMeasurement(previousposeleft,
    // timestampsecondsl);
    // swervePoseEstimator.addVisionMeasurement(previousposeright,
    // timestampsecondsr);

    field.setRobotPose(getPose());
    SmartDashboard.putNumber("X Meters", round2dp(getX(), 2));
    SmartDashboard.putNumber("Y Meters", round2dp(getY(), 2));
    SmartDashboard.putNumber("Est Pose Heaading", round2dp(getPoseHeading(), 2));

    SmartDashboard.putNumber("GyroYaw", round2dp(getYaw().getDegrees(), 2));
    SmartDashboard.putNumberArray("Odometry",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });

    putStates();

  }

  private void resetAll() {
    // gyro.reset();
    resetModuleEncoders();
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), new Pose2d());
    simOdometryPose = new Pose2d();

  }

  public Command setPoseToX0Y0() {
    return Commands.runOnce(() -> resetAll());
  }

  @Override
  public void simulationPeriodic() {

    SwerveModuleState[] measuredStates

        = new SwerveModuleState[] {
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        };

    ChassisSpeeds speeds = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(measuredStates);
    SmartDashboard.putNumber("VX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("VTH", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("VOM", speeds.omegaRadiansPerSecond);

    simOdometryPose = simOdometryPose.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * .02,
            speeds.vyMetersPerSecond * .02,
            speeds.omegaRadiansPerSecond * .02));

  }

  private void putStates() {

    double[] realStates = {
        mSwerveMods[0].getState().angle.getDegrees(),
        mSwerveMods[0].getState().speedMetersPerSecond,
        mSwerveMods[1].getState().angle.getDegrees(),
        mSwerveMods[1].getState().speedMetersPerSecond,
        mSwerveMods[2].getState().angle.getDegrees(),
        mSwerveMods[2].getState().speedMetersPerSecond,
        mSwerveMods[3].getState().angle.getDegrees(),
        mSwerveMods[3].getState().speedMetersPerSecond
    };

    double[] theoreticalStates = {
        mSwerveMods[0].getDesiredState().angle.getDegrees(),
        mSwerveMods[0].getDesiredState().speedMetersPerSecond,
        mSwerveMods[1].getDesiredState().angle.getDegrees(),
        mSwerveMods[1].getDesiredState().speedMetersPerSecond,
        mSwerveMods[2].getDesiredState().angle.getDegrees(),
        mSwerveMods[2].getDesiredState().speedMetersPerSecond,
        mSwerveMods[3].getDesiredState().angle.getDegrees(),
        mSwerveMods[3].getDesiredState().speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
    SmartDashboard.putNumberArray("Real States", realStates);
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public boolean isStopped() {
    return mSwerveMods[0].isStopped()
        && mSwerveMods[1].isStopped()
        && mSwerveMods[2].isStopped()
        && mSwerveMods[3].isStopped();
  }

}
