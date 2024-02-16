package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.config.SwerveModuleConstants;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

public final class Constants {

        public static final class CANIDConstants {
                // can ids 4 through 15 aee]used for swerve modules see SwerveConstants
                public static final int leftShooterID = 16;
                public static final int rightShooterID = 17;

                public static final int intakeID = 18;
                public static final int elevatorID = 19;

                public static final int rearLeftSensor = 20;
                public static final int rearRightSensor = 21;

                public static final int noteSensor = 23;

                public static final int holdNoteID = 22;

                public static final int shooterangleID = 24;

                public static final int feedShooterID = 25;

                public static final int intakeDistanceSensorID = 26;

        }

        public static final class SwerveConstants {

                public static final double stickDeadband = 0.05;

                public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

                /* Drivetrain Constants */
                public static final Measure<Distance> trackWidth = Meters.of(Meters.convertFrom(22.125, Inches));
                public static final Measure<Distance> wheelBase = Meters.of(Meters.convertFrom(27.25, Inches));
                public static final Measure<Distance> wheelDiameter = Meters.of(Meters.convertFrom(4.0, Inches));
                public static final Measure<Distance> wheelCircumference = Meters
                                .of(wheelDiameter.magnitude() * Math.PI);

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

                public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double driveGearRatio = mk4iL1DriveGearRatio;

                public static double angleGearRatio = mk4iL1TurnGearRatio;

                public static final Translation2d flModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d frModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);
                public static final Translation2d blModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d brModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 20;
                public static final int driveContinuousCurrentLimit = 30;

                /* Swerve Profiling Values */
                public static final double kmaxTheoreticalSpeed = 3.7;// mps
                public static final double kmaxSpeed = 3.25; // meters per second
                public static final double kmaxAngularVelocity = 1.0 * Math.PI;

                /* Angle Motor PID Values */
                public static final double angleKP = 0.01;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.0;
                public static final double angleKFF = 0.0;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.0;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = .5 / kmaxTheoreticalSpeed;

                /* Drive Motor Characterization Values */
                public static final double driveKS = 0.667;
                public static final double driveKV = 3.04;
                public static final double driveKA = 0.27;

                /* Drive Motor Conversion Factors */
                public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI)
                                / driveGearRatio;
                public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
                public static final double angleConversionFactor = 360.0 / angleGearRatio;

                /* Neutral Modes */
                public static final IdleMode angleNeutralMode = IdleMode.kBrake;
                public static final IdleMode driveNeutralMode = IdleMode.kBrake;

                /* Motor Inverts */
                public static final boolean driveInvert = false;
                public static final boolean angleInvert = true;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                public static String[] modNames = { "FL ", "FR ", "BL ", "BR " };

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 5;
                        public static final int cancoderID = 6;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 253
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 7;
                        public static final int angleMotorID = 8;
                        public static final int cancoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 108
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int cancoderID = 12;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 207
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 13;
                        public static final int angleMotorID = 14;
                        public static final int cancoderID = 15;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 239
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                                new PIDConstants(5.0, 0, 0), // Translation constants
                                new PIDConstants(5.0, 0, 0), // Rotation constants
                                kmaxSpeed,
                                flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                                new ReplanningConfig());

        }

        public static final class KeepAngle {
                public static final double kp = 0.30;
                public static final double ki = 0.0;
                public static final double kd = 0.0;
        }

        public static final class FieldConstants {
                public static final double FIELD_WIDTH = 8.21;
                public static final double FIELD_LENGTH = 16.54;

                public static final Pose2d blueNote1 = new Pose2d(2.89, 6.99, new Rotation2d());
                public static final Pose2d blueNote2 = new Pose2d(2.89, 5.54, new Rotation2d());
                public static final Pose2d blueNote3 = new Pose2d(2.89, 4.09, new Rotation2d());

                public static final Pose2d centerNote1 = new Pose2d(8.28, 7.45, new Rotation2d());
                public static final Pose2d centerNote2 = new Pose2d(8.28, 5.77, new Rotation2d());
                public static final Pose2d centerNote3 = new Pose2d(8.28, 4.10, new Rotation2d());
                public static final Pose2d centerNote4 = new Pose2d(8.28, 2.44, new Rotation2d());
                public static final Pose2d centerNote5 = new Pose2d(8.28, 0.75, new Rotation2d());

        }

        public static final class AprilTagConstants {
                public static AprilTagFieldLayout layout;
                static {
                        try {
                                layout = AprilTagFieldLayout
                                                .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                        } catch (Exception e) {
                                e.printStackTrace();
                        }
                }
        }

        public static final class GlobalConstants {
                public static final int ROBOT_LOOP_HZ = 50;
                /** Robot loop period */
                public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;
        }

        public static final class CameraConstants {

                public static class CameraValues {
                        public String camname = "name";
                        public String ipaddress = "ip";
                        public Transform3d transform = new Transform3d();
                        public boolean isUsed = false;

                        public CameraValues(String camname, String ipaddress, Transform3d transform, boolean isUsed) {
                                this.camname = camname;
                                this.ipaddress = ipaddress;
                                this.transform = transform;
                                this.isUsed = isUsed;
                        }
                }

                public static CameraValues frontLeftCamera = new CameraValues("limelight-frleft", "10.21.94.5",
                                new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
                                                new Rotation3d(0, .1, .2)),
                                true);

                public static CameraValues frontRightCamera = new CameraValues("limelight-frright", "10.21.94.6",
                                new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                                new Rotation3d(0, .1, .2)),
                                false);

                public static CameraValues rearCamera = new CameraValues("limelight-rear", "10.21.94.10",
                                new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                                new Rotation3d(0, .1, .2)),
                                true);

                public static final double POSE_AMBIGUITY_CUTOFF = 0.05;
                public static final double DISTANCE_CUTOFF = 4.0;

        }

        public static final class ShooterConstants {

                public static final double maxShooterMotorRPM = 5700;
                public static final double shooterConversionVelocityFactor = 1; // TODO change value
                public static final double shooterConversionPositionFactor = 1; // TODO change value
                public static final double shooterKP = .0;
                public static final double shooterKI = 0;
                public static final double shooterKD = 0;
                public static final double shooterKFF = .9 / maxShooterMotorRPM;
                public static final double voltageComp = 12;
                public static final IdleMode shooterIdleMode = IdleMode.kCoast;
                public static final int shooterContinuousCurrentLimit = 30;

                public static final double closeShootSpeed = 1000;
                public static final double dist1ShootSpeed = 1000;
                public static final double dist2ShootSpeed = 1000;
                public static final double dist3ShootSpeed = 1000;

        }

        public static final class ShooterFeederConstants {

                public static final double maxShooterFeederMotorRPM = 5700;
                public static final double shooterfeederConversionVelocityFactor = 1; // TODO change value
                public static final double shooterfeederConversionPositionFactor = 1; // TODO change value
                public static final double shooterfeederKP = .001;
                public static final double shooterfeederKI = 0;
                public static final double shooterfeederKD = 0;
                public static final double shooterfeederKFF = .9;
                public static final double voltageComp = 12;
                public static final IdleMode shooterfeederIdleMode = IdleMode.kCoast;
                public static final int shooterfeederContinuousCurrentLimit = 30;

                public static final double closeShootSpeed = 1000;
                public static final double dist1ShootSpeed = 1000;
                public static final double dist2ShootSpeed = 1000;
                public static final double dist3ShootSpeed = 1000;

        }

        public static final class IntakeConstants {

                public static final double maxIntakeMotorRPM = 5700;
                public static final double intakeConversionVelocityFactor = 1; // TODO change value
                public static final double intakeConversionPositionFactor = 1; // TODO change value
                public static final double intakeKP = .000001;
                public static final double intakeKI = 0;
                public static final double intakeKD = 0;
                public static final double intakeKFF = .5 / maxIntakeMotorRPM;
                public static final double voltageComp = 12;
                public static final IdleMode intakeIdleMode = IdleMode.kCoast;
                public static final int intakeContinuousCurrentLimit = 30;
                public static final double noteSensedInches = 4;

        }

        public static final class HoldNoteConstants {

                public static final double maxHoldNoteMotorRPM = 5700;
                public static final double holdnoteConversionVelocityFactor = 1; // TODO change value
                public static final double holdnoteConversionPositionFactor = 1; // TODO change value
                public static final double holdnoteKP = .000001;
                public static final double holdnoteKI = 0;
                public static final double holdnoteKD = 0;
                public static final double holdnoteKFF = .9;
                public static final double voltageComp = 12;
                public static final IdleMode holdnoteIdleMode = IdleMode.kCoast;
                public static final int holdnoteContinuousCurrentLimit = 30;
                public static final double noteSeenInches = 3;
                public static double intakeSpeed = 550;

        }

        public static final class ElevatorConstants {

                public static final double maxElevatorMotorRPM = 5700;
                public static final double elevatorConversionVelocityFactor = 1; // TODO change value
                public static final double elevatorConversionPositionFactor = 1; // TODO change value
                public static final double elevatorKP = .000001;
                public static final double elevatorKI = 0;
                public static final double elevatorKD = 0;
                public static final double elevatorKFF = 1 / maxElevatorMotorRPM;
                public static final double voltageComp = 12;
                public static final IdleMode elevatorIdleMode = IdleMode.kCoast;
                public static final int elevatorContinuousCurrentLimit = 30;

                public static double elevatorPositionToAmp = 14;
                public static double elevatorPositionToIntake = 2;
                public static double elevatorMinInches = 0;
                public static double elevatorMaxinches = 15;

        }

        public static final class ShooterAngleConstants {

                public static final double maxShooterAngleMotorRPM = 5700;
                public static final double shooterangleConversionVelocityFactor = 1; // TODO change value
                public static final double shooterangleConversionPositionFactor = 1; // TODO change value
                public static final double shooterangleKP = 5e-5;
                public static final double shooterangleKI = 0;
                public static final double shooterangleKD = 0;
                public static final double shooterangleKFF = .000156;
                public static final double shooterangleKIz = .05;
                public static final double shooteranglekMaxOutput = .5;
                public static final double shooteranglekMinOutput = -.5;

                public static double maxVelocity = 500;
                public static final double minVelocity = 0;
                public static final double maxAcceleration = 500;
                public static final double allowedError = 0;

                public static final double voltageComp = 12;
                public static final IdleMode shooterangleIdleMode = IdleMode.kCoast;
                public static final int shooterangleContinuousCurrentLimit = 30;
                public static double shooterangleMinDegrees = 1;
                public static double shooterangleMaxDegrees = 60;
                public static double shooterangleMidDegrees = 15;

        }

        public static final double[][] speedAngleValues

                        = { { 1, 2, 3, 4, 5, 6, 7 }, { 3, 4, 5, 6, 7, 8, 9 } };
}
