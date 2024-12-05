// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.Constructors.CamConfig;
import frc.lib.Constructors.PivotMotorConfig;
import frc.lib.Constructors.SwerveModuleConfig;
import frc.lib.Constructors.CamConfig.CamGain;
import frc.robot.subsystems.Leds.LEDStrip.Pattern;
import frc.robot.subsystems.Leds.Patterns.BlinkPattern;
import frc.robot.subsystems.Leds.Patterns.BreathingPattern;
import frc.robot.subsystems.Leds.Patterns.SolidPattern;

public final class Constants {

  public static final class PhysicalConstants {
    public static final double g = 9.81;
  }

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final String driverControllerType = "PS5";

    public static final int operatorControllerPort = 1;
    public static final String operatorControllerType = "PS5";

    public static final int pow = 5;
    public static final double n = 0.066;

    public static final int invertOmega = 1;
  }

  public static final class PoseEstimatorConstants {
    public static final double ODOMETRY_CYCLE_TIME = 250.0;
  }

  public static final class AutonomousConstants {
    public static final String DEFAULT_AUTO = "4 Note";
  }

  public static final class SubwooferPositions {

    public static final Map<Positions, Pose2d> Blue = Map.of(
        Positions.CENTER,
        new Pose2d(1.4707398414611816, 5.518360614776611, new Rotation2d(Units.degreesToRadians(180))),
        Positions.LEFT, new Pose2d(0.7096580862998962, 6.733922958374023, new Rotation2d(-2.0967819327074633)),
        Positions.RIGHT, new Pose2d(0.7007207870483398, 4.35414981842041, new Rotation2d(2.1055839986079654)));

    public static final Map<Positions, Pose2d> Red = Map.of(
        Positions.CENTER, new Pose2d(15.159453392028825, 5.543461322784424, new Rotation2d()),
        Positions.LEFT, new Pose2d(15.892428398132324, 4.328725814819336, new Rotation2d(1.0547094351486783)),
        Positions.RIGHT, new Pose2d(15.877486228942873, 6.759148120880127, new Rotation2d(-1.0547094351486783)));

    public enum Positions {
      CENTER,
      LEFT,
      RIGHT
    }
  }

  public static final class ClimbingPositions {
    public static final Map<Positions, Pose2d> Blue = Map.of(

        Positions.LEFT_300,
        new Pose2d(4.831638813018799, 5.479323863983154, new Rotation2d(Units.degreesToRadians(300))),

        Positions.LEFT_180,
        new Pose2d(6.06581449508667, 3.3881855964660645, new Rotation2d(Units.degreesToRadians(180))),

        Positions.LEFT_60,
        new Pose2d(3.5984339714050293, 3.479323863983154, new Rotation2d(Units.degreesToRadians(60))));

    public static final Map<Positions, Pose2d> Red = Map.of(

        Positions.LEFT_300, new Pose2d(11.7291789055, 2.76874685287, new Rotation2d(Units.degreesToRadians(120))),

        Positions.LEFT_180, new Pose2d(10.4950032234, 4.85988512039, new Rotation2d()),

        Positions.LEFT_60, new Pose2d(12.9623837471, 4.76874685287, new Rotation2d(Units.degreesToRadians(240))));

    public enum Positions {
      LEFT_180,
      LEFT_300,
      LEFT_60
    }
  }

  public static final class SpeakerConstants {
    public static final Pose3d Blue = new Pose3d(new Translation3d(-0.3050912916660309, 5.647868, 0), new Rotation3d());

    public static final Pose3d Red = new Pose3d(new Translation3d(16.84684181213379, 5.647868, 0), new Rotation3d());

    public static final double TOLERANCE = 0.1;
  }

  public static final class VisionConstants {

    public class Note {
      public static final double THICKNESS = Units.inchesToMeters(2.0);
      public static final double OUTER_DIAMETER = Units.inchesToMeters(14.0);
    }

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
        .loadAprilTagLayoutField();

    public static final CamConfig FRONTCAM = new CamConfig(
        "front",
        new Transform3d(new Translation3d(0.252061327, -0.1330425, 0.0),
            new Rotation3d(0, 121 - 90, 0)),
        1280, 800,
        new CamGain(0.075, 0, 1, 0),
        new CamGain(Units.degreesToRadians(5), 0, 0, 0));

    public static final CamConfig BACKCAM = new CamConfig(
        "back",
        new Transform3d(new Translation3d(), new Rotation3d()),
        320, 240,
        new CamGain(0, 0, 0, 0),
        new CamGain(0, 0, 0, 0));

    public static final CamConfig INTAKECAM = new CamConfig(
        "intake",
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d()),
        1280, 800);

    public static final double FRONTCAM_CYCLE_TIME = 10;
    public static final double BACKCAM_CYCLE_TIME = 10;

    public static final double ResetStDevThreshhold = 0.25;

    public static final double kPY = 5;
    public static final double kPTheta = 5;
  }

  public class DrivebaseConstants {

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DrivebaseConstants.swerveWidth / 2, DrivebaseConstants.swerveLength / 2),
        new Translation2d(DrivebaseConstants.swerveWidth / 2, -DrivebaseConstants.swerveLength / 2),
        new Translation2d(-DrivebaseConstants.swerveWidth / 2, DrivebaseConstants.swerveLength / 2),
        new Translation2d(-DrivebaseConstants.swerveWidth / 2, -DrivebaseConstants.swerveLength / 2));

    public class SwerveModuleConstants {

      public static final int DRIVE_CURRENT_LIMIT = 70;
      public static final int STEER_CURRENT_LIMIT = 30;

      public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
      public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

      public static final double DRIVE_GEAR_RATIO = 5.9;
      public static final double STEER_GEAR_RATIO = 150.0 / 7;

      public static final double DRIVE_RATIO = DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;

      public static TalonFXConfiguration TalonFXConfigDrive() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.001;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        config.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
      }

      public static TalonFXConfiguration TalonFXConfigSteer() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 4.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
      }

      public static CANcoderConfiguration canCoderConfig(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.MagnetOffset = angleOffset;

        return config;
      }

      public static final SwerveModuleConfig FL = new SwerveModuleConfig(
          Ports.FL_DRIVE,
          Ports.FL_STEER,
          Ports.FL_CANCODER,
          0.12548828125);

      public static final SwerveModuleConfig FR = new SwerveModuleConfig(
          Ports.FR_DRIVE,
          Ports.FR_STEER,
          Ports.FR_CANCODER,
          0.486083984375);

      public static final SwerveModuleConfig BL = new SwerveModuleConfig(
          Ports.BL_DRIVE,
          Ports.BL_STEER,
          Ports.BL_CANCODER,
          0.612060546875);

      public static final SwerveModuleConfig BR = new SwerveModuleConfig(
          Ports.BR_DRIVE,
          Ports.BR_STEER,
          Ports.BR_CANCODER,
          0.137939453125);
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        DrivebaseConstants.maxWheelVelocity, // m/s
        DrivebaseConstants.swerveRadius, // Distance from robot center to furthest module.
        new ReplanningConfig());

    public static final double maxRobotVelocity = 4.85;
    public static final double maxRobotOmega = 9;

    public static final double collisionAcc = 30.0;

    public static final double maxAcc = 100;
    public static final double maxAngularAcc = 100;

    public static final double maxWheelVelocity = 4.9;

    public static final double swerveWidth = 0.52;
    public static final double swerveLength = 0.52;
    public static final double swerveRadius = Math.hypot(swerveWidth / 2, swerveLength / 2);

    public static final double skew_dt = 0.1;

    public static final double CYCLE_TIME = 0.02;

    public static final long collision_sleep = 500;

    public static final PathConstraints pathConstraints = new PathConstraints(
        maxRobotVelocity, maxAcc, Units.radiansToRotations(maxRobotOmega), 5);
  }

  public static final class TesterConstants {
    public static TalonFXConfiguration talonFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.Slot0.kP = 0.11;
      config.Slot0.kI = 0.001;
      config.Slot0.kD = 0.0;
      config.Slot0.kV = 0.12;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      return config;
    }
  }

  public static final class IntakeConstants {
    public static final double INTAKE_VOLTAGE = 6;
    public static final double EJECT_VOLTAGE = -6;

    public static TalonFXConfiguration TalonFXConfigIntake() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      return config;
    }
  }

  public static final class FeederConstants {
    public static final double FEED_VOLTAGE = 6;
    public static final double SLOW_VOLTAGE = 2;
    public static final double INTAKE_VOLTAGE = 3;

    public static final double EJECT_VOLTAGE = -4;

    public static final double NOTE_STUCK_TIME = 3;
  }

  public static final class ShooterConstants {
    public static final double SHOOTER_GEAR_RATIO = 0.75;
    public static final double SHOOTER_RATIO = SHOOTER_GEAR_RATIO / Units.inchesToMeters(3);

    public static final double SHOOT_MS = 5;
    public static final double TOLERANCE_MS = 0.3;

    public static final double EJECT_MS = 1;

    public static final double SUBWOOFER_SHOOT_VOLTAGE = -6;
    public static final double EJECT_VOLTAGE = 6;

    public static final int SLOT = 0;

    public static TalonFXConfiguration TalonFXConfigLowerShooter() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.Slot0.kP = 0.2;
      config.Slot0.kI = 0.0;
      config.Slot0.kD = 0.0;
      config.Slot0.kV = 0.12;

      return config;
    }

    public static TalonFXConfiguration TalonFXConfigUpperShooter() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.Slot0.kP = 0.2;
      config.Slot0.kI = 0.0;
      config.Slot0.kD = 0.0;
      config.Slot0.kV = 0.12;

      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      return config;
    }
  }

  public static final class PivotConstants {

    public static final double PIVOT_GEAR_RATIO = 3 * 5 * (62/18) * (58/16);

    public static final double SUBWOOFER_ANGLE = Units.degreesToRotations(60);

    public static final double AMP_ANGLE = Units.degreesToRotations(100);

    public static final double EJECT_ANGLE = Units.degreesToRotations(30);

    public static final double TOLERANCE_RAD = Units.degreesToRotations(5);
    public static final double HOME_TOLERANCE = Units.degreesToRotations(20);

    public static final double maxVelocity = Units.degreesToRotations(90);
    public static final double maxAcc = Units.degreesToRotations(100);

    public static final PivotMotorConfig LeftMotorConfig = new PivotMotorConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final PivotMotorConfig RightMotorConfig = new PivotMotorConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  }

  public static final class ClimbConstants {
    public static final double CLIMB_VOLTAGE = 8;
    public static final double CLIMB_POSITION_EXTENDED = 10;
  }

  public static final class LedConstants {
    public static final int STRIP_LENGTH = 60;

    public static final Pattern Disabled = new BreathingPattern(STRIP_LENGTH, new Color(), new Color(0, 0, 255), 5);

    public static final Pattern NoNote = new SolidPattern(STRIP_LENGTH, new Color(255, 0, 0));
    public static final Pattern NoteInIntake = new BlinkPattern(STRIP_LENGTH, new Color(0, 255, 0), 0.25);
    public static final Pattern NoteInRobot = new SolidPattern(STRIP_LENGTH, new Color(0, 255, 0));
    public static final Pattern NoteStuck = new BlinkPattern(STRIP_LENGTH, new Color(0, 0, 255), 0.25);
  }
}
