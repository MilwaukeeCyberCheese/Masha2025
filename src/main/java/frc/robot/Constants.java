// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.ChuteSubsystem.ChuteState;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.CoralHandlerSubsystem.CoralHandlerState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.utils.PIDConstants;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kTau = Math.PI * 2;

  public static class Sensors {}

  public static final class IOConstants {
    public static final int kOperatorControllerPort = 0;
    public static final int kDriverControllerPort = 1;
    public static final int kButtonBoardPort = 2;
    public static final double kDriveDeadband = 0.05;

    // When test mode is enabled, the operator controller is used for driving and testing
    // This should always be false on the main branch
    public static final boolean kTestMode = false;
  }

  public static final class Vision {

    public static final class HandlerCamera {
      public static final String kCameraName = "handlerCamera";
    }

    public static final class LeftCamera {
      public static final String kCameraName = "leftCamera";

      // z is good
      public static final Transform3d kRobotToCamera =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-6.94),
                  Units.inchesToMeters(10.27),
                  Units.inchesToMeters(10.75)),
              new Rotation3d(0, 0, Math.PI));

      public static final Transform3d kAlignOffset =
          new Transform3d(
              new Translation3d(0.1, 0.12, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(10)));
    }

    public static final class RightCamera {
      public static final String kCameraName = "rightCamera";

      // z is good
      public static final Transform3d kRobotToCamera =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-6.94),
                  Units.inchesToMeters(-10.27),
                  Units.inchesToMeters(10.75)),
              new Rotation3d(0, 0, Math.PI));

      public static final Transform3d kAlignOffset =
          new Transform3d(
              new Translation3d(0.1, 0.12, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(10)));
    }
  }

  public static final class Elevator {
    // These are inverted, but I'm not gonna change it cause it'll probably break something
    public static final int kLeftElevatorCANid = 9;
    public static final int kRightElevatorCANid = 10;

    public static final double kSimLerpSpeed = 60;

    public static final SparkMax kLeftElevatorSparkMax =
        new SparkMax(kLeftElevatorCANid, MotorType.kBrushless);
    public static final SparkMax kRightElevatorSparkMax =
        new SparkMax(kRightElevatorCANid, MotorType.kBrushless);

    // TODO: determine reverse or forward
    public static final DigitalInput kElevatorLimitSwitch = new DigitalInput(0);

    public static final SparkMaxConfig kLeftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightElevatorConfig = new SparkMaxConfig();

    // only one cause we slave the other motor to this one
    public static final SparkClosedLoopController kElevatorController =
        kRightElevatorSparkMax.getClosedLoopController();

    // TODO: veloc and accel is in inches per second and inches per second squared
    public static final PIDConstants kElevatorPIDConstants =
        new PIDConstants(0.15, 0.0, 0.0, 0.5, 0.5);
    public static final double kG = 0.35;

    // TODO: figure out the heights
    public static final HashMap<ElevatorState, Double> kHeights =
        new HashMap<>() {
          {
            put(ElevatorState.DOWN, 0.0);
            put(ElevatorState.L1, 0.0);
            put(ElevatorState.L2, 5.75);
            put(ElevatorState.L3, 14.0);
            put(ElevatorState.L4, 28.0);
          }
        };

    // TODO: figure out the conversion factor
    public static final double kConversionFactor = 9.0 / 23.0;

    // TODO: figure out the tolerance
    public static final double kElevatorTolerance = 0.01;

    // distance that the elevator steps when zeroing
    public static final double kZeroingStep = 0.01;

    public static final double kCustomStep = 0.25;

    static {
      kLeftElevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30);
      kRightElevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30).inverted(true);

      kLeftElevatorConfig.follow(
          kRightElevatorSparkMax, true); // you can pass in an inverted value after the
      // kLeftElevatorSparkMax, but idk quite how that works yet

      kRightElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kElevatorPIDConstants.kP, kElevatorPIDConstants.kI, kElevatorPIDConstants.kD)
          // Now using these, comment them out if it breaks
          .maxMotion
          .maxAcceleration(kElevatorPIDConstants.kMaxAcceleration)
          .maxVelocity(kElevatorPIDConstants.kMaxVelocity);

      kRightElevatorConfig.encoder.positionConversionFactor(kConversionFactor);

      // TODO: see if this is right
      // it's not on there yet :(
      kRightElevatorConfig
          .limitSwitch
          .forwardLimitSwitchEnabled(false)
          .forwardLimitSwitchType(Type.kNormallyOpen);
    }
  }

  // TODO: find like all of these
  public static final class Handler {
    public static final class Coral {

      // Beam Break

      public static final DigitalInput kBeamBreak = new DigitalInput(5);

      // TODO: figure these out
      public static final int kLeftMotorCANid = 11;
      public static final int kRightMotorCANid = 12;

      public static final SparkMax kLeftSparkMax =
          new SparkMax(kLeftMotorCANid, SparkMax.MotorType.kBrushless);
      public static final SparkMax kRightSparkMax =
          new SparkMax(kRightMotorCANid, SparkMax.MotorType.kBrushless);

      public static final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
      public static final SparkMaxConfig kRightConfig = new SparkMaxConfig();

      public static final boolean kLeftInverted = true;
      public static final boolean kRightInverted = false;

      public static final HashMap<CoralHandlerState, Double> kSpeeds =
          new HashMap<>() {
            {
              put(CoralHandlerState.INACTIVE, 0.0);
              put(CoralHandlerState.GRAB, 0.3);
              put(CoralHandlerState.RELEASE, 0.3);
              put(CoralHandlerState.REVERSE, -0.3);
            }
          };

      // TODO: find these
      public static final double kDetectionDelayTimeMS = 150;
      public static final double kReleaseTimeMS = 1000;

      static {
        kLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).inverted(kLeftInverted);

        kRightConfig.apply(kLeftConfig).inverted(kRightInverted);
      }
    }
  }

  public static final class Climber {
    // TODO: figure this out
    public static final int kClimberMotorCanId = 15;

    public static final SparkMax kClimberSparkMax =
        new SparkMax(kClimberMotorCanId, SparkMax.MotorType.kBrushless);

    public static final SparkMaxConfig kClimberConfig = new SparkMaxConfig();

    public static final boolean kClimberInverted = false;

    // TODO: find these
    public static final HashMap<ClimberState, Double> kSpeeds =
        new HashMap<ClimberState, Double>() {
          {
            put(ClimberState.INACTIVE, 0.0);
            put(ClimberState.UP, 0.4);
            put(ClimberState.DOWN, -1.0);
            put(ClimberState.DOWNSLOW, -0.4);
          }
        };

    // TODO: find this
    public static final double[] kClimberLimits = {20.0, 180.0};

    // TODO: find this
    public static final double kDownPosition = 0.0;
    public static final double kClimberUpPosition = 35.0;
    public static final double kClimberDownPosition = -35.0;

    static {
      kClimberConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(kClimberInverted)
          .absoluteEncoder
          .positionConversionFactor(360)
          .inverted(true);
    }
  }

  public static final class Chute {
    public static final Servo kServo = new Servo(0);

    public static final HashMap<ChuteState, Double> kPositions =
        new HashMap<ChuteState, Double>() {
          {
            put(ChuteState.UP, 0.0);
            put(ChuteState.DOWN, 90.0);
          }
        };
  }

  public static final class DriveConstants {

    public static final BooleanSupplier kRateLimitsEnabled = () -> true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = kTau; // radians per second

    // First one is normal, second is slow
    public static final double[] kRotationSpeeds = {0.9, 0.5};
    public static final double[] kDrivingSpeeds = {1.2, 0.7};
  }
}
