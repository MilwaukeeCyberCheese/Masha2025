// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.AlgaeHandlerSubsystem.AlgaeHandlerIntakeState;
import frc.robot.subsystems.AlgaeHandlerSubsystem.AlgaeHandlerPositionState;
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

  public static class Sensors {
    public static final AHRS gyro = new AHRS(NavXComType.kUSB1);
    public static final Rev2mDistanceSensor handlerDistanceSensor =
        new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    static {
      handlerDistanceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
    }
  }

  public static final class IOConstants {
    public static final int kControllerPort = 0;
    public static final int kLeftJoystickPort = 1;
    public static final int kRightJoystickPort = 2;
    public static final int kButtonBoardPort = 3;
    public static final double kDriveDeadband = 0.05;

    // When test mode is enabled, the operator controller is used for driving and testing
    // This should always be false on the main branch
    public static final boolean kTestMode = false;
  }

  public static final class Vision {
    public static final String kCameraName = "Brio_100";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  // TODO: figure out the best way to run the elevator
  // is it separate PIDs running locally? (also this one means another encoder is
  // needed)
  // or is it a single PID running on the roborio?
  // or is it a single PID running locally, and one slaved to it? (probably this
  // one)
  public static final class Elevator {
    // TODO: figure these out
    public static final int kLeftElevatorCANid = 9;
    public static final int kRightElevatorCANid = 10;

    public static final double kSimLerpSpeed = 60;

    public static final SparkMax kLeftElevatorSparkMax =
        new SparkMax(kLeftElevatorCANid, MotorType.kBrushless);
    public static final SparkMax kRightElevatorSparkMax =
        new SparkMax(kRightElevatorCANid, MotorType.kBrushless);

    public static final SparkLimitSwitch kElevatorLimitSwitch =
        kRightElevatorSparkMax.getReverseLimitSwitch();

    public static final SparkMaxConfig kLeftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightElevatorConfig = new SparkMaxConfig();

    // TODO: positive should be up
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;

    // only one cause we slave the other motor to this one
    public static final SparkClosedLoopController kElevatorController =
        kRightElevatorSparkMax.getClosedLoopController();

    // TODO: veloc and accel is in inches per second and inches per second squared
    public static final PIDConstants kElevatorPIDConstants =
        new PIDConstants(0.1, 0.0, 0.0, 16.0, 20.0);

    // TODO: figure out the heights
    public static final HashMap<ElevatorState, Double> kHeights =
        new HashMap<>() {
          {
            put(ElevatorState.DOWN, 0.0);
            put(ElevatorState.L1, 16.5);
            put(ElevatorState.L2, 20.0);
            put(ElevatorState.L3, 45.0);
            put(ElevatorState.L4, 0.4);
            put(ElevatorState.ALGAE_FROM_REEF, 0.5);
            put(ElevatorState.ALGAE_FROM_FLOOR, 0.6);
          }
        };

    // TODO: figure out the conversion factor
    public static final double kConversionFactor = 1.0;

    // TODO: figure out the tolerance
    public static final double kElevatorTolerance = 0.01;

    // distance that the elevator steps when zeroing
    public static final double kZeroingStep = 0.01;

    /*
     * TODO: TEST IN SIMULATION THE DIRECTION THE LIFT MOTORS SPIN
     * ISTFG WE HAVE TO DO THIS CAUSE THEY'RE MECHANICALLY LINKED
     * IF IT GETS MESSED UP, I'M LOSING IT
     */
    static {
      kLeftElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(kLeftInverted);
      kRightElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(kRightInverted);

      kLeftElevatorConfig.follow(
          kLeftElevatorSparkMax); // you can pass in an inverted value after the
      // kLeftElevatorSparkMax, but idk quite how that works yet

      kRightElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kElevatorPIDConstants.kP, kElevatorPIDConstants.kI, kElevatorPIDConstants.kD)
          .maxMotion
          .maxAcceleration(kElevatorPIDConstants.kMaxAcceleration)
          .maxVelocity(kElevatorPIDConstants.kMaxVelocity);

      kRightElevatorConfig.encoder.positionConversionFactor(kConversionFactor);

      // TODO: see if this is right
      kRightElevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
    }
  }

  // TODO: find like all of these
  public static final class Handler {
    public static final class Coral {
      // TODO: figure these out
      public static final int kLeftMotorCANid = 11;
      public static final int kRightMotorCANid = 12;

      public static final SparkMax kLeftSparkMax =
          new SparkMax(kLeftMotorCANid, SparkMax.MotorType.kBrushless);
      public static final SparkMax kRightSparkMax =
          new SparkMax(kRightMotorCANid, SparkMax.MotorType.kBrushless);

      public static final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
      public static final SparkMaxConfig kRightConfig = new SparkMaxConfig();

      public static final SparkClosedLoopController kLeftController =
          kLeftSparkMax.getClosedLoopController();
      public static final SparkClosedLoopController kRightController =
          kRightSparkMax.getClosedLoopController();

      // Max accel is in RPM
      public static final PIDConstants kPIDConstants = new PIDConstants(0.1, 0.0, 0.0, -1.0, 300.0);

      public static final boolean kLeftInverted = true;
      public static final boolean kRightInverted = false;

      // TODO: find these
      public static final HashMap<CoralHandlerState, Double> kSpeeds =
          new HashMap<>() {
            {
              put(CoralHandlerState.INACTIVE, 0.0);
              put(CoralHandlerState.GRAB, 50.0);
              put(CoralHandlerState.RELEASE, 20.00);
            }
          };

      // TODO: find these
      public static final double kConversionFactor = 1.0;
      public static final double kTolerance = 10;
      public static final double kDetectionDelayTimeMS = 1000;
      public static final double kHasCoralDistance = 2.0;

      static {
        kLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).inverted(kLeftInverted);

        kLeftConfig.encoder.velocityConversionFactor(kConversionFactor);

        kLeftConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kPIDConstants.kP, kPIDConstants.kI, kPIDConstants.kD)
            .outputRange(-1, 1)
            .maxMotion
            .maxAcceleration(kPIDConstants.kMaxAcceleration);

        kRightConfig.apply(kLeftConfig).inverted(kRightInverted);
      }
    }

    public static final class Algae {
      public static final int kPositionMotorCanId = 13;
      public static final int kIntakeMotorCanId = 14;

      public static final SparkMax kPositionSparkMax =
          new SparkMax(kPositionMotorCanId, SparkMax.MotorType.kBrushless);
      public static final SparkMax kIntakeSparkMax =
          new SparkMax(kIntakeMotorCanId, SparkMax.MotorType.kBrushless);

      public static final SparkMaxConfig kPositionConfig = new SparkMaxConfig();
      public static final SparkMaxConfig kIntakeConfig = new SparkMaxConfig();

      // TODO: find out if it's inverted
      public static final boolean kPositionInverted = false;
      public static final boolean kIntakeInverted = false;

      public static final SparkClosedLoopController kPositionController =
          kPositionSparkMax.getClosedLoopController();
      public static final SparkClosedLoopController kIntakeController =
          kIntakeSparkMax.getClosedLoopController();

      public static final PIDConstants kPositionPIDConstants = new PIDConstants(0.1, 0.0, 0.0);
      public static final PIDConstants kIntakePIDConstants = new PIDConstants(0.1, 0.0, 0.0);

      // TODO: confirm that this is right
      public static final double kPositionConversionFactor = Math.PI * 2;
      public static final double kIntakeConversionFactor = 1.0;

      // TODO: find these
      public static final HashMap<AlgaeHandlerPositionState, Double> kPositions =
          new HashMap<AlgaeHandlerPositionState, Double>() {
            {
              put(AlgaeHandlerPositionState.STOWED, 0.0);
              put(AlgaeHandlerPositionState.GRAB_FROM_REEF, 0.0);
              put(AlgaeHandlerPositionState.GRAB_FROM_GROUND, 0.0);
            }
          };
      public static final HashMap<AlgaeHandlerIntakeState, Double> kSpeeds =
          new HashMap<AlgaeHandlerIntakeState, Double>() {
            {
              put(AlgaeHandlerIntakeState.INTAKE, 0.0);
              put(AlgaeHandlerIntakeState.OUTTAKE, 0.0);
              put(AlgaeHandlerIntakeState.STOPPED, 0.0);
            }
          };

      // TODO: find this
      public static final double[] kPositionLimits = {0.0, 0.0};

      // TODO: find this
      public static final double kPositionTolerance = 0.01;
      public static final double kIntakeTolerance = 10;

      static {
        kPositionConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(kPositionInverted);
        kPositionConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(kIntakeInverted);

        kPositionConfig
            .absoluteEncoder
            .positionConversionFactor(kPositionConversionFactor)
            .velocityConversionFactor(kPositionConversionFactor / 60.0);
        kIntakeConfig.encoder.velocityConversionFactor(kIntakeConversionFactor / 60);

        kPositionConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kPositionPIDConstants.kP, kPositionPIDConstants.kI, kPositionPIDConstants.kD)
            .outputRange(-1, 1);

        kIntakeConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kIntakePIDConstants.kP, kIntakePIDConstants.kI, kIntakePIDConstants.kD)
            .outputRange(-1, 1);
      }
    }
  }

  public static final class Climber {
    // TODO: figure this out
    public static final int kClimberMotorCanId = 15;

    public static final SparkMax kClimberSparkMax =
        new SparkMax(kClimberMotorCanId, SparkMax.MotorType.kBrushless);

    public static final SparkMaxConfig kClimberConfig = new SparkMaxConfig();

    // TODO: find out if it's inverted
    public static final boolean kClimberInverted = false;

    public static final SparkClosedLoopController kClimberController =
        kClimberSparkMax.getClosedLoopController();

    public static final PIDConstants kClimberPIDConstants = new PIDConstants(0.1, 0.0, 0.0);

    // TODO: confirm that this is right
    public static final double kConversionFactor = Math.PI * 2;

    // TODO: find these
    public static final HashMap<ClimberState, Double> kPositions =
        new HashMap<ClimberState, Double>() {
          {
            put(ClimberState.WAITING, 0.0);
            put(ClimberState.STOWED, 0.0);
            put(ClimberState.CLIMB, 0.0);
          }
        };

    // TODO: find this
    public static final double[] kClimberLimits = {0.0, 0.0};

    // TODO: find this
    public static final double kClimberTolerance = 0.01;

    static {
      kClimberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(kClimberInverted);

      kClimberConfig
          .absoluteEncoder
          .positionConversionFactor(kConversionFactor)
          .velocityConversionFactor(kConversionFactor / 60.0);

      kClimberConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(kClimberPIDConstants.kP, kClimberPIDConstants.kI, kClimberPIDConstants.kD)
          .outputRange(-1, 1);
    }
  }

  public static final class DriveConstants {

    public static final BooleanSupplier kRateLimitsEnabled = () -> true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = kTau; // radians per second

    // First one is normal, second is slow
    public static final double[] kRotationSpeeds = {0.7, 0.3};
    public static final double[] kDrivingSpeeds = {0.5, 0.3};
  }
}
