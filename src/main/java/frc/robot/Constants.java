// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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
    public static final DigitalInput elevatorLimitSwitch = new DigitalInput(0);
    public static final Rev2mDistanceSensor handlerDistanceSensor =
        new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    static {
      handlerDistanceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
    }
  }

  public static class OIConstants {
    public static final int kOperatorControllerPort = 0;
    public static final int kLeftJoystickPort = 1;
    public static final int kRightJoystickPort = 2;
    public static final int kButtonBoardPort = 3;
    public static final double kDriveDeadband = 0.05;
  }

  public static class Vision {
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
  public static class Elevator {
    // TODO: figure these out
    public static final int kLeftElevatorCANid = 9;
    public static final int kRightElevatorCANid = 10;

    public static final double kSimLerpSpeed = 60;

    public static final SparkMax kLeftElevatorSparkMax =
        new SparkMax(kLeftElevatorCANid, MotorType.kBrushless);
    public static final SparkMax kRightElevatorSparkMax =
        new SparkMax(kRightElevatorCANid, MotorType.kBrushless);

    public static final SparkMaxConfig kLeftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightElevatorConfig = new SparkMaxConfig();

    // TODO: positive should be up
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;

    public static final boolean kLeftEncoderInverted = false;

    // only one cause we slave the other motor to this one
    public static final SparkClosedLoopController kElevatorController =
        kLeftElevatorSparkMax.getClosedLoopController();

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

      kRightElevatorConfig.follow(
          kLeftElevatorSparkMax); // you can pass in an inverted value after the
      // kLeftElevatorSparkMax, but idk quite how that works yet

      kLeftElevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kElevatorPIDConstants.kP, kElevatorPIDConstants.kI, kElevatorPIDConstants.kD)
          .maxMotion
          .maxAcceleration(kElevatorPIDConstants.kMaxAcceleration)
          .maxVelocity(kElevatorPIDConstants.kMaxVelocity);

      kLeftElevatorConfig
          .encoder
          .positionConversionFactor(kConversionFactor)
          .inverted(kLeftEncoderInverted);
    }
  }

  // TODO: find like all of these
  public static class Handler {
    public static class Coral {
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

    public static class Algae {
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

  public static class Climber {
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

  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = kTau;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class DriveConstants {

    public static final BooleanSupplier kRateLimitsEnabled = () -> true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = kTau; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kSlowModifier = 0.25;
    public static final double kTurningSpeedModifier = 0.5;

    // TODO: what is it?
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);
    // Distance between front and back wheels on robot
    // TODO: make it work with choreo
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    // TODO: what are they?
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    // TODO: is it?
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    // TODO: change this depending on what we do
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  // Not digits cause java doesn't like that
  public static final class FiveFiftyMotorConstants {
    public static final double kFreeSpeedRPM = 11000;
  }
}
