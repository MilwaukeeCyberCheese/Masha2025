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
  public static final double TAU = Math.PI * 2;

  public static class Sensors {
    public static final AHRS gyro = new AHRS(NavXComType.kUSB1);
    public static final Rev2mDistanceSensor handlerDistanceSensor =
        new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    static {
      handlerDistanceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
    }
  }

  public static final class IOConstants {
    public static final int OPERATOR_CONTROLLER_PORT = 0;
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 2;
    public static final int BUTTON_BOARD_PORT = 3;
    public static final double DRIVE_DEADBAND = 0.05;

    // When test mode is enabled, the operator controller is used for driving and testing
    // This should always be false on the main branch
    public static final boolean TEST_MODE = false;
  }

  public static final class Vision {
    public static final String CAMERA_NAME = "Brio_100";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  }

  // TODO: figure out the best way to run the elevator
  // is it separate PIDs running locally? (also this one means another encoder is
  // needed)
  // or is it a single PID running on the roborio?
  // or is it a single PID running locally, and one slaved to it? (probably this
  // one)
  public static final class Elevator {
    // TODO: figure these out
    public static final int LEFT_ELEVATOR_C_A_NID = 9;
    public static final int RIGHT_ELEVATOR_C_A_NID = 10;

    public static final double SIM_LERP_SPEED = 60;

    public static final SparkMax kLeftElevatorSparkMax =
        new SparkMax(LEFT_ELEVATOR_C_A_NID, MotorType.kBrushless);
    public static final SparkMax RIGHT_ELEVATOR_SPARK_MAX =
        new SparkMax(RIGHT_ELEVATOR_C_A_NID, MotorType.kBrushless);

    public static final SparkLimitSwitch ELEVATOR_LIMIT_SWITCH =
        RIGHT_ELEVATOR_SPARK_MAX.getReverseLimitSwitch();

    public static final SparkMaxConfig LEFT_ELEVATOR_CONFIG = new SparkMaxConfig();
    public static final SparkMaxConfig RIGHT_ELEVATOR_CONFIG = new SparkMaxConfig();

    // TODO: positive should be up
    public static final boolean LEFT_INVERTED = false;
    public static final boolean RIGHT_INVERTED = true;

    // only one cause we slave the other motor to this one
    public static final SparkClosedLoopController ELEVATOR_CONTROLLER =
        RIGHT_ELEVATOR_SPARK_MAX.getClosedLoopController();

    // TODO: veloc and accel is in inches per second and inches per second squared
    public static final PIDConstants ELEVATOR_P_I_D_CONSTANTS =
        new PIDConstants(0.1, 0.0, 0.0, 16.0, 20.0);

    // TODO: figure out the heights
    public static final HashMap<ElevatorState, Double> HEIGHTS =
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
    public static final double CONVERSION_FACTOR = 1.0;

    // TODO: figure out the tolerance
    public static final double ELEVATOR_TOLERANCE = 0.01;

    // distance that the elevator steps when zeroing
    public static final double ZEROING_STEP = 0.01;

    /*
     * TODO: TEST IN SIMULATION THE DIRECTION THE LIFT MOTORS SPIN
     * ISTFG WE HAVE TO DO THIS CAUSE THEY'RE MECHANICALLY LINKED
     * IF IT GETS MESSED UP, I'M LOSING IT
     */
    static {
      LEFT_ELEVATOR_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(LEFT_INVERTED);
      RIGHT_ELEVATOR_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(RIGHT_INVERTED);

      LEFT_ELEVATOR_CONFIG.follow(
          kLeftElevatorSparkMax); // you can pass in an inverted value after the
      // kLeftElevatorSparkMax, but idk quite how that works yet

      RIGHT_ELEVATOR_CONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ELEVATOR_P_I_D_CONSTANTS.kP, ELEVATOR_P_I_D_CONSTANTS.kI, ELEVATOR_P_I_D_CONSTANTS.kD)
          .maxMotion
          .maxAcceleration(ELEVATOR_P_I_D_CONSTANTS.kMaxAcceleration)
          .maxVelocity(ELEVATOR_P_I_D_CONSTANTS.kMaxVelocity);

      RIGHT_ELEVATOR_CONFIG.encoder.positionConversionFactor(CONVERSION_FACTOR);

      // TODO: see if this is right
      RIGHT_ELEVATOR_CONFIG
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
    }
  }

  // TODO: find like all of these
  public static final class Handler {
    public static final class Coral {
      // TODO: figure these out
      public static final int LEFT_MOTOR_C_A_NID = 11;
      public static final int RIGHT_MOTOR_C_A_NID = 12;

      public static final SparkMax LEFT_SPARK_MAX =
          new SparkMax(LEFT_MOTOR_C_A_NID, SparkMax.MotorType.kBrushless);
      public static final SparkMax RIGHT_SPARK_MAX =
          new SparkMax(RIGHT_MOTOR_C_A_NID, SparkMax.MotorType.kBrushless);

      public static final SparkMaxConfig LEFT_CONFIG = new SparkMaxConfig();
      public static final SparkMaxConfig RIGHT_CONFIG = new SparkMaxConfig();

      public static final SparkClosedLoopController LEFT_CONTROLLER =
          LEFT_SPARK_MAX.getClosedLoopController();
      public static final SparkClosedLoopController RIGHT_CONTROLLER =
          RIGHT_SPARK_MAX.getClosedLoopController();

      // Max accel is in RPM
      public static final PIDConstants P_I_D_CONSTANTS = new PIDConstants(0.1, 0.0, 0.0, -1.0, 300.0);

      public static final boolean LEFT_INVERTED = true;
      public static final boolean RIGHT_INVERTED = false;

      // TODO: find these
      public static final HashMap<CoralHandlerState, Double> SPEEDS =
          new HashMap<>() {
            {
              put(CoralHandlerState.INACTIVE, 0.0);
              put(CoralHandlerState.GRAB, 50.0);
              put(CoralHandlerState.RELEASE, 20.00);
            }
          };

      // TODO: find these
      public static final double CONVERSION_FACTOR = 1.0;
      public static final double TOLERANCE = 10;
      public static final double DETECTION_DELAY_TIME_M_S = 1000;
      public static final double HAS_CORAL_DISTANCE = 2.0;

      static {
        LEFT_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(20).inverted(LEFT_INVERTED);

        LEFT_CONFIG.encoder.velocityConversionFactor(CONVERSION_FACTOR);

        LEFT_CONFIG
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(P_I_D_CONSTANTS.kP, P_I_D_CONSTANTS.kI, P_I_D_CONSTANTS.kD)
            .outputRange(-1, 1)
            .maxMotion
            .maxAcceleration(P_I_D_CONSTANTS.kMaxAcceleration);

        RIGHT_CONFIG.apply(LEFT_CONFIG).inverted(RIGHT_INVERTED);
      }
    }

    public static final class Algae {
      public static final int POSITION_MOTOR_CAN_ID = 13;
      public static final int INTAKE_MOTOR_CAN_ID = 14;

      public static final SparkMax POSITION_SPARK_MAX =
          new SparkMax(POSITION_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
      public static final SparkMax INTAKE_SPARK_MAX =
          new SparkMax(INTAKE_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);

      public static final SparkMaxConfig POSITION_CONFIG = new SparkMaxConfig();
      public static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig();

      // TODO: find out if it's inverted
      public static final boolean POSITION_INVERTED = false;
      public static final boolean INTAKE_INVERTED = false;

      public static final SparkClosedLoopController POSITION_CONTROLLER =
          POSITION_SPARK_MAX.getClosedLoopController();
      public static final SparkClosedLoopController INTAKE_CONTROLLER =
          INTAKE_SPARK_MAX.getClosedLoopController();

      public static final PIDConstants POSITION_P_I_D_CONSTANTS = new PIDConstants(0.1, 0.0, 0.0);
      public static final PIDConstants INTAKE_P_I_D_CONSTANTS = new PIDConstants(0.1, 0.0, 0.0);

      // TODO: confirm that this is right
      public static final double POSITION_CONVERSION_FACTOR = Math.PI * 2;
      public static final double INTAKE_CONVERSION_FACTOR = 1.0;

      // TODO: find these
      public static final HashMap<AlgaeHandlerPositionState, Double> POSITIONS =
          new HashMap<AlgaeHandlerPositionState, Double>() {
            {
              put(AlgaeHandlerPositionState.STOWED, 0.0);
              put(AlgaeHandlerPositionState.GRAB_FROM_REEF, 0.0);
              put(AlgaeHandlerPositionState.GRAB_FROM_GROUND, 0.0);
            }
          };
      public static final HashMap<AlgaeHandlerIntakeState, Double> SPEEDS =
          new HashMap<AlgaeHandlerIntakeState, Double>() {
            {
              put(AlgaeHandlerIntakeState.INTAKE, 0.0);
              put(AlgaeHandlerIntakeState.OUTTAKE, 0.0);
              put(AlgaeHandlerIntakeState.STOPPED, 0.0);
            }
          };

      // TODO: find this
      public static final double[] POSITION_LIMITS = {0.0, 0.0};

      // TODO: find this
      public static final double POSITION_TOLERANCE = 0.01;
      public static final double INTAKE_TOLERANCE = 10;

      static {
        POSITION_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(POSITION_INVERTED);
        POSITION_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(INTAKE_INVERTED);

        POSITION_CONFIG
            .absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        INTAKE_CONFIG.encoder.velocityConversionFactor(INTAKE_CONVERSION_FACTOR / 60);

        POSITION_CONFIG
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(POSITION_P_I_D_CONSTANTS.kP, POSITION_P_I_D_CONSTANTS.kI, POSITION_P_I_D_CONSTANTS.kD)
            .outputRange(-1, 1);

        INTAKE_CONFIG
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(INTAKE_P_I_D_CONSTANTS.kP, INTAKE_P_I_D_CONSTANTS.kI, INTAKE_P_I_D_CONSTANTS.kD)
            .outputRange(-1, 1);
      }
    }
  }

  public static final class Climber {
    // TODO: figure this out
    public static final int CLIMBER_MOTOR_CAN_ID = 15;

    public static final SparkMax CLIMBER_SPARK_MAX =
        new SparkMax(CLIMBER_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);

    public static final SparkMaxConfig CLIMBER_CONFIG = new SparkMaxConfig();

    // TODO: find out if it's inverted
    public static final boolean CLIMBER_INVERTED = false;

    public static final SparkClosedLoopController CLIMBER_CONTROLLER =
        CLIMBER_SPARK_MAX.getClosedLoopController();

    public static final PIDConstants CLIMBER_P_I_D_CONSTANTS = new PIDConstants(0.1, 0.0, 0.0);

    // TODO: confirm that this is right
    public static final double CONVERSION_FACTOR = Math.PI * 2;

    // TODO: find these
    public static final HashMap<ClimberState, Double> POSITIONS =
        new HashMap<ClimberState, Double>() {
          {
            put(ClimberState.WAITING, 0.0);
            put(ClimberState.STOWED, 0.0);
            put(ClimberState.CLIMB, 0.0);
          }
        };

    // TODO: find this
    public static final double[] CLIMBER_LIMITS = {0.0, 0.0};

    // TODO: find this
    public static final double CLIMBER_TOLERANCE = 0.01;

    static {
      CLIMBER_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(CLIMBER_INVERTED);

      CLIMBER_CONFIG
          .absoluteEncoder
          .positionConversionFactor(CONVERSION_FACTOR)
          .velocityConversionFactor(CONVERSION_FACTOR / 60.0);

      CLIMBER_CONFIG
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(CLIMBER_P_I_D_CONSTANTS.kP, CLIMBER_P_I_D_CONSTANTS.kI, CLIMBER_P_I_D_CONSTANTS.kD)
          .outputRange(-1, 1);
    }
  }

  public static final class DriveConstants {

    public static final BooleanSupplier RATE_LIMITS_ENABLED = () -> true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = TAU; // radians per second

    // First one is normal, second is slow
    public static final double[] ROTATION_SPEEDS = {0.7, 0.3};
    public static final double[] DRIVING_SPEEDS = {0.5, 0.3};
  }
}
