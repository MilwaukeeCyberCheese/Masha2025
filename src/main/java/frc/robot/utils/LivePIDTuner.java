package frc.robot.utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * An object that will automatically update a PID controller's constants based on the SmartDashboard
 */
public class LivePIDTuner {
  private DashboardUpdater<PIDConstants> pidConstants;

  /**
   * Instantiates a new LivePIDTuner
   *
   * @param name
   * @param controller
   * @param constants
   */
  public LivePIDTuner(String name, SparkClosedLoopController controller, PIDConstants constants) {
    pidConstants = new DashboardUpdater<PIDConstants>(name, constants);
  }

  /** Update the pid based off the constants */
  public void update(SparkMax spark) {
    pidConstants.update();
    setSparkPID(spark, pidConstants.get());
  }

  /**
   * @return PIDConstants
   */
  public PIDConstants getConstants() {
    pidConstants.update();
    return pidConstants.get();
  }

  /**
   * Set the PID constants of a SparkPIDController
   *
   * @param controller
   * @param constants
   */
  public static void setSparkPID(SparkMax controller, PIDConstants constants) {
    SparkMaxConfig config = new SparkMaxConfig();
    if (constants.maxAcceleration != -1.0) {
      config.closedLoop.maxMotion.maxVelocity(constants.maxAcceleration);
    }
    if (constants.maxVelocity != -1.0) {
      config.closedLoop.maxMotion.maxAcceleration(constants.maxVelocity);
    }

    config.closedLoop.pid(constants.p, constants.i, constants.d);
    config.closedLoop.iZone(constants.iZone);
    config.closedLoop.velocityFF(constants.ff);

    controller.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
