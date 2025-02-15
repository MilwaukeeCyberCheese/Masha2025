package frc.robot.utils;

/** PID constants used to create PID controllers */
public class PIDConstants {
  /** P */
  public final double p;

  /** I */
  public final double i;

  /** D */
  public final double d;

  /** FF */
  public final double ff;

  /** IZone */
  public final double iZone;

  /** Max Velocity (for use with MAXMOTION) */
  public final double maxVelocity;

  /** Max Acceleration (for use with MAXMOTION) */
  public final double maxAcceleration;

  /**
   * Create a new PIDConstants object
   *
   * @param p P
   * @param i I
   * @param d D
   * @param ff Feedforward
   * @param iZone IZone
   * @param maxVelocity Max Velocity (for use with MAXMOTION)
   * @param maxAcceleration Max Acceleration (for use with MAXMOTION)
   */
  public PIDConstants(
      double p,
      double i,
      double d,
      double ff,
      double iZone,
      double maxVelocity,
      double maxAcceleration) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.ff = ff;
    this.iZone = iZone;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }

  /**
   * Create a new PIDConstants object
   *
   * @param p P
   * @param i I
   * @param d D
   */
  public PIDConstants(double p, double i, double d) {
    this(p, i, d, 0.0, 0.0, -1.0, -1.0);
  }

  /**
   * Create a new PIDConstants object
   *
   * @param p P
   * @param i I
   * @param d D
   * @param ff Feedforward
   */
  public PIDConstants(double p, double i, double d, double ff) {
    this(p, i, d, ff, 0.0, -1.0, -1.0);
  }

  /**
   * Create a new PIDConstants object
   *
   * @param p P
   * @param i I
   * @param d D
   * @param maxVelocity Max Velocity (for use with MAXMOTION)
   * @param maxAcceleration Max Acceleration (for use with MAXMOTION)
   */
  public PIDConstants(
          double p, double i, double d, double maxVelocity, double maxAcceleration) {
    this(p, i, d, 0.0, 0.0, maxVelocity, maxAcceleration);
  }

  /** Return a string of all the values */
  public String toString() {
    return "P: "
        + p
        + " I: "
        + i
        + " D: "
        + d
        + " FF: "
        + ff
        + " IZone: "
        + iZone
        + " MaxVelocity: "
        + maxVelocity
        + " MaxAcceleration: "
        + maxAcceleration;
  }

  /** Create a new PIDConstants object from string */
  public static PIDConstants parsePIDConstants(String str) {
    String[] parts = str.split(" ");
    try {
      return new PIDConstants(
          Double.parseDouble(parts[1]),
          Double.parseDouble(parts[3]),
          Double.parseDouble(parts[5]),
          Double.parseDouble(parts[7]),
          Double.parseDouble(parts[9]),
          Double.parseDouble(parts[11]),
          Double.parseDouble(parts[13]));
    } catch (Exception e) {
      throw new IllegalArgumentException(
          "Invalid PIDConstants string format. Must be 'P: <double> I: <double> D: <double> FF: <double> IZone: <double> MaxVelocity: <double> MaxAcceleration: <double>'");
    }
  }
}
