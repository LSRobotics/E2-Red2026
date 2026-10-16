package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // CAN IDs
  public static final int kRightMotorCanId = 1;
  public static final int kLeftMotorCanId = 2;

  // Mechanical Configuration
  public static final double kGearReduction = 1.0; 
  public static final double kShootTargetRPM = 2000.0;

  // Velocity Tolerance (Shooter is ready if within +/- 30 RPM of target) Also for shuffle board
  public static final double kShooterToleranceRPM = 30.0;

  //PID & Feedforward. Only tested in sim for now. Will need to be tuned on robot.
  public static final double kP = 0.18;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.096828; // took forever to get this number
  public static final double kS = 0.25; 
}