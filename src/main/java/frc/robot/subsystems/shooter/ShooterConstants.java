package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // CAN IDs
  public static final int kTopMotorCanId = 1;
  public static final int kBottomMotorCanId = 2;

  // Mechanical Configuration
  public static final double kGearReduction = 1.0; 
  public static final double kShootTargetRPM = 2000.0;

  // Velocity Tolerance (Shooter is "ready" if within +/- 30 RPM of target)
  public static final double kShooterToleranceRPM = 30.0;

  // Real Hardware Closed-Loop PID & Feedforward (Phoenix 6 Slot 0)
  public static final double kP = 0.18;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.096828; 
  public static final double kS = 0.25; 
}