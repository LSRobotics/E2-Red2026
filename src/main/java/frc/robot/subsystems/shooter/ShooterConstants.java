package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // CAN IDs 
  public static final int kRightMotorCanId = 60;
  public static final int kLeftMotorCanId = 10;

  // Mechanical Configuration
  public static final double kGearReduction = 1.0; 
  public static final double kShootTargetRPM = 2000.0;
  public static final double kShooterToleranceRPM = 30.0;
  public static final double kRpmAdjustmentPercent = 0.0005; // 0.05% per bumper tap

  // PID & Feedforward gains
  public static final double kP = 0.18;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.096828; // Volts per RPS
  public static final double kS = 0.25;

  // Safety
  public static final int kCurrentLimitAmps = 40;
}