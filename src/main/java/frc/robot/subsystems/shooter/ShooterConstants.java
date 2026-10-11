package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  // CAN IDs
  public static final int kTopMotorCanId = 22;
  public static final int kBottomMotorCanId = 23;

  // Mechanical Configuration
  public static final double kGearReduction = 1.0; // 1:1 direct drive
  public static final double kShootTargetRPM = 2000.0; // Target RPM for shooting

  // Real Hardware Closed-Loop PID & Feedforward (Phoenix 6 Slot 0)
  public static final double kP = 0.12;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.11; // Volts per RPS (Radial Velocity)
  public static final double kS = 0.25; // Friction override voltage
}