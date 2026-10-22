package frc.robot.subsystems.indexer;

public final class IndexerConstants {
  // RoboRIO PWM port (change to match your physical wiring)
  public static final int kIndexerPwmChannel = 0;

  // Servo pulse width bounds (microseconds)
  public static final int kMaxPulseUs = 2000;
  public static final int kMinPulseUs = 1000;

  // Position setpoints (0.0 = fully retracted, 1.0 = fully extended)
  public static final double kExtendedPosition = 1.0;
  public static final double kRetractedPosition = 0.0;
}