package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean topMotorConnected = false;
    public boolean bottomMotorConnected = false;
    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double targetVelocityRPM = 0.0;
  }
  
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocity(double rpm) {}

  public default void stopMotor() {}
}