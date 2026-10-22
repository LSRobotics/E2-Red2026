package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean rightMotorConnected = false;
    public boolean leftMotorConnected = false;
    public double rightAppliedVolts = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightVelocityRPM = 0.0;
    public double leftVelocityRPM = 0.0;
    public double rightCurrentAmps = 0.0;
    public double leftCurrentAmps = 0.0;
    public double targetVelocityRPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
  public default void setVelocity(double rpm) {}
  public default void setPIDFF(double kP, double kV) {}
  public default void stopMotor() {}
}