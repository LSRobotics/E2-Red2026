package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double position = 0.0;
    public double targetPosition = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void stop() {}
}