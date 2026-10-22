package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {
  private double currentPosition = 0.0;
  private double targetPosition = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Simulate linear extension/retraction over time
    if (currentPosition < targetPosition) {
      currentPosition = Math.min(targetPosition, currentPosition + 0.05);
    } else if (currentPosition > targetPosition) {
      currentPosition = Math.max(targetPosition, currentPosition - 0.05);
    }

    inputs.position = currentPosition;
    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setPosition(double position) {
    this.targetPosition = position;
  }

  @Override
  public void stop() {
    this.targetPosition = currentPosition;
  }
}