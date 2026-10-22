package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.Servo;

public class IndexerIOReal implements IndexerIO {
  private final Servo actuator = new Servo(kIndexerPwmChannel);

  public IndexerIOReal() {
    actuator.setBoundsMicroseconds(kMaxPulseUs, 1500, 1500, 1500, kMinPulseUs);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.position = actuator.get();
    inputs.targetPosition = actuator.get();
  }

  @Override
  public void setPosition(double position) {
    actuator.set(position);
  }

  @Override
  public void stop() {
    actuator.setDisabled();
  }
}