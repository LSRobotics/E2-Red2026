package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer() {
    this(RobotBase.isReal() ? new IndexerIOReal() : new IndexerIOSim());
  }

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Dashboard Telemetry
    SmartDashboard.putNumber("Indexer/Position", inputs.position);
    SmartDashboard.putNumber("Indexer/TargetPosition", inputs.targetPosition);
  }

  /** Command to extend the indexer */
  public Command extendCommand() {
    return runOnce(() -> io.setPosition(kExtendedPosition))
        .withName("ExtendIndexer");
  }

  /** Command to retract the indexer */
  public Command retractCommand() {
    return runOnce(() -> io.setPosition(kRetractedPosition))
        .withName("RetractIndexer");
  }

  public void stop() {
    io.stop();
  }
}