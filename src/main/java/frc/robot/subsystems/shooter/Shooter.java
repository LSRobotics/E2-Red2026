package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {
  switch (Constants.currentMode) {
    case REAL:
      io = new ShooterIOReal();
      break;
    case SIM:
      io = new ShooterIOSim();
      break;
    default:
      io = new ShooterIO() {};
      break;
  }
}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // Command 1: Start spinning both motors at target RPM
  public Command startShootingCommand() {
    return runOnce(() -> io.setVelocity(ShooterConstants.kShootTargetRPM));
  }

  // Command 2: Stop both motors
  public Command stopShootingCommand() {
    return runOnce(() -> io.stopMotor());
  }
}