package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Commands the shooter flywheels to run at target RPM. */
  public Command runShooterCommand(double rpm) {
    return run(() -> io.setVelocity(rpm)).finallyDo(() -> io.stopMotor());
  }

  /** Returns true if both flywheels are within tolerance of the target RPM. */
  public boolean atSetpoint() {
    boolean topAtSpeed = Math.abs(inputs.topVelocityRPM - inputs.targetVelocityRPM) <= kShooterToleranceRPM;
    boolean bottomAtSpeed = Math.abs(inputs.bottomVelocityRPM - inputs.targetVelocityRPM) <= kShooterToleranceRPM;
  
    return inputs.targetVelocityRPM > 0.0 && topAtSpeed && bottomAtSpeed;
  }

  public void stop() {
    io.stopMotor();
  }
}