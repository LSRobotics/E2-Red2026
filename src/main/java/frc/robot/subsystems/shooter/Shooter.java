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

  // Advantage Kit
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // Run it at target RPM 
  public Command runShooterCommand(double rpm) {
    return run(() -> io.setVelocity(rpm)).finallyDo(() -> io.stopMotor());
  }

  // Returns true if both flywheels are within tolerance of the target RPM. Could be used on shuffle board I think
  public boolean atSetpoint() {
    boolean rightAtSpeed = Math.abs(inputs.rightVelocityRPM - inputs.targetVelocityRPM) <= kShooterToleranceRPM;
    boolean leftAtSpeed = Math.abs(inputs.leftVelocityRPM - inputs.targetVelocityRPM) <= kShooterToleranceRPM;
  
    return inputs.targetVelocityRPM > 0.0 && rightAtSpeed && leftAtSpeed;
  }

  public void stop() {
    io.stopMotor();
  }
}