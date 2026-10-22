package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double currentTargetRPM = kShootTargetRPM;

  /** Zero-argument constructor delegating hardware vs simulation selection */
  public Shooter() {
    this(RobotBase.isReal() ? new ShooterIOReal() : new ShooterIOSim());
  }

  /** Overloaded constructor for unit testing and dependency injection */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    boolean atSpeed = isAtSpeed();
    double avgActualRPM = (inputs.rightVelocityRPM + inputs.leftVelocityRPM) / 2.0;

    // AdvantageKit logging
    Logger.recordOutput("Shooter/TargetRPM", currentTargetRPM);
    Logger.recordOutput("Shooter/IsAtSpeed", atSpeed);
    Logger.recordOutput("Shooter/AvgActualRPM", avgActualRPM);

    // SmartDashboard display values
    SmartDashboard.putBoolean("Shooter/IsAtSpeed", atSpeed);
    SmartDashboard.putNumber("Shooter/TargetRPM", currentTargetRPM);
    SmartDashboard.putNumber("Shooter/RightActualRPM", inputs.rightVelocityRPM);
    SmartDashboard.putNumber("Shooter/LeftActualRPM", inputs.leftVelocityRPM);
    SmartDashboard.putNumber("Shooter/AvgActualRPM", avgActualRPM);
  }

  /** Runs velocity setpoint while held; automatically stops motors on release */
  public Command runShooterCommand() {
    return runEnd(
        () -> io.setVelocity(currentTargetRPM),
        () -> io.stopMotor()
    ).withName("RunShooter");
  }

  /** Resets target RPM back to the constant default (2000.0 RPM) */
  public Command resetRPMCommand() {
    return runOnce(() -> currentTargetRPM = kShootTargetRPM)
        .withName("ResetRPM");
  }

  /** Adjusts target RPM by percentage delta */
  public Command adjustRPMCommand(double deltaPercent) {
    return runOnce(() -> currentTargetRPM += currentTargetRPM * deltaPercent)
        .withName("AdjustRPM");
  }

  /** Checks if both dual flywheel wheels are within RPM tolerance */
  public boolean isAtSpeed() {
    return currentTargetRPM > 0.0
        && Math.abs(inputs.rightVelocityRPM - currentTargetRPM) <= kShooterToleranceRPM
        && Math.abs(inputs.leftVelocityRPM - currentTargetRPM) <= kShooterToleranceRPM;
  }

  public double getTargetRPM() {
    return currentTargetRPM;
  }

  public void stop() {
    io.stopMotor();
  }
}