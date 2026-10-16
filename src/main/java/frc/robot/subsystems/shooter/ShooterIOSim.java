package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final double SIM_UPDATE_INTERVAL = 0.020; // 20ms loop cycle

  // Physics model using Neo Vortex / TalonFX motor properties
  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, kGearReduction);

  private final DCMotorSim rightSim = new DCMotorSim(plant, DCMotor.getNeoVortex(1));
  private final DCMotorSim leftSim = new DCMotorSim(plant, DCMotor.getNeoVortex(1));

  // Convert Phoenix 6 RPS kP to RPM units for WPILib PIDController
  private final PIDController simPID = new PIDController(kP / 60.0, kI, kD, SIM_UPDATE_INTERVAL);

  private double targetVelocityRPM = 0.0;
  private boolean isRunning = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double appliedVolts = 0.0;

    if (isRunning) {
      double targetRPS = targetVelocityRPM / 60.0;

      // Calculate Feedforward: kS + (kV * RPS) when spinning up
      double ffVolts = targetVelocityRPM > 0.0 ? (kS + (kV * targetRPS)) : 0.0;

      // PID calculates corrective or braking voltage
      double pidVolts = simPID.calculate(rightSim.getAngularVelocityRPM(), targetVelocityRPM);

      double totalVolts = ffVolts + pidVolts;

      // Clamp voltage between 0.0V and 12.0V so simulated wheels NEVER spin backward
      appliedVolts = Math.max(0.0, Math.min(12.0, totalVolts));
    }

    // Right motor spins positive (CW), Left motor spins inverted (CCW)
    rightSim.setInputVoltage(appliedVolts);
    leftSim.setInputVoltage(-appliedVolts);

    rightSim.update(SIM_UPDATE_INTERVAL);
    leftSim.update(SIM_UPDATE_INTERVAL);

    // Populate AdvantageKit logged inputs
    inputs.rightMotorConnected = true;
    inputs.leftMotorConnected = true;
    inputs.rightAppliedVolts = rightSim.getInputVoltage();
    inputs.leftAppliedVolts = leftSim.getInputVoltage();
    inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
    inputs.leftVelocityRPM = Math.abs(leftSim.getAngularVelocityRPM());
    inputs.targetVelocityRPM = targetVelocityRPM;
  }

  @Override
  public void setVelocity(double rpm) {
    if (!isRunning || this.targetVelocityRPM != rpm) {
      simPID.reset();
    }
    this.targetVelocityRPM = rpm;
    this.isRunning = true;
  }

  @Override
  public void stopMotor() {
    // Keep isRunning = true with target = 0.0 RPM so closed-loop braking decelerates the wheels,
    // while appliedVolts clamp at 0.0V prevents reverse spin.
    this.targetVelocityRPM = 0.0;
    this.isRunning = true;
  }
}
