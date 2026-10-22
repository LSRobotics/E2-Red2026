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
  private static final double SIM_LOOP_INTERVAL = 0.020; // 20ms

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, kGearReduction);

  private final DCMotorSim rightSim = new DCMotorSim(plant, DCMotor.getNEO(1));
  private final DCMotorSim leftSim = new DCMotorSim(plant, DCMotor.getNEO(1));

  private final PIDController simPID = new PIDController(kP / 60.0, kI, kD, SIM_LOOP_INTERVAL);

  private double currentKv = kV;
  private double targetVelocityRPM = 0.0;
  private boolean isRunning = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double appliedVolts = 0.0;

    if (isRunning && targetVelocityRPM > 0.0) {
      double targetRPS = targetVelocityRPM / 60.0;
      double ffVolts = kS + (currentKv * targetRPS);
      double pidVolts = simPID.calculate(rightSim.getAngularVelocityRPM(), targetVelocityRPM);

      appliedVolts = Math.max(0.0, Math.min(12.0, ffVolts + pidVolts));
    }

    rightSim.setInputVoltage(appliedVolts);
    leftSim.setInputVoltage(-appliedVolts);

    rightSim.update(SIM_LOOP_INTERVAL);
    leftSim.update(SIM_LOOP_INTERVAL);

    inputs.rightMotorConnected = true;
    inputs.leftMotorConnected = true;
    inputs.rightAppliedVolts = rightSim.getInputVoltage();
    inputs.leftAppliedVolts = Math.abs(leftSim.getInputVoltage());
    inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
    inputs.leftVelocityRPM = Math.abs(leftSim.getAngularVelocityRPM());
    inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
    inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());
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
  public void setPIDFF(double kP, double kV) {
    this.simPID.setP(kP / 60.0);
    this.currentKv = kV;
  }

  @Override
  public void stopMotor() {
    this.targetVelocityRPM = 0.0;
    this.isRunning = false;
  }
}