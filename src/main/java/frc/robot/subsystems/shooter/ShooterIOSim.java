package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final double GEAR_REDUCTION = 1.0;
  private static final double SIM_UPDATE_INTERVAL = 0.020; // 20ms loop cycle

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, GEAR_REDUCTION);

  private final DCMotorSim topSim = new DCMotorSim(plant, DCMotor.getNeoVortex(1));
  private final DCMotorSim bottomSim = new DCMotorSim(plant, DCMotor.getNeoVortex(1));

  private final PIDController controller = new PIDController(0.001, 0.0, 0.0, SIM_UPDATE_INTERVAL);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.002, 0.0);

  private double targetVelocityRPM = 0.0;
  private boolean isRunning = false;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double appliedVolts = 0.0;

    if (isRunning && targetVelocityRPM > 0.0) {
      double pid = controller.calculate(topSim.getAngularVelocityRPM(), targetVelocityRPM);
      double ff = feedforward.calculate(targetVelocityRPM);
      appliedVolts = Math.max(-12.0, Math.min(12.0, pid + ff));
    }

    // Top motor spins positive (CW), Bottom motor spins inverted negative (CCW)
    topSim.setInputVoltage(appliedVolts);
    bottomSim.setInputVoltage(-appliedVolts);

    topSim.update(SIM_UPDATE_INTERVAL);
    bottomSim.update(SIM_UPDATE_INTERVAL);

    // Populate AdvantageKit logged inputs
    inputs.topMotorConnected = true;
    inputs.bottomMotorConnected = true;
    inputs.topAppliedVolts = topSim.getInputVoltage();
    inputs.bottomAppliedVolts = bottomSim.getInputVoltage();
    inputs.topVelocityRPM = topSim.getAngularVelocityRPM();
    inputs.bottomVelocityRPM = Math.abs(bottomSim.getAngularVelocityRPM());
    inputs.targetVelocityRPM = targetVelocityRPM;
  }

  @Override
  public void setVelocity(double rpm) {
    this.targetVelocityRPM = rpm;
    this.isRunning = true;
  }

  @Override
  public void stopMotor() {
    this.isRunning = false;
    this.targetVelocityRPM = 0.0;
    topSim.setInputVoltage(0.0);
    bottomSim.setInputVoltage(0.0);
  }
}