package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX topMotor = new TalonFX(kTopMotorCanId);
  private final TalonFX bottomMotor = new TalonFX(kBottomMotorCanId);

  // Phoenix 6 Control Request
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

  // Strongly-typed StatusSignals for Phoenix 6
  private final StatusSignal<Voltage> topVolts = topMotor.getMotorVoltage();
  private final StatusSignal<Voltage> bottomVolts = bottomMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> topVelocity = topMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomVelocity = bottomMotor.getVelocity();
  private final StatusSignal<Current> topAmps = topMotor.getSupplyCurrent();
  private final StatusSignal<Current> bottomAmps = bottomMotor.getSupplyCurrent();

  private double targetRPM = 0.0;

  public ShooterIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Slot 0 PID & Feedforward setup
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Top motor configuration
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topMotor.getConfigurator().apply(config);

    // Bottom motor configuration (Inverted for CCW spin)
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomMotor.getConfigurator().apply(config);

    // Optimize CAN bus update rate for AdvantageKit
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, topVolts, bottomVolts, topVelocity, bottomVelocity, topAmps, bottomAmps);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh signals over CAN
    BaseStatusSignal.refreshAll(
        topVolts, bottomVolts, topVelocity, bottomVelocity, topAmps, bottomAmps);

    inputs.topMotorConnected = topVolts.getStatus().isOK();
    inputs.bottomMotorConnected = bottomVolts.getStatus().isOK();

    // Extract numeric double values from measure objects
    inputs.topAppliedVolts = topVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = bottomVolts.getValueAsDouble();

    // Convert RPS (Rotations Per Second) to RPM
    inputs.topVelocityRPM = topVelocity.getValueAsDouble() * 60.0;
    inputs.bottomVelocityRPM = Math.abs(bottomVelocity.getValueAsDouble() * 60.0);

    inputs.targetVelocityRPM = targetRPM;
  }

  @Override
  public void setVelocity(double rpm) {
    this.targetRPM = rpm;
    double rps = rpm / 60.0; // Phoenix 6 expects Rotations Per Second (RPS)

    topMotor.setControl(velocityRequest.withVelocity(rps));
    bottomMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stopMotor() {
    this.targetRPM = 0.0;
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}