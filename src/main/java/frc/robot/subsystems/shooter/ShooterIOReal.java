package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX topMotor = new TalonFX(kTopMotorCanId);
  private final TalonFX bottomMotor = new TalonFX(kBottomMotorCanId);

  // Velocity control request explicitly set to Slot 0
  private final VelocityVoltage velocityRequest = 
      new VelocityVoltage(0.0)
          .withSlot(0)
          .withEnableFOC(false); // Set to true if using Phoenix Pro

  private final StatusSignal<Voltage> topVolts = topMotor.getMotorVoltage();
  private final StatusSignal<Voltage> bottomVolts = bottomMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> topVelocity = topMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomVelocity = bottomMotor.getVelocity();
  private final StatusSignal<Current> topAmps = topMotor.getSupplyCurrent();
  private final StatusSignal<Current> bottomAmps = bottomMotor.getSupplyCurrent();

  private double targetRPM = 0.0;

  public ShooterIOReal() {
    TalonFXConfiguration topConfig = new TalonFXConfiguration();
    TalonFXConfiguration bottomConfig = new TalonFXConfiguration();

    // Slot 0 Closed-Loop Gains
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kS = kS;

    topConfig.Slot0 = slot0;
    bottomConfig.Slot0 = slot0;

    // Set Neutral Mode to Brake for active deceleration
    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Safety: Clamp Peak Reverse Voltage to 0.0V so wheels NEVER spin backward
    topConfig.Voltage.PeakReverseVoltage = 0.0;
    bottomConfig.Voltage.PeakReverseVoltage = 0.0;

    // Inversion configurations cuz one wheel spins clockwise and the other counter-clockwise to shoot straight
    topConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply configs to TalonFX motor controllers cuz they don't have a default config
    var statusTop = topMotor.getConfigurator().apply(topConfig, 0.050);
    var statusBottom = bottomMotor.getConfigurator().apply(bottomConfig, 0.050);


    if (!statusTop.isOK()) {
      System.err.println("Top Motor Config Error: " + statusTop.toString());
    }
    if (!statusBottom.isOK()) {
      System.err.println("Bottom Motor Config Error: " + statusBottom.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, topVolts, bottomVolts, topVelocity, bottomVelocity, topAmps, bottomAmps);
  }

  // Advantage Kit
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topVolts, bottomVolts, topVelocity, bottomVelocity, topAmps, bottomAmps);

    inputs.topMotorConnected = topVolts.getStatus().isOK();
    inputs.bottomMotorConnected = bottomVolts.getStatus().isOK();

    inputs.topAppliedVolts = topVolts.getValueAsDouble();
    inputs.bottomAppliedVolts = bottomVolts.getValueAsDouble();

    // Convert RPS (Rotations Per Second) to RPM
    inputs.topVelocityRPM = Math.abs(topVelocity.getValueAsDouble() * 60.0);
    inputs.bottomVelocityRPM = Math.abs(bottomVelocity.getValueAsDouble() * 60.0);

    inputs.targetVelocityRPM = targetRPM;
  }
  // Advantage Kit
  @Override
  public void setVelocity(double rpm) {
    this.targetRPM = rpm;
    double rps = rpm / 60.0; // Convert RPM to RPS for Phoenix 6

    topMotor.setControl(velocityRequest.withVelocity(rps));
    bottomMotor.setControl(velocityRequest.withVelocity(rps));
  }
  // Advantage Kit
  @Override
  public void stopMotor() {
    this.targetRPM = 0.0;
    // Request 0 RPS so motors use Brake mode down to 0, but PeakReverseVoltage prevents backward spin
    topMotor.setControl(velocityRequest.withVelocity(0.0));
    bottomMotor.setControl(velocityRequest.withVelocity(0.0));
  }
}