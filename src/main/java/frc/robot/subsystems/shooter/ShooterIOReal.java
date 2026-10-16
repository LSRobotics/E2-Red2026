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
  private final TalonFX rightMotor = new TalonFX(kRightMotorCanId);
  private final TalonFX leftMotor = new TalonFX(kLeftMotorCanId);

  // Velocity control request explicitly set to Slot 0
  private final VelocityVoltage velocityRequest = 
      new VelocityVoltage(0.0)
          .withSlot(0)
          .withEnableFOC(false); // Set to true if using Phoenix Pro

  private final StatusSignal<Voltage> rightVolts = rightMotor.getMotorVoltage();
  private final StatusSignal<Voltage> leftVolts = leftMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
  private final StatusSignal<AngularVelocity> leftVelocity = leftMotor.getVelocity();
  private final StatusSignal<Current> rightAmps = rightMotor.getSupplyCurrent();
  private final StatusSignal<Current> leftAmps = leftMotor.getSupplyCurrent();

  private double targetRPM = 0.0;

  public ShooterIOReal() {
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    // Slot 0 Closed-Loop Gains
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kS = kS;

    rightConfig.Slot0 = slot0;
    leftConfig.Slot0 = slot0;

    // Set Neutral Mode to Brake for active deceleration
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Safety: Clamp Peak Reverse Voltage to 0.0V so wheels NEVER spin backward
    rightConfig.Voltage.PeakReverseVoltage = 0.0;
    leftConfig.Voltage.PeakReverseVoltage = 0.0;

    // Inversion configurations cuz one wheel spins clockwise and the other counter-clockwise to shoot straight
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply configs to TalonFX motor controllers cuz they don't have a default config
    var statusRight = rightMotor.getConfigurator().apply(rightConfig, 0.050);
    var statusLeft = leftMotor.getConfigurator().apply(leftConfig, 0.050);


    if (!statusRight.isOK()) {
      System.err.println("Right Motor Config Error: " + statusRight.toString());
    }
    if (!statusLeft.isOK()) {
      System.err.println("Left Motor Config Error: " + statusLeft.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rightVolts, leftVolts, rightVelocity, leftVelocity, rightAmps, leftAmps);
  }

  // Advantage Kit
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rightVolts, leftVolts, rightVelocity, leftVelocity, rightAmps, leftAmps);

    inputs.rightMotorConnected = rightVolts.getStatus().isOK();
    inputs.leftMotorConnected = leftVolts.getStatus().isOK();

    inputs.rightAppliedVolts = rightVolts.getValueAsDouble();
    inputs.leftAppliedVolts = leftVolts.getValueAsDouble();

    // Convert RPS (Rotations Per Second) to RPM
    inputs.rightVelocityRPM = Math.abs(rightVelocity.getValueAsDouble() * 60.0);
    inputs.leftVelocityRPM = Math.abs(leftVelocity.getValueAsDouble() * 60.0);

    inputs.targetVelocityRPM = targetRPM;
  }
  // Advantage Kit
  @Override
  public void setVelocity(double rpm) {
    this.targetRPM = rpm;
    double rps = rpm / 60.0; // Convert RPM to RPS for Phoenix 6

    rightMotor.setControl(velocityRequest.withVelocity(rps));
    leftMotor.setControl(velocityRequest.withVelocity(rps));
  }
  // Advantage Kit
  @Override
  public void stopMotor() {
    this.targetRPM = 0.0;
    // Request 0 RPS so motors use Brake mode down to 0, but PeakReverseVoltage prevents backward spin
    rightMotor.setControl(velocityRequest.withVelocity(0.0));
    leftMotor.setControl(velocityRequest.withVelocity(0.0));
  }
}