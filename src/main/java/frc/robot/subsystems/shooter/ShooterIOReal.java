package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {
  private final SparkMax rightMotor = new SparkMax(kRightMotorCanId, MotorType.kBrushless);
  private final SparkMax leftMotor = new SparkMax(kLeftMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

  private final SparkClosedLoopController rightPID = rightMotor.getClosedLoopController();
  private final SparkClosedLoopController leftPID = leftMotor.getClosedLoopController();

  private final SparkMaxConfig config = new SparkMaxConfig();

  private final LoggedTunableNumber tunableP = new LoggedTunableNumber("Shooter/P", kP);
  private final LoggedTunableNumber tunableV = new LoggedTunableNumber("Shooter/V", kV);

  private double targetVelocityRPM = 0.0;

  public ShooterIOReal() {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(kCurrentLimitAmps);

    // Closed-loop gains (kV converted from RPS to RPM for REVLib)
    config.closedLoop
        .p(tunableP.get())
        .i(kI)
      .d(kD);
    config.closedLoop.feedForward.kV(tunableV.get() / 60.0);

    // CAN traffic optimization: slow down non-essential telemetry frames to 200ms
    config.signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(200)
        .busVoltagePeriodMs(200)
        .outputCurrentPeriodMs(200);

    // Apply config to right motor
    config.inverted(false);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Apply config to inverted left motor
    config.inverted(true);
    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rightMotorConnected = true;
    inputs.leftMotorConnected = true;

    inputs.rightVelocityRPM = Math.abs(rightEncoder.getVelocity());
    inputs.leftVelocityRPM = Math.abs(leftEncoder.getVelocity());

    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();

    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.targetVelocityRPM = targetVelocityRPM;

    checkTunableNumbers();
  }

  @Override
  public void setVelocity(double rpm) {
    targetVelocityRPM = rpm;
    rightPID.setSetpoint(rpm, ControlType.kVelocity);
    leftPID.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void stopMotor() {
    targetVelocityRPM = 0.0;
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }

  private void checkTunableNumbers() {
    if (tunableP.hasChanged(tunableP.hashCode()) || tunableV.hasChanged(tunableV.hashCode())) {
      config.closedLoop
          .p(tunableP.get());
        config.closedLoop.feedForward.kV(tunableV.get() / 60.0);

        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}