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

public class ShooterIOReal implements ShooterIO {
  private final SparkMax rightMotor = new SparkMax(kRightMotorCanId, MotorType.kBrushless);
  private final SparkMax leftMotor = new SparkMax(kLeftMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

  private final SparkClosedLoopController rightPID = rightMotor.getClosedLoopController();
  private final SparkClosedLoopController leftPID = leftMotor.getClosedLoopController();

  private double targetVelocityRPM = 0.0;

  public ShooterIOReal() {
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.smartCurrentLimit(kCurrentLimitAmps);
    rightConfig.voltageCompensation(12);

    rightConfig.closedLoop
        .p(kP)
        .i(kI)
      .d(kD);
    rightConfig.closedLoop.feedForward.kV(kV / 60.0);
    rightConfig.signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    rightConfig.inverted(false);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(kCurrentLimitAmps);
    leftConfig.voltageCompensation(12.0);

    leftConfig.closedLoop
        .p(kP)
        .i(kI)
      .d(kD);
    leftConfig.closedLoop.feedForward.kV(kV / 60.0);
    leftConfig.signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leftConfig.inverted(true);

    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
  }

  @Override
  public void setVelocity(double rpm) {
    targetVelocityRPM = rpm;
    rightPID.setSetpoint(rpm, ControlType.kVelocity);
    leftPID.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setPIDFF(double kP, double kV) {
    SparkMaxConfig updateConfig = new SparkMaxConfig();
    updateConfig.closedLoop.p(kP);
    updateConfig.closedLoop.feedForward.kV(kV / 60.0);

    rightMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void stopMotor() {
    targetVelocityRPM = 0.0;
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }
}