// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Subsystems initializeds
  private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();
  // Closed-loop swerve request
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry m_telemetry = new Telemetry(MaxSpeed);
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    m_Swerve.registerTelemetry(m_telemetry::telemeterize); // Ensures swerve telemetry updates
    configureBindings();
  }

  private void configureBindings() {
  // ---------------- Shooter Controls ----------------
  // Hold R2 to run shooter
  m_driverController.rightTrigger(0.05)
      .whileTrue(m_shooter.runShooterCommand());

  // Tap L1 (Left Bumper) to decrease target RPM by 0.05%
  m_driverController.leftBumper()
      .onTrue(m_shooter.adjustRPMCommand(-kRpmAdjustmentPercent));

  // Tap R1 (Right Bumper) to increase target RPM by 0.05%
  m_driverController.rightBumper()
      .onTrue(m_shooter.adjustRPMCommand(kRpmAdjustmentPercent));

  // Press A to reset target RPM back to 2000.0 RPM
  m_driverController.a()
      .onTrue(m_shooter.resetRPMCommand());
  
  // ---------------- Indexer Controls ----------------
  // Extend indexer on X button press
  m_driverController.x()
      .onTrue(m_indexer.extendCommand());

  // Retract indexer on B button press
  m_driverController.b()
      .onTrue(m_indexer.retractCommand());
  // ---------------- Drivetrain Controls ----------------
  // Press Y to reset field-centric zero on the fly without stopping drive
  m_driverController.y()
      .onTrue(Commands.runOnce(() -> m_Swerve.seedFieldCentric()));

  // Default Swerve Request
  m_Swerve.setDefaultCommand(
      m_Swerve.applyRequest(() -> drive
          .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
          .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
          .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
      )
  );
}

  public Command getAutonomousCommand() {
    return Commands.idle();
  }

  public void periodic() {
    m_telemetry.telemeterize(m_Swerve.getState());
  }
}