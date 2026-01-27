// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  TalonFX turretMotor = new TalonFX(MotorIDConstants.turretMotor);
  double turretSpeed = TurretConstants.turretSpeed;
  double targetPosition;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    targetPosition = TurretConstants.calibrationPosition;
  }

  public void runTurret() {
    turretMotor.set(turretSpeed);
  }

  public void setTargetPosition(double position) {
    this.targetPosition = position;
  }

  public Command RotateTurret() {
    return new RunCommand(() -> turretMotor.setControl(new PositionDutyCycle(targetPosition)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
