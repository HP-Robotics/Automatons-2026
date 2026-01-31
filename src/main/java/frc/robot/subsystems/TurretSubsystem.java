// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  TalonFX m_turretMotor = new TalonFX(MotorIDConstants.turretMotor);
  double turretSpeed = TurretConstants.turretSpeed;
  double m_targetPosition; // in robot relative degrees
  NetworkTable m_table = NetworkTableInstance.getDefault().getTable("TurretSubsystem");
  double m_offset = 0;
  StatusSignal<ReverseLimitValue> m_limit = m_turretMotor.getReverseLimit(); // TODO: is it reversed?
  DigitalInput m_limitInput = new DigitalInput(PortConstants.turretLimitPort);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_targetPosition = TurretConstants.calibrationPosition;
  }

  public void runTurret() {
    m_turretMotor.set(turretSpeed);
  }

  public void stopTurret() {
    m_turretMotor.set(0);
  }

  public void setTargetPosition(double position) {
    m_targetPosition = position;
  }

  public void getFromNetworkTables() {
    this.turretSpeed = m_table.getEntry("turretSpeed").getDouble(TurretConstants.turretSpeed);
    this.m_targetPosition = m_table.getEntry("turretTargetPosition").getDouble(m_targetPosition);
  }

  public boolean atPosition() {
    return (Math.abs(m_turretMotor.getRotorPosition().getValueAsDouble()
        - m_targetPosition) <= TurretConstants.errorTolerance);
    // return (turretMotor.getControlMode())
  }

  public boolean atLimit() {
    return !m_limitInput.get(); // flipped because this limit switch returns false when hit
  }

  public boolean flip() { // calculate if we're in the overlap, if we're nearer the top or bottom limit,
                          // and which version of target position we're closest to --> flip
    if (m_targetPosition % 360 > TurretConstants.bottomLimitPosition
        || m_targetPosition % 360 < TurretConstants.topLimitPosition) {
      return false; // TODO: might need to invert motor if turret is oriented the
                    // opposite way
    }
    double fullRotationDegrees = (Math.abs(m_targetPosition - TurretConstants.bottomLimitPosition) < Math
        .abs(m_targetPosition - TurretConstants.topLimitPosition)) ? 360 : -360;
    return (Math.abs(m_targetPosition - m_turretMotor.getPosition().getValueAsDouble()) > Math
        .abs((m_targetPosition + fullRotationDegrees) - m_turretMotor.getPosition().getValueAsDouble())
        && ((m_targetPosition + fullRotationDegrees) < (m_offset + TurretConstants.distanceToLimitThreshold)));
  }

  public Command RotateTurret() {
    m_targetPosition = flip() ? m_targetPosition + 360 : m_targetPosition; // will get targetPosition from aiming math
    return new RunCommand(() -> {
      var target = new PositionVoltage(0).withPosition(degreesToMotorTicks(m_targetPosition));
      m_turretMotor.setControl(target);
    });
  }

  public double degreesToMotorTicks(double degrees) {
    return (degrees * TurretConstants.encoderCPR * TurretConstants.gearRatio / 360) + m_offset;
  }

  public double motorTicksToDegrees(double motorTicks) {
    return 360 * (motorTicks - m_offset) / (TurretConstants.encoderCPR * TurretConstants.gearRatio);
  }

  public Command Calibrate() { // turns turret until we hit the limit switch, then sets the offset to the motor
                               // position
    return new StartEndCommand(() -> runTurret(), () -> stopTurret()).until(() -> atLimit())
        .finallyDo(() -> resetMotorEncoders());
  } // -> m_offset = atLimit() ? m_turretMotor.getRotorPosition().getValueAsDouble()
    // : m_offset

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getFromNetworkTables();
    m_table.putValue("limitSwitchOn", NetworkTableValue.makeBoolean(atLimit()));
    m_table.putValue("motorPosition", NetworkTableValue.makeDouble(m_turretMotor.getPosition().getValueAsDouble()));
    m_table.putValue("turretDegrees",
        NetworkTableValue.makeDouble(motorTicksToDegrees(m_turretMotor.getPosition().getValueAsDouble())));

  }

  public void resetMotorEncoders() {
    if (atLimit()) {
      m_offset = m_turretMotor.getRotorPosition().getValueAsDouble();
    }
  }
}
