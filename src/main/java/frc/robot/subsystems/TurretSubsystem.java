// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  TalonFX m_turretMotor = new TalonFX(MotorIDConstants.turretMotor);
  double m_turretSpeed = TurretConstants.turretSpeed;
  private double m_targetPosition; // in robot relative degrees
  NetworkTable m_table = NetworkTableInstance.getDefault().getTable("TurretSubsystem");
  double m_offset = 0;
  StatusSignal<ReverseLimitValue> m_limit = m_turretMotor.getReverseLimit();
  DigitalInput m_limitInput = new DigitalInput(PortConstants.turretLimitPort);
  Slot0Configs m_turretConfig = new Slot0Configs();
  MotorOutputConfigs m_turretOutputConfig = new MotorOutputConfigs();
  public static boolean m_isCalibrated;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_targetPosition = TurretConstants.calibrationPosition;
    m_table.putValue("targetPosition", NetworkTableValue.makeDouble(m_targetPosition));
    m_turretConfig.kP = TurretConstants.kP;
    m_turretConfig.kI = TurretConstants.kI;
    m_turretConfig.kD = TurretConstants.kD;
    m_turretOutputConfig.PeakForwardDutyCycle = TurretConstants.maxForwardDutyCycle;
    m_turretOutputConfig.PeakReverseDutyCycle = TurretConstants.maxReverseDutyCycle;
    m_isCalibrated = false;
  }

  public void runTurret() {
    m_turretMotor.setControl(new DutyCycleOut(m_turretSpeed));
  }

  public void stopTurret() {
    m_turretMotor.setControl(new DutyCycleOut(0));
  }

  // ONLY CHANGE M_TARGETPOSITION THROUGH THIS
  // this is in robot relative degrees
  public void setTargetPosition(double position) {
    m_targetPosition = position;
    while (m_targetPosition >= TurretConstants.topLimitPosition) {
      m_targetPosition -= 360;
    }
    while (m_targetPosition <= TurretConstants.bottomLimitPosition) {
      m_targetPosition += 360;
    }
    m_targetPosition = flipToNewTarget(); // will get targetPosition from aiming math
    if (m_targetPosition > TurretConstants.topLimitPosition) {
      m_targetPosition = TurretConstants.topLimitPosition - TurretConstants.distanceToLimitThreshold;
    } else if (m_targetPosition < TurretConstants.bottomLimitPosition) {
      m_targetPosition = TurretConstants.bottomLimitPosition + TurretConstants.distanceToLimitThreshold;
    }
  }

  public void getFromNetworkTables() {
    this.m_turretSpeed = m_table.getEntry("turretSpeed").getDouble(TurretConstants.turretSpeed);
    setTargetPosition(m_table.getEntry("targetPosition").getDouble(m_targetPosition));
  }

  public boolean atPosition() {
    return (Math.abs(m_turretMotor.getRotorPosition().getValueAsDouble()
        - m_targetPosition) <= TurretConstants.errorTolerance);
    // return (turretMotor.getControlMode())
  }

  public boolean atLimit() {
    return !m_limitInput.get(); // flipped because this limit switch returns false when hit
  }

  public Angle getAngle() {
    return Degrees.of(motorTicksToDegrees(m_turretMotor.getPosition().getValueAsDouble()));
  }

  public double flipToNewTarget() { // calculate if we're in the overlap, if we're nearer the top or bottom limit,
    // and which version of target position we're closest to --> flip
    if (m_targetPosition - 360 < TurretConstants.bottomLimitPosition
        && m_targetPosition + 360 > TurretConstants.topLimitPosition) {
      return m_targetPosition; // TODO: might need to invert motor if turret is oriented the
      // opposite way
    }
    double fullRotationDegrees = (Math.abs(m_targetPosition - TurretConstants.bottomLimitPosition) < Math
        .abs(m_targetPosition - TurretConstants.topLimitPosition)) ? 360 : -360;
    m_table.putValue("fullRotationDegrees", NetworkTableValue.makeDouble(fullRotationDegrees));
    if (Math.abs(m_targetPosition - motorTicksToDegrees(m_turretMotor.getPosition().getValueAsDouble())) > Math
        .abs((m_targetPosition + fullRotationDegrees)
            - motorTicksToDegrees(m_turretMotor.getPosition().getValueAsDouble()))) {
      return m_targetPosition + fullRotationDegrees;
    }
    return m_targetPosition;
  }

  public Command RotateTurret() {
    if (m_isCalibrated) {
      return new RunCommand(() -> {
        rotateTurretToTarget();
      });
    } else {
      return new WaitCommand(0);
    }
  }

  public void rotateTurretToTarget() {
    var target = new PositionDutyCycle(degreesToMotorTicks(m_targetPosition));
    m_turretMotor.setControl(target);
  }

  public double getQuotient(double x, double modulo) {
    double remainder = x % modulo;
    return remainder > 0 ? x - remainder : x - (remainder + 1);
  }

  public double degreesToMotorTicks(double degrees) {
    return ((degrees - TurretConstants.calibrationDegrees) * TurretConstants.encoderCPR * TurretConstants.gearRatio
        / 360) + m_offset;
  }

  public double motorTicksToDegrees(double motorTicks) {
    return 360 * (motorTicks - m_offset) / (TurretConstants.encoderCPR * TurretConstants.gearRatio)
        + (TurretConstants.calibrationDegrees);
  }

  // TODO: if we don't hit the limit switch and our current rises, we need to
  // assume we messed up and reverse until we hit the limit switch, and stop after
  // the limit switch turns off
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
    m_table.putValue("realTargetPosition", NetworkTableValue.makeDouble(m_targetPosition));

  }

  public void resetMotorEncoders() {
    if (atLimit()) {
      m_offset = -(getQuotient(m_turretMotor.getRotorPosition().getValueAsDouble(), 1)
          - getQuotient(TurretConstants.limitSwitchMotorPosition, 1)) - TurretConstants.calibrationPosition;
      m_isCalibrated = true;
    }
  }
}
