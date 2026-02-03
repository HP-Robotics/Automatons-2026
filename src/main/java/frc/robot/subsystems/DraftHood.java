package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class DraftHood extends SubsystemBase {
    TalonFX m_hoodMotor = new TalonFX(MotorIDConstants.hoodMotor);

    public void setHood(double position) {
        m_hoodMotor.setControl(new PositionVoltage(0).withPosition(position));
    }

    public double absoluteToRelative(double absolute) {
        return (absolute) + m_offset;
    }
    // relative is the actual command that we send to the motor

    public double relativeToAbsolute(double motorTicks) {
        return (motorTicks - m_offset);
    }

    public Command setHoodPosition() {
        return new InstantCommand(
                () -> {
                    this.setHood();
                }, this);
    }

    public Command Calibrate() {
        return new StartEndCommand(() -> climberUp(), () -> climberDown()).until(() -> isDown())
                .finallyDo(() -> m_offset = m_hoodMotor.getRotorPosition().getValueAsDouble());

    }

    public boolean isDown() {
        m_bottomLimit.refresh();
        if (m_bottomLimit.getValue() == ReverseLimitValue.ClosedToGround) {
            m_isCalibrated = true;
            return true;
        } else {
            return false;

        }
    }

    public void resetMotorEncoders() {
        if (isDown()) {
            m_offset = m_hoodMotor.getRotorPosition().getValueAsDouble();
        }
    }
}
