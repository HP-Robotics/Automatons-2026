package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDConstants;

public class DraftHood extends SubsystemBase {
    double m_offset = 0;
    TalonFX m_hoodMotor = new TalonFX(MotorIDConstants.hoodMotor);
    boolean m_isCalibrated = false;
    StatusSignal<ReverseLimitValue> m_bottomLimit = m_hoodMotor.getReverseLimit();
    double m_targetPosition;

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("DraftHood");

    @Override
    public void periodic() {
        table.putValue("targetPosition", NetworkTableValue.makeDouble(m_targetPosition));
    }

    public void setHood(double position) {
        m_hoodMotor.setControl(new PositionVoltage(absoluteToRelative(position)));
    }

    public void hoodUp() {
        setHood(HoodConstants.hoodTop);
    }

    public void hoodDown() {
        setHood(HoodConstants.hoodBottom);
    }

    public void magicHood(double position) {
        setHood(m_targetPosition);
    }

    public void networktablesHood(double position) {
        setHood(m_targetPosition);
    }

    public double absoluteToRelative(double absolute) {
        return (absolute) + m_offset;
    }
    // relative is the actual command that we send to the motor

    public double relativeToAbsolute(double motorTicks) {
        return (motorTicks - m_offset);
    }

    public Command setHoodPosition(double position) {
        return new InstantCommand(
                () -> {
                    this.setHood(position);
                }, this);
    }

    public Command Calibrate() {
        return new StartEndCommand(() -> hoodUp(), () -> hoodDown()).until(() -> isDown())
                .finallyDo(() -> m_offset = m_hoodMotor.getRotorPosition().getValueAsDouble());

    }

    public Command hoodFromMagic() {
        return new InstantCommand(
                () -> magicHood(m_targetPosition));

    }

    public Command hoodFromNetworkTables() {
        return new InstantCommand(
                () -> magicHood(m_targetPosition));
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
