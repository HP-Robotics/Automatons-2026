package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDConstants;

public class HoodSubsystem extends SubsystemBase {
    double m_offset = 0;
    TalonFX m_hoodMotor = new TalonFX(MotorIDConstants.hoodMotor);
    boolean m_isCalibrated = false;
    StatusSignal<ReverseLimitValue> m_bottomLimit = m_hoodMotor.getReverseLimit();
    double m_targetPosition;
    double m_defaultPosition;

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("DraftHood");

    public HoodSubsystem() {
        double currentPosition = m_hoodMotor.getPosition().getValueAsDouble();
        m_defaultPosition = currentPosition + 0.01;
        m_targetPosition = m_defaultPosition;
        table.putValue("setTargetPosition", NetworkTableValue.makeDouble(m_defaultPosition));

        final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(currentPosition + 0.01)
                        .withForwardSoftLimitThreshold(
                                currentPosition + (HoodConstants.hoodTop - HoodConstants.hoodBottom)))
                .withSlot0(new Slot0Configs()
                        .withKP(HoodConstants.kP)
                        .withKD(HoodConstants.kD))
                .withMotorOutput(new MotorOutputConfigs()
                        .withPeakForwardDutyCycle(HoodConstants.maxSpeed)
                        .withPeakReverseDutyCycle(-HoodConstants.maxSpeed));
        m_hoodMotor.getConfigurator().apply(hoodMotorConfigs);
    }

    @Override
    public void periodic() {
        table.putValue("targetPosition", NetworkTableValue.makeDouble(m_targetPosition));
        table.putValue("position", NetworkTableValue.makeDouble(m_hoodMotor.getPosition().getValueAsDouble()));
    }

    public void getFromNetworkTables() {
        m_targetPosition = table.getEntry("setTargetPosition").getDouble(m_defaultPosition);
    }

    public void setHood(double position) {
        m_hoodMotor.setControl(new PositionDutyCycle(absoluteToRelative(position)));
    }

    public void hoodUp() {
        setHood(HoodConstants.hoodTop);
    }

    public void hoodDown() {
        setHood(HoodConstants.hoodBottom);
    }

    public void magicHood(double position) {
        setHood(absoluteToRelative(m_targetPosition));
    }

    public void networktablesHood(double position) {
        setHood(absoluteToRelative(m_targetPosition));
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
        return new RunCommand(
                () -> {
                    getFromNetworkTables();
                    magicHood(m_targetPosition);
                });
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
