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
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.climberConstants;

public class DraftClimber extends SubsystemBase {
    TalonFX m_climberMotor = new TalonFX(MotorIDConstants.climberMotor);
    boolean m_isCalibrated = false;
    StatusSignal<ReverseLimitValue> m_bottomLimit = m_climberMotor.getReverseLimit();
    double m_offset = 0;
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("DraftClimber");

    public DraftClimber() {

    }

    public void periodic() {
        table.putValue("limitValue", NetworkTableValue.makeBoolean(isDown()));
        table.putValue("relativeValue",
                NetworkTableValue.makeDouble(m_climberMotor.getRotorPosition().getValueAsDouble()));
        table.putValue("absoluteValue",
                NetworkTableValue.makeDouble(relativeToAbsolute(m_climberMotor.getRotorPosition().getValueAsDouble())));
        table.putValue("m_offset", NetworkTableValue.makeDouble(m_offset));
    }

    public void climberUp() {
        m_climberMotor.setControl(new PositionVoltage(0).withPosition(climberConstants.climberTopPosition));
    }

    public void climberDown() {
        m_climberMotor.setControl(new PositionVoltage(0).withPosition(climberConstants.climberBottomPosition));
    }

    public Command Climb() {
        return new StartEndCommand(
                () -> {
                    this.climberUp();

                },
                () -> {
                    this.climberDown();
                }, this);

    }

    public Command ClimbUp() {
        return new InstantCommand(
                () -> {
                    this.climberUp();
                }, this);
    }

     public Command ClimbDown() {
        return new InstantCommand(
                () -> {
                    this.climberDown();
                }, this);
    }

    

    // TODO:find which way is up

    public double absoluteToRelative(double absolute) {
        return (absolute) + m_offset;
    }
    // relative is the actual command that we send to the motor

    public double relativeToAbsolute(double motorTicks) {
        return (motorTicks - m_offset);
    }

    public Command Calibrate() {
        return new StartEndCommand(() -> climberUp(), () -> climberDown()).until(() -> isDown())
                .finallyDo(() -> m_offset = m_climberMotor.getRotorPosition().getValueAsDouble());

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
            m_offset = m_climberMotor.getRotorPosition().getValueAsDouble();
        }
    }

}
