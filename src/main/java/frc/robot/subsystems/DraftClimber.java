package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class DraftClimber extends SubsystemBase {
    TalonFX m_climberMotor = new TalonFX(MotorIDConstants.climberMotor);
    boolean m_isCalibrated = false;
    StatusSignal<ReverseLimitValue> m_bottomLimit = m_climberMotor.getReverseLimit();

    public DraftClimber() {

    }

    public void periodic() {

    }

    public void climberUp() {
        m_climberMotor.set(0);
    }

    public void climberDown() {
        m_climberMotor.set(0);
    }

    public Command Climb() {
        return new StartEndCommand(
        ()-> {
            this.climberUp();
        
        },
        ()-> {
            this.climberDown();
        },this);
        
    }
    //TODO:find which way is up

    public boolean isDown() {
        m_bottomLimit.refresh();
        if (m_bottomLimit.getValue() == ReverseLimitValue.ClosedToGround) {
            m_isCalibrated = true;
            return true;
        } else {
            return false;
        }
    }

}
