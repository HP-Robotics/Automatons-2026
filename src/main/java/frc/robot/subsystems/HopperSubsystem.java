
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.MotorIDConstants;

public class HopperSubsystem extends SubsystemBase {
    TalonFX m_hopperMotor;
    TalonFX m_outakeMotor;
    final TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive));

    public HopperSubsystem() {
        // m_hopperMotor = new TalonFX(MotorIDConstants.HopperMotorSpinner);
        m_outakeMotor = new TalonFX(MotorIDConstants.HopperMotorOutake);
        m_outakeMotor.getConfigurator().apply(rightMotorConfigs);
    }

    public void runHopper(double spinnerSpeed, double outakeSpeed) {
        // m_hopperMotor.set(spinnerSpeed);
        m_outakeMotor.set(outakeSpeed);
    }

    public void stopHopper() {
        // m_hopperMotor.set(0);
        m_outakeMotor.set(0);
    }

    public void periodic() {

    }

    public Command RunHopper() {
        return new StartEndCommand(
                () -> {
                    this.runHopper(HopperConstants.spinnerSpeed, HopperConstants.outakeSpeed);
                },
                () -> {
                    this.stopHopper();
                }, this);
    }

}

// TODO: make it start/stop using a button (DONE?)