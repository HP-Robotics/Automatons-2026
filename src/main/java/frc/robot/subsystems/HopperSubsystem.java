
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.MotorIDConstants;

public class HopperSubsystem extends SubsystemBase {
    TalonFX m_hopperMotor;
    TalonFX m_uplifterMotor;
    final TalonFXConfiguration m_uplifterMotorConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs()
                    .withKP(HopperConstants.uplifterkP)
                    .withKS(HopperConstants.uplifterkS)
                    .withKV(HopperConstants.uplifterkV));

    public HopperSubsystem() {
        m_hopperMotor = new TalonFX(MotorIDConstants.HopperMotorSpinner);
        m_uplifterMotor = new TalonFX(MotorIDConstants.HopperMotorUplifter);
        m_uplifterMotor.getConfigurator().apply(m_uplifterMotorConfigs);
    }

    public void runHopper(double spinnerSpeed, double uplifterSpeed) {
        // m_hopperMotor.set(spinnerSpeed);
        m_uplifterMotor.setControl(new VelocityTorqueCurrentFOC(uplifterSpeed));
    }

    public void stopHopper() {
        // m_hopperMotor.set(0);
        m_uplifterMotor.set(0);
    }

    public void periodic() {

    }

    public Command RunHopper() {
        return new StartEndCommand(
                () -> {
                    this.runHopper(HopperConstants.spinnerSpeed, HopperConstants.uplifterSpeed);
                },
                () -> {
                    this.stopHopper();
                }, this);
    }

}