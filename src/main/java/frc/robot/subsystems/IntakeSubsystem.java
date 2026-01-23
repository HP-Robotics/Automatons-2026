package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(0); // TODO: make a constant for device ID

    public IntakeSubsystem() {

    }

    public void periodic() {

    }

    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void extendIntake() {
        // TODO: actaully write this function
    }

    public void retractIntake() {
        // TODO: actaully write this function
    }

    public Command Intake() {
        return new StartEndCommand(
                () -> {
                    this.runIntake(IntakeConstants.speed);
                    this.extendIntake();
                },
                () -> {
                    this.retractIntake();
                    this.stopIntake();
                }, this);
    }

}