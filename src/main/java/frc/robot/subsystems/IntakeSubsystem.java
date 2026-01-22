package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX motor = new TalonFX(0);

    public IntakeSubsystem() {

    }

    public void periodic() {

    }

    public void runIntake(double speed) {
        motor.set(speed);
    }

    public void stopIntake() {
        motor.set(0);
    }

    public void extendIntake() {
        // TODO: actaully write this function
    }

    public void retractIntake() {
        // TODO: actaully write this function
    }

    public Command intake() {
        return new StartEndCommand(
                () -> {
                    this.runIntake(1);
                    this.extendIntake();
                },
                () -> {
                    this.retractIntake();
                    this.stopIntake();
                }, this);
    }

}