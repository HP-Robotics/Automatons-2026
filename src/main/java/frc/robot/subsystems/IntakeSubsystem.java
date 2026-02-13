package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(MotorIDConstants.intakeSpinMotor);
    TalonFX m_leftMotor = new TalonFX(MotorIDConstants.intakeExtendLeftMotor);
    TalonFX m_rightMotor = new TalonFX(MotorIDConstants.intakeExtendRightMotor);
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeSubsystem");
    Slot0Configs m_intakeConfig = new Slot0Configs();

    final TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration()
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive));

    public IntakeSubsystem() {
        m_rightMotor.getConfigurator().apply(rightMotorConfigs);
        m_rightMotor.setControl(new Follower(MotorIDConstants.intakeExtendLeftMotor, MotorAlignmentValue.Opposed));
        m_intakeConfig.kP = IntakeConstants.kP;
        m_intakeConfig.kI = IntakeConstants.kI;
        m_intakeConfig.kD = IntakeConstants.kD;
    }

    public void periodic() {
        table.putValue(("intakeMotorValue"),
                NetworkTableValue.makeDouble(m_intakeMotor.getPosition().getValueAsDouble()));

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