package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        // m_rightMotor.setControl(new Follower(MotorIDConstants.intakeExtendLeftMotor,
        // MotorAlignmentValue.Aligned));
        m_intakeConfig.kP = IntakeConstants.kP;
        m_intakeConfig.kI = IntakeConstants.kI;
        m_intakeConfig.kD = IntakeConstants.kD;
        m_rightMotor.setPosition(0);
        m_leftMotor.setPosition(0);

        var leftMotorConfigs = new TalonFXConfiguration();
        var rightMotorConfigs = new TalonFXConfiguration();

        var leftSlot0Configs = leftMotorConfigs.Slot0;
        leftSlot0Configs.kS = IntakeConstants.leftkS; // Add 0.25 V output to overcome static friction
        leftSlot0Configs.kV = IntakeConstants.leftkV; // A velocity target of 1 rps results in 0.12 V output
        leftSlot0Configs.kA = IntakeConstants.leftkA; // An acceleration of 1 rps/s requires 0.01 V output
        leftSlot0Configs.kP = IntakeConstants.leftkP; // A position error of 2.5 rotations results in 12 V output
        leftSlot0Configs.kI = IntakeConstants.leftkI; // no output for integrated error
        leftSlot0Configs.kD = IntakeConstants.leftkD; // A velocity error of 1 rps results in 0.1 V output

        var leftMotionMagicConfigs = leftMotorConfigs.MotionMagic;
        leftMotionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        leftMotionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        leftMotionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_leftMotor.getConfigurator().apply(leftMotorConfigs);

        var rightSlot0Configs = rightMotorConfigs.Slot0;
        rightSlot0Configs.kS = IntakeConstants.rightkS; // Add 0.25 V output to overcome static friction
        rightSlot0Configs.kV = IntakeConstants.rightkV; // A velocity target of 1 rps results in 0.12 V output
        rightSlot0Configs.kA = IntakeConstants.rightkA; // An acceleration of 1 rps/s requires 0.01 V output
        rightSlot0Configs.kP = IntakeConstants.rightkP; // A position error of 2.5 rotations results in 12 V output
        rightSlot0Configs.kI = IntakeConstants.rightkI; // no output for integrated error
        rightSlot0Configs.kD = IntakeConstants.rightkD; // A velocity error of 1 rps results in 0.1 V output

        var rightMotionMagicConfigs = rightMotorConfigs.MotionMagic;
        rightMotionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        rightMotionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        rightMotionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_rightMotor.getConfigurator().apply(rightMotorConfigs);
    }

    public void periodic() {
        table.putValue(("intakeMotorValue"),
                NetworkTableValue.makeDouble(m_intakeMotor.getPosition().getValueAsDouble()));
        table.putValue(("leftMotorValue"),
                NetworkTableValue.makeDouble(m_leftMotor.getPosition().getValueAsDouble()));
        table.putValue(("rightMotorValue"),
                NetworkTableValue.makeDouble(m_rightMotor.getPosition().getValueAsDouble()));
    }

    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void extendIntake() {
        final MotionMagicVoltage m_leftRequest = new MotionMagicVoltage(0);
        m_leftMotor.setControl(m_leftRequest.withPosition(IntakeConstants.leftExtendPosition));
        final MotionMagicVoltage m_rightRequest = new MotionMagicVoltage(0);
        m_rightMotor.setControl(m_rightRequest.withPosition(IntakeConstants.rightExtendPosition));
    }

    public void retractIntake() {
        final MotionMagicVoltage m_leftRequest = new MotionMagicVoltage(0);
        m_leftMotor.setControl(m_leftRequest.withPosition(IntakeConstants.leftRetractPosition));
        final MotionMagicVoltage m_rightRequest = new MotionMagicVoltage(0);
        m_rightMotor.setControl(m_rightRequest.withPosition(IntakeConstants.rightRetractPosition));
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