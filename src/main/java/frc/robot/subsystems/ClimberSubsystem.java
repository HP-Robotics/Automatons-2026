package frc.robot.subsystems;

import org.opencv.dnn.Net;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX m_climberMotor = new TalonFX(MotorIDConstants.climberMotor);
    boolean m_isCalibrated = false;
    public NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Climber");
    Slot0Configs m_climberConfig = new Slot0Configs();
    MotorOutputConfigs m_climberConfigs = new MotorOutputConfigs();
    Timer m_timer = new Timer();

    public ClimberSubsystem() {
        m_climberConfig.kP = ClimberConstants.kP;
        m_climberConfig.kI = ClimberConstants.kI;
        m_climberConfig.kD = ClimberConstants.kD;
        m_climberConfig.kG = ClimberConstants.kG;

        m_climberConfigs.NeutralMode = NeutralModeValue.Brake;

        m_climberMotor.getConfigurator().apply(m_climberConfig);
        m_climberMotor.getConfigurator().apply(m_climberConfigs);
        // TODO: get the calibration to happen at robot boot
    }

    public void periodic() {
        // m_table.putValue("position",
        // NetworkTableValue.makeDouble(m_climberMotor.getRotorPosition().getValueAsDouble()));
        // m_table.putValue("velocity",
        // NetworkTableValue.makeDouble(m_climberMotor.getVelocity().getValueAsDouble()));
        // m_table.putValue("isDown", NetworkTableValue.makeBoolean(isDown()));
        // m_table.putValue("isCalibrated",
        // NetworkTableValue.makeBoolean(m_isCalibrated));
        // m_table.putValue("DutyCycle",
        // NetworkTableValue.makeDouble(m_climberMotor.getDutyCycle().getValueAsDouble()));
    }

    public void climberUp() {
        m_climberMotor.setControl(new PositionVoltage(0).withPosition(ClimberConstants.climberTopPosition));
    }

    public void climberDown() {
        m_climberMotor.setControl(new PositionVoltage(0).withPosition(ClimberConstants.climberBottomPosition));
    }

    // TODO: don't do it if not calibrated

    public void climberCalibrateDown() {
        m_climberMotor.setControl(new DutyCycleOut(ClimberConstants.climberCalibrateSpeed));
    }

    public void startCalibration() {
        if (m_isCalibrated) {
            return;
        }
        m_timer.reset();
        climberCalibrateDown();
        m_timer.start();
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

    public Command Calibrate() {
        return new InstantCommand(() -> startCalibration()).andThen(new WaitUntilCommand(() -> isDown()))
                .finallyDo(() -> {
                    resetMotorEncoders();
                        m_isCalibrated = true;
                    ;
                    m_climberMotor.stopMotor();
                }).withTimeout(15.0);

    }

    public Command clearCallibration() {
        return new InstantCommand(() -> m_isCalibrated = false);
    }

    public boolean isDown() {
        return (Math.abs(m_climberMotor.getVelocity().getValueAsDouble()) < 0.05 // make constants
                && m_timer.hasElapsed(0.2));
    }

    public void resetMotorEncoders() {
        m_climberMotor.setPosition(0);
    }

}
