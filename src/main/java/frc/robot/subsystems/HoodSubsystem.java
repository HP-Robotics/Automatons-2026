package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {
    TalonFX m_hoodMotor = new TalonFX(MotorIDConstants.hoodMotor);
    boolean m_isCalibrated = false;
    Timer m_timer = new Timer();
    double m_targetPosition = 0;
    

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("Hood");

    public HoodSubsystem() {
        final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(HoodConstants.hoodTop))
                .withSlot0(new Slot0Configs()
                        .withKP(HoodConstants.kP)
                        .withKD(HoodConstants.kD))
                .withMotorOutput(new MotorOutputConfigs()
                        .withPeakForwardDutyCycle(HoodConstants.maxSpeed)
                        .withPeakReverseDutyCycle(-HoodConstants.maxSpeed)
                        .withInverted(InvertedValue.Clockwise_Positive));
        m_hoodMotor.getConfigurator().apply(hoodMotorConfigs);

        table.putValue("setTargetPosition", NetworkTableValue.makeDouble(HoodConstants.defaultHoodPosition));
    }

    @Override
    public void periodic() {
        table.putValue("position", NetworkTableValue.makeDouble(m_hoodMotor.getPosition().getValueAsDouble()));
        table.putValue("isCalibrated", NetworkTableValue.makeBoolean(m_isCalibrated));
        table.putValue("isShotLegal", NetworkTableValue.makeBoolean(isHoodAimed()));
        table.putValue("targetPosition", NetworkTableValue.makeDouble(m_targetPosition));
    }

    public void setHood(double position) {

        if (m_isCalibrated && position > 0) {
            m_targetPosition = position;
            m_hoodMotor.setControl(new PositionDutyCycle(position));
        }

    }

    public Command setHoodPosition(double position) {
        return new InstantCommand(
                () -> {
                    this.setHood(position);
                }, this);
    }

    public boolean isHoodAimed() {
        if ( Math.abs(m_targetPosition - m_hoodMotor.getRotorPosition().getValueAsDouble()) < HoodConstants.hoodErrorThreshold) {
            return m_isCalibrated; 
        }
        return false;
    }

    public void hoodDown() {
        m_targetPosition = HoodConstants.hoodBottom;
        m_hoodMotor.setControl(new PositionDutyCycle(HoodConstants.hoodBottom));
    }

    public void startCalibration() {
        if (m_isCalibrated) {
            return;
        }
        m_timer.reset();
        m_hoodMotor.setControl(new DutyCycleOut(HoodConstants.hoodCalibrateSpeed));
        m_timer.start();
    }

    public Command MagicHood(DoubleSupplier magicHoodPosition) {
        return new RunCommand(() -> {

            setHood(magicHoodPosition.getAsDouble());
        }, this).finallyDo(this::hoodDown);
    }

    public Command Calibrate() {
        if (m_isCalibrated) {
            return new WaitCommand(0);
        }
        return new InstantCommand(() -> startCalibration()).andThen(new WaitUntilCommand(() -> isDown()))
                .finallyDo(() -> {
                    resetMotorEncoders();
                    if (isDown()) {
                        m_isCalibrated = true;
                    }
                    ;
                    m_hoodMotor.stopMotor();
                }).withTimeout(3.0);

    }

    public Command clearCallibration() {
        return new InstantCommand(() -> m_isCalibrated = false);
    }

    public Command hoodFromNetworkTables() {
        return new RunCommand(
                () -> {
                    setHood(table.getEntry("setTargetPosition").getDouble(HoodConstants.defaultHoodPosition));

                }, this).finallyDo(this::hoodDown);
    }

    public boolean isDown() {
        return (Math.abs(m_hoodMotor.getVelocity().getValueAsDouble()) < 0.02 // make constants
                && m_timer.hasElapsed(0.2));
    }

    public void resetMotorEncoders() {
        m_hoodMotor.setPosition(0);
    }
}
