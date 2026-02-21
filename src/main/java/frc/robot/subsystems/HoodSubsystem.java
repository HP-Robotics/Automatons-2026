package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDConstants;

public class HoodSubsystem extends SubsystemBase {
    TalonFX m_hoodMotor = new TalonFX(MotorIDConstants.hoodMotor);
    boolean m_isCalibrated = false;
    double m_targetPosition;
    double m_defaultPosition;
    Timer m_timer = new Timer();

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("Hood");

    public HoodSubsystem() {
        double currentPosition = m_hoodMotor.getPosition().getValueAsDouble();
        m_defaultPosition = currentPosition + 0.01;
        m_targetPosition = m_defaultPosition;
        table.putValue("setTargetPosition", NetworkTableValue.makeDouble(m_defaultPosition));

        final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0.01)
                        .withForwardSoftLimitThreshold(HoodConstants.hoodTop))
                .withSlot0(new Slot0Configs()
                        .withKP(HoodConstants.kP)
                        .withKD(HoodConstants.kD))
                .withMotorOutput(new MotorOutputConfigs()
                        .withPeakForwardDutyCycle(HoodConstants.maxSpeed)
                        .withPeakReverseDutyCycle(-HoodConstants.maxSpeed));
        m_hoodMotor.getConfigurator().apply(hoodMotorConfigs);
        m_hoodMotor.setPosition(HoodConstants.hoodTop);
    }

    public void hoodCalibrateDown() {
        m_hoodMotor.setControl(new DutyCycleOut(HoodConstants.hoodCalibrateSpeed));
    }

    @Override
    public void periodic() {
        table.putValue("targetPosition", NetworkTableValue.makeDouble(m_targetPosition));
        table.putValue("position", NetworkTableValue.makeDouble(m_hoodMotor.getPosition().getValueAsDouble()));
        table.putValue("isCalibrated", NetworkTableValue.makeBoolean(m_isCalibrated));
    }

    public void getFromNetworkTables() {
        m_targetPosition = table.getEntry("setTargetPosition").getDouble(m_defaultPosition);
    }

    public void setHood(double position) {

        if (m_isCalibrated) {
            m_hoodMotor.setControl(new PositionDutyCycle(position));
        }

    }

    public void hoodUp() {
        setHood(HoodConstants.hoodTop);
    }

    public void hoodDown() {
        setHood(HoodConstants.hoodBottom);
    }

    public void magicHood(double position) {
        setHood((m_targetPosition));
    }

    public void networktablesHood(double position) {
        setHood((m_targetPosition));
    }

    public Command setHoodPosition(double position) {
        return new InstantCommand(
                () -> {
                    this.setHood(position);
                }, this);
    }

    public void startCalibration() {
        if (m_isCalibrated) {
            return;
        }
        m_timer.reset();
        hoodCalibrateDown();
        m_timer.start();
    }

    public Command Calibrate() {
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

    public Command hoodFromMagic() {
        return new InstantCommand(
                () -> magicHood(m_targetPosition));

    }

    public Command hoodFromNetworkTables() {
        return new RunCommand(
                () -> {
                    getFromNetworkTables();
                    magicHood(m_targetPosition);
                }, this);
    }

    public boolean isDown() {
        return (Math.abs(m_hoodMotor.getVelocity().getValueAsDouble()) < 0.02 // make constants
                && m_timer.hasElapsed(0.2));
    }

    public void resetMotorEncoders() {
        m_hoodMotor.setPosition(0);
    }
}
