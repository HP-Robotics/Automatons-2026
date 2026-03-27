package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX m_shooterMotor1 = new TalonFX(MotorIDConstants.shooterMotorLeft);
    TalonFX m_shooterMotor2 = new TalonFX(MotorIDConstants.shooterMotorRight);
    public NetworkTable m_table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");
    double m_velocity = 0.0;
    double shooterSetpoint;

    public ShooterSubsystem() {
        final TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                        .withKP(ShooterConstants.kP0)
                        .withKI(ShooterConstants.kI)
                        .withKD(ShooterConstants.kD)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV))
                .withSlot1(new Slot1Configs()
                        .withKP(ShooterConstants.kP1)
                        .withKI(ShooterConstants.kI)
                        .withKD(ShooterConstants.kD)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV))
                .withSlot2(new Slot2Configs()
                        .withKP(ShooterConstants.kP2)
                        .withKI(ShooterConstants.kI)
                        .withKD(ShooterConstants.kD)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV));
        m_shooterMotor1.getConfigurator().apply(leftMotorConfigs);
        m_shooterMotor1.getClosedLoopOutput().setUpdateFrequency(50);
        m_shooterMotor1.getClosedLoopProportionalOutput().setUpdateFrequency(50);
        final TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.Clockwise_Positive));
        m_shooterMotor2.getConfigurator().apply(rightMotorConfigs);
        m_shooterMotor2.setControl(new Follower(MotorIDConstants.shooterMotorLeft, MotorAlignmentValue.Opposed));
        m_table.putValue("speedToSet", NetworkTableValue.makeDouble(ShooterConstants.defaultShootingSpeed));
    }
    // shooter modes: magic mode, fixed speed, network tables, stopped, idle

    public void setVelocity(double speed) {
        m_velocity = speed;
        if (speed > 1) {
            if (speed < 50) {
                m_shooterMotor1.setControl(new VelocityVoltage(speed).withSlot(0));
            } else if (speed < 65) {
                m_shooterMotor1.setControl(new VelocityVoltage(speed).withSlot(1));
            } else {
                m_shooterMotor1.setControl(new VelocityVoltage(speed).withSlot(2));
            }
        } else {
            m_shooterMotor1.setControl(new NeutralOut());
        }
        m_table.putValue("shooterSpeed", NetworkTableValue.makeDouble(speed));
        // shooterMotor2.set(speed);
    }

    public void stopMotor() {
        setVelocity(0);

    }

    public void idleMotor() {
        setVelocity(ShooterConstants.idleSpeed);

    }

    public void networkTablesSpeed() {
        double speed = m_table.getEntry("speedToSet").getDouble(ShooterConstants.defaultShootingSpeed);
        setVelocity(speed);
    }

    public boolean atSpeed(double threshold) {
        return (m_velocity > 0 && (Math
                .abs(m_velocity - m_shooterMotor1.getVelocity()
                        .getValueAsDouble()) < threshold));
    }

    public void magicSpeed() {
        double speed = ShooterConstants.idleSpeed;
        // TODO: add the magic
        m_shooterMotor1.set(speed);
        m_shooterMotor2.set(speed);
    }

    @Override
    public void periodic() {
        m_table.putValue("motorSpeed",
                NetworkTableValue.makeDouble(m_shooterMotor1.getRotorVelocity().getValueAsDouble()));
        m_table.putValue("atSpeed", NetworkTableValue.makeBoolean(atSpeed(ShooterConstants.shooterErrorThreshold)));
        m_table.putValue("atStableSpeed", NetworkTableValue.makeBoolean(atSpeed(ShooterConstants.shooterStableErrorThreshold)));
    }

    public Command FixedShooter(double speed) {
        return new StartEndCommand(() -> {
            setVelocity(speed);
        },
                this::stopMotor,
                this);
    }

    public Command StopShooter() {
        return new InstantCommand(this::stopMotor, this);
    }

    public Command IdleShooter() {
        return new StartEndCommand(this::idleMotor, this::stopMotor, this);
    }

    public Command AdjustableShooter() {
        return new RunCommand(this::networkTablesSpeed, this).finallyDo(this::stopMotor);
    }

    public Command MagicShooter(DoubleSupplier getMagicWheelSpeed) {
        return new RunCommand(() -> {
            setVelocity(getMagicWheelSpeed.getAsDouble());
        }, this).finallyDo(this::stopMotor);
    }
}
