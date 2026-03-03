package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
                        .withKP(ShooterConstants.kP)
                        .withKI(ShooterConstants.kI)
                        .withKD(ShooterConstants.kD)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV));
        m_shooterMotor1.getConfigurator().apply(leftMotorConfigs);
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
            m_shooterMotor1.setControl(new VelocityTorqueCurrentFOC(speed));
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
        double speed = m_table.getEntry("speedToSet").getDouble(ShooterConstants.idleSpeed);
        setVelocity(speed);
    }

    public boolean atSpeed() {
        return (m_velocity > 0 && (Math
                .abs(m_velocity - m_shooterMotor1.getVelocity()
                        .getValueAsDouble()) < ShooterConstants.shooterErrorThreshold));
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
        m_table.putValue("atSpeed", NetworkTableValue.makeBoolean(atSpeed()));
    }

    public Command FixedShooter() {
        return new StartEndCommand(() -> {
            setVelocity(ShooterConstants.defaultShootingSpeed);
        },
                this::idleMotor,
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
