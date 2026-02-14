package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterMotor1 = new TalonFX(MotorIDConstants.shooterMotor1);
    TalonFX shooterMotor2 = new TalonFX(MotorIDConstants.shooterMotor2);
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");
    
    public ShooterSubsystem() {
        final TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs()
                        .withKP(ShooterConstants.kP)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV));
        shooterMotor1.getConfigurator().apply(leftMotorConfigs);
        final TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive));
        shooterMotor2.getConfigurator().apply(rightMotorConfigs);
        shooterMotor2.setControl(new Follower(MotorIDConstants.shooterMotor1, MotorAlignmentValue.Opposed));
        table.putValue("shooterSpeed", NetworkTableValue.makeDouble(ShooterConstants.shootingSpeed));
    
        
    }
    // shooter modes: magic mode, fixed speed, network tables, stopped, idle

    public void setVelocity(double speed) {
        shooterMotor1.setControl(new VelocityTorqueCurrentFOC(speed));
        // shooterMotor2.set(speed);
    }

    public void stopMotor() {
        setVelocity(0);

    }

    public void idleMotor() {
        setVelocity(ShooterConstants.idleSpeed);

    }

    public void networkTablesSpeed() {
        double speed = table.getEntry("shooterSpeed").getDouble(ShooterConstants.idleSpeed);
        setVelocity(speed);
    }

    public void magicSpeed() {
        double speed = ShooterConstants.idleSpeed;
        // TODO: add the magic
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    public Command FixedShooter() {
        return new StartEndCommand(() -> {
            setVelocity(ShooterConstants.shootingSpeed);
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

    public Command MagicShooter() {
        return new StartEndCommand(this::magicSpeed, this::idleMotor, this);
    }

}
