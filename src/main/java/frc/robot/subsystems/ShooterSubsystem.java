package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limits;
import frc.robot.Constants.Motors;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor1;
    private final SparkMax shooterMotor2;

    public ShooterSubsystem() {
        shooterMotor1 = new SparkMax(Motors.shooterMotor1ID, MotorType.kBrushed);
        shooterMotor2 = new SparkMax(Motors.shooterMotor2ID, MotorType.kBrushed);

        SparkMaxConfig shooterMotor1Config = new SparkMaxConfig();
        SparkMaxConfig shooterMotor2Config = new SparkMaxConfig();

        shooterMotor1Config.voltageCompensation(Limits.voltageLimit);
        shooterMotor2Config.voltageCompensation(Limits.voltageLimit);
        
        shooterMotor1Config.smartCurrentLimit(Limits.motorCurrentLimit);
        shooterMotor2Config.smartCurrentLimit(Limits.motorCurrentLimit);

        shooterMotor2Config.inverted(true);

        shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(shooterMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runShooter(double speed) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }
    
}
