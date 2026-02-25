package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limits;
import frc.robot.Constants.Motors;

public class BallSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final SparkMax intakeMotor;

    public BallSubsystem() {
        shooterMotor = new SparkMax(Motors.shooterMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(Motors.intakeMotorID, MotorType.kBrushed);
        SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        shooterMotorConfig.voltageCompensation(Limits.voltageLimit);
        intakeMotorConfig.voltageCompensation(Limits.voltageLimit);
        
        shooterMotorConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        intakeMotorConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runShooter(double speed) {        
        shooterMotor.set(speed); 
        intakeMotor.set(speed);
    }
    
    public void runIntake(double speed) {
        intakeMotor.set(speed);
        shooterMotor.set(-speed);
    }
}
