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
    private final SparkMax spinnyMotor;
    private final SparkMax john;

    public ShooterSubsystem() {
        spinnyMotor = new SparkMax(Motors.keremdeniz, MotorType.kBrushed);
        john = new SparkMax(Motors.john, MotorType.kBrushed);//shootmotor
        SparkMaxConfig spinnyMotorConfig = new SparkMaxConfig();
        SparkMaxConfig johnConfig = new SparkMaxConfig();

        spinnyMotorConfig.voltageCompensation(Limits.voltageLimit);
        johnConfig.voltageCompensation(Limits.voltageLimit);
        
        spinnyMotorConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        johnConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        john.configure(johnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shoot(double speed) {        
        spinnyMotor.set(speed);
        john.set(speed);
    }
    
    public void spin(double speed) {
        spinnyMotor.set(speed);
    }
}
