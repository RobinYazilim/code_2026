package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limits;
import frc.robot.Constants.Motors;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeSpin;
    private final SparkMax intakeMove;

    public IntakeSubsystem() {
        intakeSpin = new SparkMax(Motors.intakeperfectdaire, MotorType.kBrushed);
        intakeMove = new SparkMax(Motors.intakeperfectsilindir, MotorType.kBrushed);
        SparkMaxConfig intakeSpinConfig = new SparkMaxConfig();
        SparkMaxConfig intakeMoveConfig = new SparkMaxConfig();

        intakeSpinConfig.voltageCompensation(Limits.voltageLimit);
        intakeMoveConfig.voltageCompensation(Limits.voltageLimit);
        
        intakeSpinConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        intakeMoveConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        intakeSpin.configure(intakeSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMove.configure(intakeMoveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinIntake(double speed) {        
        intakeSpin.set(speed);
    }
    
    public void moveIntake(double speed) {
        intakeMove.set(speed);
    }
}
