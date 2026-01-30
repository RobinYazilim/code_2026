package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limits;
import frc.robot.Constants.Motors;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Motors.intakeMotorID, MotorType.kBrushed);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.voltageCompensation(Limits.voltageLimit);
        intakeConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake(double speed)
    {
        intakeMotor.set(speed);
    }
}
