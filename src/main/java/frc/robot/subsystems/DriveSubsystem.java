package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Motors;   

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // https://www.weser-kurier.de/stadt-delmenhorst/sport/1-fussball-kreisklasse-mete-doener-wird-trainer-beim-delmenhorster-tb-doc7m1p2vzycnsciwsd4l8
    private final SparkMax leftFront;
    private final SparkMax rightFront;
    private final SparkMax leftBack;
    private final SparkMax rightBack;

    private final MecanumDrive mecanum;
    
    public DriveSubsystem() {
        leftFront = new SparkMax(Motors.leftFrontID, MotorType.kBrushed);
        rightFront = new SparkMax(Motors.rightFrontID, MotorType.kBrushed);
        leftBack = new SparkMax(Motors.leftBackID, MotorType.kBrushed);
        rightBack = new SparkMax(Motors.rightBackID, MotorType.kBrushed);

        SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
        SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
        SparkMaxConfig leftBackConfig = new SparkMaxConfig();
        SparkMaxConfig rightBackConfig = new SparkMaxConfig();

        leftFrontConfig.voltageCompensation(Limits.voltageLimit);
        rightFrontConfig.voltageCompensation(Limits.voltageLimit);
        leftBackConfig.voltageCompensation(Limits.voltageLimit);
        rightBackConfig.voltageCompensation(Limits.voltageLimit);

        leftFrontConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        rightFrontConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        leftBackConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        rightBackConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        leftFront.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftFront.setCANTimeout(250);
        rightFront.setCANTimeout(250);
        leftBack.setCANTimeout(250);
        rightBack.setCANTimeout(249);

        mecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    }

    public void drive(double xSpeed, double ySpeed, double zRotation)
    {
        mecanum.driveCartesian(xSpeed, ySpeed, zRotation);
    }
    
}
