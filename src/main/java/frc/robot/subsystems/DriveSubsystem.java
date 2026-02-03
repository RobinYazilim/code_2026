package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Motors;   
/* kitbot'da burayı import static yapmislar.
+ drive constants shooter constants diye ayırsak daha rahat olmaz mı? */

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax rightLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightFollower;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final DifferentialDrive drive;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveWheelPositions wheels;
    private final DifferentialDrivePoseEstimator estimator;
    private final Pose2d pose;

    // log isi
    private final DoubleLogEntry logLeft = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Left Meters");
    private final DoubleLogEntry logRight = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Right Meters");
    private final DoubleLogEntry logAverage = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Average Meters");


    public DriveSubsystem() {
        leftLeader = new SparkMax(Motors.leftLeaderID, MotorType.kBrushless);
        rightLeader = new SparkMax(Motors.rightLeaderID, MotorType.kBrushless);
        leftFollower = new SparkMax(Motors.leftFollowerID, MotorType.kBrushless);
        rightFollower = new SparkMax(Motors.rightFollowerID, MotorType.kBrushless);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        leftLeaderConfig.voltageCompensation(Limits.voltageLimit);
        rightLeaderConfig.voltageCompensation(Limits.voltageLimit);
        leftFollowerConfig.voltageCompensation(Limits.voltageLimit);
        rightFollowerConfig.voltageCompensation(Limits.voltageLimit);

        leftLeaderConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        rightLeaderConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        leftFollowerConfig.smartCurrentLimit(Limits.motorCurrentLimit);
        rightFollowerConfig.smartCurrentLimit(Limits.motorCurrentLimit);

        leftFollowerConfig.follow(leftLeader);
        rightFollowerConfig.follow(rightLeader);

        rightLeaderConfig.inverted(true);

        leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);


        drive = new DifferentialDrive(leftLeader, rightLeader);
        kinematics = new DifferentialDriveKinematics(Measurements.distBetweenWheels);
        wheels = new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
        pose = new Pose2d(0, 0, new Rotation2d());
        estimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), wheels.leftMeters, wheels.rightMeters, pose);
    }

    @Override
    public void periodic() {
        wheels.leftMeters = leftEncoder.getPosition() * Measurements.metersPerMotorRotation;
        wheels.rightMeters = rightEncoder.getPosition() * Measurements.metersPerMotorRotation;

        estimator.update(new Rotation2d(), wheels);

        //SmartDashboard.putNumber("Drive/Left Meters", wheels.leftMeters);
        //SmartDashboard.putNumber("Drive/Right Meters", wheels.rightMeters);
        //SmartDashboard.putNumber("Drive/AVG Meters", (wheels.leftMeters + wheels.rightMeters)/(double)2);
        logLeft.append(wheels.leftMeters);
        logRight.append(wheels.rightMeters);
        logAverage.append(getAverageMeters());
    }

    public void drive(double forward, double rotation)
    {
        drive.arcadeDrive(forward, rotation);
    }

    public void tankDrive(double left, double right)
    {
        drive.tankDrive(left, right);
    }

    public double getAverageMeters()
    {
        return (wheels.leftMeters + wheels.rightMeters)/2;
    }

    public double getLeftMeters()
    {
        return wheels.leftMeters;
    }

    public double getRightMeters()
    {
        return wheels.rightMeters;
    }

}
