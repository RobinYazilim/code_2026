package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Motors;   
/* kitbot'da burayı import static yapmislar.
+ drive constants shooter constants diye ayırsak daha rahat olmaz mı? */

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax rightLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightFollower;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    
    private final ADXRS450_Gyro gyro;

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

        gyro = new ADXRS450_Gyro();

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
        pose = new Pose2d(0, 0, new Rotation2d(gyro.getAngle()));
        estimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(gyro.getAngle()), wheels.leftMeters, wheels.rightMeters, pose);

        /*
        
        ModuleConfig moduleConfig = new ModuleConfig(
            Measurements.wheelDiameter / 2.0,
            Limits.maxPhysicalSpeedMetersPerSecond,
            1.2,
            DCMotor.getNEO(2),
            Measurements.gearRatio,
            Limits.motorCurrentLimit
        );

        RobotConfig config = new RobotConfig(
            30.0,
            Measurements.momentOfInertia,
            moduleConfig,
            Measurements.distBetweenWheels
        );

        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPLTVController(0.02), 
            config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        
         */
    }

    @Override
    public void periodic() {
        wheels.leftMeters = leftEncoder.getPosition() * Measurements.metersPerMotorRotation;
        wheels.rightMeters = rightEncoder.getPosition() * Measurements.metersPerMotorRotation;

        estimator.update(Rotation2d.fromDegrees(gyro.getAngle()), wheels);

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

    public double getGyroValue()
    {
        return gyro.getAngle();
    }

    // BUNLARI ASLA SILMEYIN VE ISIMLERINI DEGISTIRMEYIN PATHPLANNER ICIN GEREKLI
    public Pose2d getPose()
    {
        return estimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose)
    {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        wheels.leftMeters = 0;
        wheels.rightMeters = 0;

        gyro.reset();

        estimator.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()), wheels, newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        double leftVelocityRPM = leftEncoder.getVelocity();
        double rightVelocityRPM = rightEncoder.getVelocity();

        double leftVelocityMetersPerSec = (leftVelocityRPM / 60.0) * Measurements.metersPerMotorRotation;
        double rightVelocityMetersPerSec = (rightVelocityRPM / 60.0) * Measurements.metersPerMotorRotation;

        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocityMetersPerSec, rightVelocityMetersPerSec);

        return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        DifferentialDriveWheelSpeeds targetWheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        double leftVolts = (targetWheelSpeeds.leftMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;
        double rightVolts = (targetWheelSpeeds.rightMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;


        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);

        drive.feed();
    }
}
