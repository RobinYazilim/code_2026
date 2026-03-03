package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Motors;   
/* kitbot'da burayı import static yapmislar.
+ drive constants shooter constants diye ayırsak daha rahat olmaz mı? */

import org.opencv.core.Mat;
import org.photonvision.EstimatedRobotPose;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    //private final DifferentialDriveWheelPositions wheels;
    private final DifferentialDrivePoseEstimator estimator;
    private final Pose2d pose;

    // log isi
    private final DoubleLogEntry logLeft = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Left Meters");
    private final DoubleLogEntry logRight = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Right Meters");
    private final DoubleLogEntry logAverage = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Average Meters");
    private final DoubleLogEntry logAngle = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Gyro Angle");

    private double maxSpeed = 0.0;

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
        gyro.calibrate();
        gyro.reset();

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

        drive = new DifferentialDrive(leftLeader, rightLeader);
        kinematics = new DifferentialDriveKinematics(Measurements.distBetweenWheels);
        DifferentialDriveWheelPositions wheels = new DifferentialDriveWheelPositions(leftEncoder.getPosition()*Measurements.metersPerMotorRotation, rightEncoder.getPosition()*Measurements.metersPerMotorRotation);
        pose = new Pose2d(0, 0, new Rotation2d(gyro.getAngle()));
        estimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(gyro.getAngle()), wheels.leftMeters, wheels.rightMeters, pose);

        DCMotor driveMotor = DCMotor.getNEO(2).withReduction(Measurements.gearRatio);
        
        ModuleConfig moduleConfig = new ModuleConfig(
            Measurements.wheelDiameter / 2.0,
            Limits.maxPhysicalSpeedMetersPerSecond,
            1.2,
            driveMotor,
            Limits.motorCurrentLimit,
            2
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
        
    }

    @Override
    public void periodic() {
        DifferentialDriveWheelPositions wheels = new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Measurements.metersPerMotorRotation, rightEncoder.getPosition() * Measurements.metersPerMotorRotation);

        estimator.update(Rotation2d.fromDegrees(gyro.getAngle()), wheels);

        double currentForwardSpeed = Math.abs(getRobotRelativeSpeeds().vxMetersPerSecond);
        
        if (currentForwardSpeed > maxSpeed) {
            maxSpeed = currentForwardSpeed;
        }
        SmartDashboard.putNumber("Drive/LIVE Speed (m_s)", currentForwardSpeed);
        SmartDashboard.putNumber("Drive/MAX Speed (m_s)", maxSpeed);

        logLeft.append(wheels.leftMeters);
        logRight.append(wheels.rightMeters);
        logAverage.append(getAverageMeters());
        logAngle.append(gyro.getAngle());

        SmartDashboard.putNumber("Drive/Left Meters", wheels.leftMeters);
        SmartDashboard.putNumber("Drive/Right Meters", wheels.rightMeters);
        SmartDashboard.putNumber("Drive/Average Meters", getAverageMeters());
        SmartDashboard.putNumber("Drive/Gyro Angle", gyro.getAngle());

        Pose2d currentPose = getPose();
        SmartDashboard.putNumberArray("Odometry/Robot Pose", new double[] {
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getDegrees()
        });
    }

    public void drive(double forward, double rotation)
    {
        drive.arcadeDrive(forward, rotation);
    }

    public void tankDrive(double left, double right)
    {
        drive.tankDrive(left, right);
    }

    public void addVisionMeasurement(EstimatedRobotPose visionPose)
    {
        Pose3d pose = visionPose.estimatedPose;
        boolean isSafeZHeight = Math.abs(pose.getZ()) < 0.25;
        boolean isWithinFieldBounds = pose.getX() >= 0 && pose.getX() <= 16.54
                                   && pose.getY() >= 0 && pose.getY() <= 8.21;
        
        if (!isSafeZHeight || !isWithinFieldBounds)
        {
            return; // cop gibi measuyrement olmus harbi saha disi
        }
        
        if (visionPose.targetsUsed.size() == 1)
        {
            double ambuguity = visionPose.targetsUsed.get(0).getPoseAmbiguity();
            if (ambuguity > 0.2 || ambuguity == -1)
            {
                return; // yine cop
            }
        }

        double averageDist = 0;
        for (var target : visionPose.targetsUsed)
        {
            averageDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        averageDist /= visionPose.targetsUsed.size();

        Vector<N3> dynamicStandardDeviationVector;
        if (visionPose.targetsUsed.size() > 0)
        {
            dynamicStandardDeviationVector = VecBuilder.fill(0.1, 0.1, Math.toRadians(5));
        }
        else 
        {
            double trust = 1.0 + (averageDist * 0.5);
            dynamicStandardDeviationVector = VecBuilder.fill(0.5 * trust, 0.5 * trust, Math.toRadians(30*trust)); // furoozin olafin pienciisi: https://upload.wikimedia.org/wikipedia/en/6/6d/Olaf_from_Disney%27s_Frozen.png
        }
        estimator.addVisionMeasurement(pose.toPose2d(), visionPose.timestampSeconds, dynamicStandardDeviationVector);
    }
    

    public double getAverageMeters()
    {
        double leftMeters = leftEncoder.getPosition() * Measurements.metersPerMotorRotation;
        double rightMeters = rightEncoder.getPosition() * Measurements.metersPerMotorRotation;
        return (leftMeters + rightMeters) / 2.0;
    }

    public double getLeftMeters()
    {
        return leftEncoder.getPosition() * Measurements.metersPerMotorRotation;
    }

    public double getRightMeters()
    {
        return rightEncoder.getPosition() * Measurements.metersPerMotorRotation;
    }

    public double getGyroValue()
    {
        return gyro.getAngle();
    }

    public void resetGyro()
    {
        gyro.reset();
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

        gyro.reset();

        estimator.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()), new DifferentialDriveWheelPositions(0,  0), newPose);
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

        targetWheelSpeeds.desaturate(Limits.maxPhysicalSpeedMetersPerSecond);
        double leftVolts = (targetWheelSpeeds.leftMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;
        double rightVolts = (targetWheelSpeeds.rightMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;

        leftVolts = MathUtil.clamp(leftVolts, -Limits.voltageLimit, Limits.voltageLimit);
        rightVolts = MathUtil.clamp(rightVolts, -Limits.voltageLimit, Limits.voltageLimit);

        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);

        drive.feed();
    }
}
