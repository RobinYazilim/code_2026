package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Motors;

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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;


public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax rightLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightFollower;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final PIDController leftPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController rightPID = new PIDController(1.5, 0.0, 0.0);
    
    private final SimpleMotorFeedforward feedforward;

    @SuppressWarnings("unused")
    private final Pigeon2 piegonWOW;
    private final Pigeon2 gyro;

    // piegon docs bunlar bakmak isteyenlere:
    // https://v6.docs.ctr-electronics.com/en/stable/ -->BU 6 bizimki
    // https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/Pigeon2.html 
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/pigeon2/index.html?utm_source=



    private final DifferentialDrive drive;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrivePoseEstimator estimator;
    private final Pose2d pose;
    private final Field2d field;

    // log isi
    private final DoubleLogEntry logLeft = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Left Meters");
    private final DoubleLogEntry logRight = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Right Meters");
    private final DoubleLogEntry logAverage = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Average Meters");
    private final DoubleLogEntry logAngle = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Gyro Angle");
    private final StructLogEntry<Pose2d> logPose = StructLogEntry.create(DataLogManager.getLog(), "Drive/Robot Pose", Pose2d.struct);

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

        gyro = new Pigeon2(27);
        gyro.reset();
        piegonWOW = gyro; // piegonWOW gyro ile aynı şeyi ifade ediyor, kodun geri kalanında gyro olarak kullanacağız
        // manevi anlamda orada 

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

        leftFollowerConfig.signals.primaryEncoderPositionPeriodMs(500);
        leftFollowerConfig.signals.primaryEncoderVelocityPeriodMs(500);
        rightFollowerConfig.signals.primaryEncoderPositionPeriodMs(500);
        rightFollowerConfig.signals.primaryEncoderVelocityPeriodMs(500);
        // optimizasyon isi

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
        feedforward = new SimpleMotorFeedforward(0.3, 3.4285714286);

        // piegon 
        pose = new Pose2d(0, 0, gyro.getRotation2d());
        estimator = new DifferentialDrivePoseEstimator(kinematics, pose.getRotation(), wheels.leftMeters, wheels.rightMeters, pose);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

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

        //estimator.update(Rotation2d.fromDegrees(gyro.getAngle()), wheels); --> eski

        Rotation2d currentRotation = gyro.getRotation2d();

        estimator.update(currentRotation, wheels);

        double currentForwardSpeed = Math.abs(getRobotRelativeSpeeds().vxMetersPerSecond);
        
        if (currentForwardSpeed > maxSpeed) {
            maxSpeed = currentForwardSpeed;
        }
        SmartDashboard.putNumber("Drive/LIVE Speed (m_s)", currentForwardSpeed);
        SmartDashboard.putNumber("Drive/MAX Speed (m_s)", maxSpeed);

        logLeft.append(wheels.leftMeters);
        logRight.append(wheels.rightMeters);
        logAverage.append(getAverageMeters());
        logAngle.append(currentRotation.getDegrees());
        logPose.append(getPose());


        SmartDashboard.putNumber("Drive/Left Meters", wheels.leftMeters);
        SmartDashboard.putNumber("Drive/Right Meters", wheels.rightMeters);
        SmartDashboard.putNumber("Drive/Average Meters", getAverageMeters());
        SmartDashboard.putNumber("Drive/Gyro Angle", currentRotation.getDegrees());

        field.setRobotPose(getPose());
    }
    // merhaba
    // bay bay 
 
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
        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 180.0)
        {
            return; // fazla hizli donuyo motion blur falan
        }
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
        if (visionPose.targetsUsed.size() > 2)
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
    
    private double[] desaturateVoltages(double leftVolts, double rightVolts, double maxVoltage) {
        double maxAbsVoltage = Math.max(Math.abs(leftVolts), Math.abs(rightVolts));
        
        if (maxAbsVoltage > maxVoltage) {
            double scale = maxVoltage / maxAbsVoltage;
            leftVolts *= scale;
            rightVolts *= scale;
        }
        
        return new double[] {leftVolts, rightVolts};
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
        return gyro.getRotation2d().getDegrees();
    }

    public void resetGyro()
    {
        gyro.reset();
    }

    // BUNLARI ASLA SILMEYIN VE ISIMLERINI DEGISTIRMEYIN PATHPLANNER ICIN GEREKLI

    //gyro için bişeler değiştirmis olabilirim ama yanlış yapmıssam diye eskileri silmedim yamuk
    public Pose2d getPose()
    {
        return estimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose)
    {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        estimator.resetPosition(
            gyro.getRotation2d(), new DifferentialDriveWheelPositions(0,  0), newPose);
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
        double leftFeedF = feedforward.calculate(targetWheelSpeeds.leftMetersPerSecond);
        double rightFeedF = feedforward.calculate(targetWheelSpeeds.rightMetersPerSecond);
        // double leftVolts = (targetWheelSpeeds.leftMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;
        // double rightVolts = (targetWheelSpeeds.rightMetersPerSecond / Limits.maxPhysicalSpeedMetersPerSecond) * Limits.voltageLimit;

        double currentLeftMpS = (leftEncoder.getVelocity() / 60) * Measurements.metersPerMotorRotation;
        double currentRightMpS = (leftEncoder.getVelocity() / 60) * Measurements.metersPerMotorRotation;

        double leftVolts = leftFeedF + leftPID.calculate(currentLeftMpS, targetWheelSpeeds.leftMetersPerSecond);
        double rightVolts = rightFeedF + rightPID.calculate(currentRightMpS, targetWheelSpeeds.rightMetersPerSecond);

        double[] desaturatedVoltages = desaturateVoltages(leftVolts, rightVolts, Limits.voltageLimit);
        leftVolts = desaturatedVoltages[0];
        rightVolts = desaturatedVoltages[1];

        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);

        drive.feed();
    }
}
