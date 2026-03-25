package frc.robot.subsystems;

import frc.robot.Constants.Limits;
import frc.robot.Constants.Measurements;
import frc.robot.Constants.Motors;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftFront;
    private final SparkMax rightFront;
    private final SparkMax leftBack;
    private final SparkMax rightBack;

    // MECANUM FIX: We need 4 encoders, not 2
    private final RelativeEncoder leftFrontEncoder;
    private final RelativeEncoder rightFrontEncoder;
    private final RelativeEncoder leftBackEncoder;
    private final RelativeEncoder rightBackEncoder;

    // MECANUM FIX: We need 4 PIDs to control each wheel independently
    private final PIDController leftFrontPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController rightFrontPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController leftBackPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController rightBackPID = new PIDController(1.5, 0.0, 0.0);
    
    private final SimpleMotorFeedforward feedforward;

    private final Pigeon2 gyro;

    private final MecanumDrive drive;
    private final MecanumDriveKinematics kinematics;
    private final MecanumDrivePoseEstimator estimator;
    private final Field2d field;

    // Log entries updated for Mecanum speeds
    private final DoubleLogEntry logSpeedMetersPerSec = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Speed (m_s)");
    private final DoubleLogEntry logAngle = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Gyro Angle");
    private final StructLogEntry<Pose2d> logPose = StructLogEntry.create(DataLogManager.getLog(), "Drive/Robot Pose", Pose2d.struct);

    private double maxSpeed = 0.0;

    public DriveSubsystem() {
        leftFront = new SparkMax(Motors.leftFrontID, MotorType.kBrushless);
        rightFront = new SparkMax(Motors.rightFrontID, MotorType.kBrushless);
        leftBack = new SparkMax(Motors.leftBackID, MotorType.kBrushless);
        rightBack = new SparkMax(Motors.rightBackID, MotorType.kBrushless);

        leftFrontEncoder = leftFront.getEncoder();
        rightFrontEncoder = rightFront.getEncoder();
        leftBackEncoder = leftBack.getEncoder();
        rightBackEncoder = rightBack.getEncoder();

        leftFrontEncoder.setPosition(0);
        rightFrontEncoder.setPosition(0);
        leftBackEncoder.setPosition(0);
        rightBackEncoder.setPosition(0);

        gyro = new Pigeon2(27);
        gyro.reset();

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

        // Optimizations
        leftFrontConfig.signals.primaryEncoderPositionPeriodMs(20);
        rightFrontConfig.signals.primaryEncoderPositionPeriodMs(20);
        leftBackConfig.signals.primaryEncoderPositionPeriodMs(20);
        rightBackConfig.signals.primaryEncoderPositionPeriodMs(20);

        rightFrontConfig.inverted(true);
        rightBackConfig.inverted(true); // Don't forget to invert the back right too!

        leftFront.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBack.configure(leftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBack.configure(rightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
        
        // MECANUM FIX: Kinematics requires the location of all 4 wheels relative to the robot center
        kinematics = new MecanumDriveKinematics(
            Measurements.leftFrontPos, 
            Measurements.rightFrontPos, 
            Measurements.leftBackPos, 
            Measurements.rightBackPos
        );
        
        feedforward = new SimpleMotorFeedforward(0.3, 3.4285714286);

        estimator = new MecanumDrivePoseEstimator(
            kinematics, 
            gyro.getRotation2d(), 
            getCurrentWheelPositions(), 
            new Pose2d()
        );

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        DCMotor driveMotor = DCMotor.getNEO(2).withReduction(Measurements.gearRatio);
        
        ModuleConfig moduleConfig = new ModuleConfig(
            Measurements.wheelDiameter / 2.0,
            Limits.maxPhysicalSpeedMetersPerSecond,
            1.2,
            driveMotor,
            Limits.motorCurrentLimit,
            4 // MECANUM FIX: 4 modules, not 2
        );

        RobotConfig config = new RobotConfig(
            30.0,
            Measurements.momentOfInertia,
            moduleConfig,
            Measurements.distBetweenWheels
        );

        // MECANUM FIX: PathPlanner Holonomic Configuration
        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ), 
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
        Rotation2d currentRotation = gyro.getRotation2d();

        // Update odometry with all 4 wheels
        estimator.update(currentRotation, getCurrentWheelPositions());

        ChassisSpeeds speeds = getRobotRelativeSpeeds();
        // Calculate hypotenuse of X and Y velocity for true speed
        double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        
        if (currentSpeed > maxSpeed) {
            maxSpeed = currentSpeed;
        }

        SmartDashboard.putNumber("Drive/LIVE Speed (m_s)", currentSpeed);
        SmartDashboard.putNumber("Drive/MAX Speed (m_s)", maxSpeed);
        SmartDashboard.putNumber("Drive/Gyro Angle", currentRotation.getDegrees());

        logSpeedMetersPerSec.append(currentSpeed);
        logAngle.append(currentRotation.getDegrees());
        logPose.append(getPose());

        field.setRobotPose(getPose());
    }

    // MECANUM FIX: Replaced Tank/Arcade drive with Cartesian Drive (allows sideways strafing)
    public void driveCartesian(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            drive.driveCartesian(xSpeed, ySpeed, rot, gyro.getRotation2d());
        } else {
            drive.driveCartesian(xSpeed, ySpeed, rot);
        }
    }

    public void addVisionMeasurement(EstimatedRobotPose visionPose) {
        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 180.0) {
            return; // Too fast, motion blur
        }
        
        Pose3d pose = visionPose.estimatedPose;
        boolean isSafeZHeight = Math.abs(pose.getZ()) < 0.25;
        boolean isWithinFieldBounds = pose.getX() >= 0 && pose.getX() <= 16.54
                                   && pose.getY() >= 0 && pose.getY() <= 8.21;
        
        if (!isSafeZHeight || !isWithinFieldBounds) {
            return; 
        }
        
        if (visionPose.targetsUsed.size() == 1) {
            double ambiguity = visionPose.targetsUsed.get(0).getPoseAmbiguity();
            if (ambiguity > 0.2 || ambiguity == -1) {
                return;
            }
        }

        double averageDist = 0;
        for (var target : visionPose.targetsUsed) {
            averageDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        averageDist /= visionPose.targetsUsed.size();

        Vector<N3> dynamicStandardDeviationVector;
        if (visionPose.targetsUsed.size() > 2) {
            dynamicStandardDeviationVector = VecBuilder.fill(0.1, 0.1, Math.toRadians(5));
        } else {
            double trust = 1.0 + (averageDist * 0.5);
            dynamicStandardDeviationVector = VecBuilder.fill(0.5 * trust, 0.5 * trust, Math.toRadians(30 * trust)); 
        }
        
        estimator.addVisionMeasurement(pose.toPose2d(), visionPose.timestampSeconds, dynamicStandardDeviationVector);
    }
    
    // MECANUM FIX: Desaturate for 4 wheels instead of 2
    private double[] desaturateVoltages(double fl, double fr, double bl, double br, double maxVoltage) {
        double maxAbsVoltage = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        
        if (maxAbsVoltage > maxVoltage) {
            double scale = maxVoltage / maxAbsVoltage;
            return new double[] {fl * scale, fr * scale, bl * scale, br * scale};
        }
        return new double[] {fl, fr, bl, br};
    }

    public double getGyroValue() {
        return gyro.getRotation2d().getDegrees();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    // Helper method to gather all 4 positions cleanly
    public MecanumDriveWheelPositions getCurrentWheelPositions() {
        return new MecanumDriveWheelPositions(
            leftFrontEncoder.getPosition() * Measurements.metersPerMotorRotation,
            rightFrontEncoder.getPosition() * Measurements.metersPerMotorRotation,
            leftBackEncoder.getPosition() * Measurements.metersPerMotorRotation,
            rightBackEncoder.getPosition() * Measurements.metersPerMotorRotation
        );
    }

    // Helper method to gather all 4 velocities cleanly
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            (leftFrontEncoder.getVelocity() / 60.0) * Measurements.metersPerMotorRotation,
            (rightFrontEncoder.getVelocity() / 60.0) * Measurements.metersPerMotorRotation,
            (leftBackEncoder.getVelocity() / 60.0) * Measurements.metersPerMotorRotation,
            (rightBackEncoder.getVelocity() / 60.0) * Measurements.metersPerMotorRotation
        );
    }

    public void resetPose(Pose2d newPose) {
        leftFrontEncoder.setPosition(0);
        rightFrontEncoder.setPosition(0);
        leftBackEncoder.setPosition(0);
        rightBackEncoder.setPosition(0);

        estimator.resetPosition(gyro.getRotation2d(), getCurrentWheelPositions(), newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds targetWheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        targetWheelSpeeds.desaturate(Limits.maxPhysicalSpeedMetersPerSecond);

        MecanumDriveWheelSpeeds currentSpeeds = getCurrentWheelSpeeds();

        // Feedforwards
        double flFeed = feedforward.calculate(targetWheelSpeeds.frontLeftMetersPerSecond);
        double frFeed = feedforward.calculate(targetWheelSpeeds.frontRightMetersPerSecond);
        double blFeed = feedforward.calculate(targetWheelSpeeds.rearLeftMetersPerSecond);
        double brFeed = feedforward.calculate(targetWheelSpeeds.rearRightMetersPerSecond);

        // PIDs
        double flPID = leftFrontPID.calculate(currentSpeeds.frontLeftMetersPerSecond, targetWheelSpeeds.frontLeftMetersPerSecond);
        double frPID = rightFrontPID.calculate(currentSpeeds.frontRightMetersPerSecond, targetWheelSpeeds.frontRightMetersPerSecond);
        double blPID = leftBackPID.calculate(currentSpeeds.rearLeftMetersPerSecond, targetWheelSpeeds.rearLeftMetersPerSecond);
        double brPID = rightBackPID.calculate(currentSpeeds.rearRightMetersPerSecond, targetWheelSpeeds.rearRightMetersPerSecond);

        // Total Volts
        double[] volts = desaturateVoltages(
            flFeed + flPID, 
            frFeed + frPID, 
            blFeed + blPID, 
            brFeed + brPID, 
            Limits.voltageLimit
        );

        leftFront.setVoltage(volts[0]);
        rightFront.setVoltage(volts[1]);
        leftBack.setVoltage(volts[2]);
        rightBack.setVoltage(volts[3]);

        drive.feed();
    }
}