package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMetersCommand extends Command {
    private final PIDController distancePid;
    private final PIDController headingPid;

    private final DriveSubsystem driveSub;

    private final double meters;
    private double startMeters;
    private double startAngle;

    private final DoubleLogEntry logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Setpoint");

    double angle = 0;                                                                                                                                                                                           

    public DriveMetersCommand(double meters, DriveSubsystem driveSub)
    {
        this.meters = meters;
        this.driveSub = driveSub;

        addRequirements(driveSub);

        angle = driveSub.getGyroValue();

        double distKp = 0.7; 
        double distKi = 0.0;
        double distKd = 0.05;
        distancePid = new PIDController(distKp, distKi, distKd);
        distancePid.setTolerance(0.03, 0.1);

        double headKp = 0.04;
        double headKi = 0.0;
        double headKd = 0.0;
        headingPid = new PIDController(headKp, headKi, headKd);

        }

    @Override
    public void initialize()
    {
        startMeters = driveSub.getAverageMeters();
        
        distancePid.reset();
        headingPid.reset();

        driveSub.resetGyro();
        
        angle = driveSub.getGyroValue();

    }

    @Override
    public void execute()
    {

        double distKp = SmartDashboard.getNumber("Tune-Dist_kP", 0.7);
        double distKi = SmartDashboard.getNumber("Tune-Dist_kI", 0.0);
        double distKd = SmartDashboard.getNumber("Tune-Dist_kD", 0.05);

        double headKp = SmartDashboard.getNumber("Tune-Head_kP", 0.04);
        double headKi = SmartDashboard.getNumber("Tune-Head_kI", 0.0);
        double headKd = SmartDashboard.getNumber("Tune-Head_kD", 0.0);

        distancePid.setPID(distKp, distKi, distKd);
        headingPid.setPID(headKp, headKi, headKd);

        double average = driveSub.getAverageMeters() - startMeters;
        double rotation = driveSub.getGyroValue() - angle;

        double forwardSpeed = distancePid.calculate(average, meters);
        
        double turnSpeed = headingPid.calculate(rotation, angle);
        
        double leftOutput = forwardSpeed + turnSpeed;
        double rightOutput = forwardSpeed - turnSpeed;

        leftOutput = MathUtil.clamp(leftOutput, -Limits.clampDriveSpeedLimit, Limits.clampDriveSpeedLimit);
        rightOutput = MathUtil.clamp(rightOutput, -Limits.clampDriveSpeedLimit, Limits.clampDriveSpeedLimit);

        driveSub.tankDrive(leftOutput, rightOutput);
        logSetpoint.append(meters);
    }

    @Override
    public boolean isFinished()
    {
        return distancePid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) 
    {
        String debugMessage = "DriveMetersCommand (ID: " + this.hashCode() + 
                              ") ended. Interrupted = " + interrupted;
        
        DataLogManager.log(debugMessage);
        driveSub.tankDrive(0.0, 0.0);
    }
}
