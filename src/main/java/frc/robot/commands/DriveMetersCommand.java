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
    private final double angle;
    private double startMeters;
    private double startAngle;

    private final DoubleLogEntry logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Setpoint");


    public DriveMetersCommand(double meters, DriveSubsystem driveSub)
    {
        this.meters = meters;
        this.angle = 0;

        this.driveSub = driveSub;

        addRequirements(driveSub);

        double distKp = 0.7; 
        double distKi = 0.0;
        double distKd = 0.05;
        distancePid = new PIDController(distKp, distKi, distKd);
        distancePid.setTolerance(0.01);

        double headKp = 0.02;
        double headKi = 0.01;
        double headKd = 0.0;
        headingPid = new PIDController(headKp, headKi, headKd);
        headingPid.setTolerance(5);

    }

    @Override
    public void initialize()
    {
        startMeters = driveSub.getAverageMeters();
        startAngle = driveSub.getGyroValue();
        
        distancePid.reset();
        headingPid.reset();
        

    }

    @Override
    public void execute()
    {

        double distKp = SmartDashboard.getNumber("Tune-Dist_kP", 0.9);
        double distKi = SmartDashboard.getNumber("Tune-Dist_kI", 0);
        double distKd = SmartDashboard.getNumber("Tune-Dist_kD", 0.05);

        distancePid.setPID(distKp, distKi, distKd);

        double average = driveSub.getAverageMeters() - startMeters;
        double rotation = driveSub.getGyroValue() - startAngle;

        double forwardSpeed = distancePid.calculate(average, meters);
        
        double turnSpeed = headingPid.calculate(rotation, angle);
        
        double leftOutput = forwardSpeed - turnSpeed;
        double rightOutput = forwardSpeed + turnSpeed;

        double[] desd = desaturateOutput(leftOutput, rightOutput);
        leftOutput = desd[0];
        rightOutput = desd[1];
        

        driveSub.tankDrive(leftOutput, rightOutput);
        logSetpoint.append(meters);

        SmartDashboard.putNumber("Drive/meters Setpoint", meters);
        SmartDashboard.putNumber("Drive/meters traveled", average);

        SmartDashboard.putNumber("Drive/gyro Setpoint", angle);
        SmartDashboard.putNumber("Drive/gyro turned", rotation);

    }
    
    private double[] desaturateOutput(double left, double right)
    {
        double maxAbs = Math.max(Math.abs(left), Math.abs(right));
        
        if (maxAbs > Limits.clampDriveSpeedLimit) {
            double scale = Limits.clampDriveSpeedLimit / maxAbs;
            left *= scale;
            right *= scale;
        }
        
        return new double[] {left, right};
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
