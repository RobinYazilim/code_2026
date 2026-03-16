package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.DriveSubsystem;

public class rotate90 extends Command {
    private final PIDController headingPid;

    private final DriveSubsystem driveSub;

    private final double angle;
    private double startAngle;

    private final DoubleLogEntry logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Angle Setpoint");


    public rotate90(DriveSubsystem driveSub)
    {
        this.angle = 90;
        this.driveSub = driveSub;

        addRequirements(driveSub);


        double headKp = 0.04;
        double headKi = 0.0;
        double headKd = 0.0;
        headingPid = new PIDController(headKp, headKi, headKd);
        headingPid.setTolerance(1);
    }
        

    @Override
    public void initialize()
    {
        
        headingPid.reset();

        startAngle = driveSub.getGyroValue();

        SmartDashboard.putNumber("Drive/Angle Setpoint", angle);

    }

    @Override
    public void execute()
    {

        double headKp = SmartDashboard.getNumber("Tune-Head_kP", 0.04);
        double headKi = SmartDashboard.getNumber("Tune-Head_kI", 0.0);
        double headKd = SmartDashboard.getNumber("Tune-Head_kD", 0.0);

        headingPid.setPID(headKp, headKi, headKd);

        double rotation = driveSub.getGyroValue() - startAngle;

        double turnSpeed = headingPid.calculate(rotation, angle);
        
        double leftOutput = -turnSpeed;
        double rightOutput = turnSpeed;

        double[] desd = desaturateOutput(leftOutput, rightOutput);
        leftOutput = desd[0];
        rightOutput = desd[1];

        

        driveSub.tankDrive(leftOutput, rightOutput);
        logSetpoint.append(angle);

        SmartDashboard.putNumber("Drive/Angle Setpoint", angle);
        SmartDashboard.putNumber("Drive/degrees left", Math.abs(angle - driveSub.getGyroValue()));

        
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
        return headingPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) 
    {
        String debugMessage = "rotate90 (ID: " + this.hashCode() + 
                              ") ended. Interrupted = " + interrupted;
        
        DataLogManager.log(debugMessage);
        driveSub.tankDrive(0.0, 0.0);
    }
}
