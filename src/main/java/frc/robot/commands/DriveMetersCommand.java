package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMetersCommand extends Command {
    private final PIDController pidLeft;
    private final PIDController pidRight;

    private final DriveSubsystem driveSub;

    private final double meters;
    private double leftStartMeters;
    private double rightStartMeters;

    private final DoubleLogEntry logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Setpoint");

    double angle = 0;


    public DriveMetersCommand(double meters, boolean lockAngle, DriveSubsystem driveSub)
    {
        this.meters = meters;
        this.driveSub = driveSub;

        addRequirements(driveSub);

        double kp = 1.2;
        double ki = 1;
        //0.0205
        double kd = 0.15;

        if (lockAngle)
            angle = driveSub.getGyroValue();

        pidLeft = new PIDController(kp, ki, kd);
        pidRight = new PIDController(kp, ki, kd);
        
        pidLeft.setTolerance(0.03, 0.1);
        pidRight.setTolerance(0.03, 0.1);
    }

    @Override
    public void initialize()
    {
        leftStartMeters = driveSub.getLeftMeters();
        rightStartMeters = driveSub.getRightMeters();
        
        pidLeft.reset();
        pidRight.reset();
    }

    @Override
    public void execute()
    {
        double left = driveSub.getLeftMeters() - leftStartMeters;
        double right = driveSub.getRightMeters() - rightStartMeters;
        double rotation = driveSub.getGyroValue() - angle;

        double leftOutput = pidLeft.calculate(left, meters) + rotation;
        double rightOutput = pidRight.calculate(right, meters) - rotation;

        leftOutput = MathUtil.clamp(leftOutput, -Limits.clampDriveSpeedLimit, Limits.clampDriveSpeedLimit);
        rightOutput = MathUtil.clamp(rightOutput, -Limits.clampDriveSpeedLimit, Limits.clampDriveSpeedLimit);

        driveSub.tankDrive(leftOutput, rightOutput);
        logSetpoint.append(meters);
    }

    @Override
    public boolean isFinished()
    {
        return pidLeft.atSetpoint() && pidRight.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) 
    {
        DataLogManager.log("DriveMetersCommand has ended.");
        System.out.println("DriveMetersCommand has ended.");
        driveSub.tankDrive(0.0, 0.0);
    }
}
