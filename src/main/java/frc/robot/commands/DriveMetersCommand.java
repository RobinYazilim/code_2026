package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMetersCommand extends Command {
    private final PIDController pid;
    private final DriveSubsystem driveSub;

    private final double meters;
    private double averageStartMeters;

    private final DoubleLogEntry logSetpoint = new DoubleLogEntry(DataLogManager.getLog(), "Drive/Setpoint");


    public DriveMetersCommand(double meters, DriveSubsystem driveSub)
    {
        this.meters = meters;
        this.driveSub = driveSub;

        addRequirements(driveSub);

        pid = new PIDController(.5, 0, 0.05);
        pid.setTolerance(0.03, 0.1);
    }

    @Override
    public void initialize()
    {
        averageStartMeters = driveSub.getAverageMeters();
        pid.reset();
    }

    @Override
    public void execute()
    {
        double currentMeters = driveSub.getAverageMeters() - averageStartMeters;

        double output = pid.calculate(currentMeters, meters);

        output = MathUtil.clamp(output, -Limits.clampSpeedLimit, Limits.clampSpeedLimit);

        driveSub.drive(output, 0.0);
        logSetpoint.append(meters);
    }

    @Override
    public boolean isFinished()
    {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) 
    {
        driveSub.drive(0.0, 0.0);
    }
}
