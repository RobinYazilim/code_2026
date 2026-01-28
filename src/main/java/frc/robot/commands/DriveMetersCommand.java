package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMetersCommand extends Command {
    private final PIDController pid;
    private final DriveSubsystem driveSub;

    private final double meters;
    private double averageStartMeters;

    public DriveMetersCommand(double meters, DriveSubsystem driveSub)
    {
        this.meters = meters;
        this.driveSub = driveSub;

        addRequirements(driveSub);
// https://docs.wpilib.org/en/2020/docs/software/advanced-control/introduction/tuning-pid-controller.html
//TODO linke bakın!!!
        pid = new PIDController(.5687, 0.74, 0);
        pid.setTolerance(0.02);
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

        output = Math.max(Math.min(output, 0.6), -0.6);

        driveSub.drive(output, 0.0);
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
