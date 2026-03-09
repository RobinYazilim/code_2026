package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.BallSubsystem;

public class ShootAllBalls extends Command {

    private final BallSubsystem ballSub;

    private final Timer timer = new Timer();

    public ShootAllBalls(BallSubsystem ballSub) {
        this.ballSub = ballSub;
        addRequirements(ballSub);
    }

    @Override
    public void initialize() 
    {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() 
    {
        ballSub.runShooter(Limits.clampShootSpeedLimit);
    }
    
    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(5);
    }

    @Override
    public void end(boolean interrupted)
    {
        ballSub.runShooter(0);
    }
}
