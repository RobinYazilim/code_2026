package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Limits;
import frc.robot.subsystems.BallSubsystem;

public class IntakeCommand extends Command {

    private final BallSubsystem ballsub;

    private final Timer timer = new Timer();

    public IntakeCommand(BallSubsystem ballSub) {
        this.ballsub = ballSub;
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
        ballsub.runIntake(Limits.clampIntakeSpeedLimit);
    }
    
    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(4); //robotun orta alanda top alması için kaç saniye gerek deniyelim ve değiştirelim
    }

    @Override
    public void end(boolean interrupted)
    {
        ballsub.runIntake(0);
    }
}
