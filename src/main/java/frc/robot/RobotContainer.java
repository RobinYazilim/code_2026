// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;
import frc.robot.commands.DriveMetersCommand;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.subsystems.BallSubsystem;
//import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSub;
    private final BallSubsystem ballSub;
    private final CommandPS4Controller controller;


    public RobotContainer() {
        driveSub = new DriveSubsystem();
        ballSub = new BallSubsystem();
        controller = new CommandPS4Controller(IDs.controllerPort);

        configureBindings();
    }

    private void configureBindings() {
        driveSub.setDefaultCommand(
            new RunCommand(
                () -> {
                    double speed = -controller.getLeftY();
                    double rotation = -controller.getRightX();

                    speed = MathUtil.applyDeadband(speed, 0.1);
                    rotation = MathUtil.applyDeadband(rotation, 0.1);

                    speed *= Limits.joystickSpeedLimit;
                    rotation *= Limits.joystickSpeedLimit;

                    driveSub.drive(speed, rotation);
                },
            driveSub));
        /*
        controller.circle().whileTrue(
            new StartEndCommand(
                () -> ballSub.runShooter(Limits.clampSpeedLimit),
                () -> ballSub.runShooter(0),
                ballSub
                )
                );
                
                controller.square().whileTrue(
                    new StartEndCommand(
                        () -> ballSub.runIntake(Limits.clampSpeedLimit),
                        () -> ballSub.runIntake(0),
                        ballSub
                        )
                        );
                        */


                            
    controller.L1().whileTrue(new Intake(ballSub));

    controller.R1().whileTrue(new Eject(ballSub));
    
    controller.R2().whileTrue(new Launch(ballSub));

    //controller.square().whileTrue(new Eject(BallSubsystem));

    //BallSubsystem.setDefaultCommand(BallSubsystem.run(() -> BallSubsystem.stop()));
                    }

    public Command getAutonomousCommand() {
        return new DriveMetersCommand(2, driveSub);
    }
}