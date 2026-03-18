// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;
import frc.robot.commands.*;

import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.systems.ComboSystem;

public class RobotContainer {
    private final DriveSubsystem driveSub;
    private final BallSubsystem ballSub;
    private final CameraSubsystem cameraSub;
    private final CommandPS4Controller controller;
    //private final ComboSystem comboSys;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        boolean isCompetition = false;

        driveSub = new DriveSubsystem();
        ballSub = new BallSubsystem();
        cameraSub = new CameraSubsystem(driveSub::addVisionMeasurement);
        controller = new CommandPS4Controller(IDs.controllerPort);
        //comboSys = new ComboSystem(controller);
        
        NamedCommands.registerCommand("ShootAllBalls", new ShootAllBalls(ballSub));
        NamedCommands.registerCommand("IntakeBalls", new IntakeCommand(ballSub));

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );
        
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // new EventTrigger("ShootAllBalls").onTrue(new ShootAllBalls(ballSub));
        // new EventTrigger("IntakeBalls").onTrue(new IntakeCommand(ballSub));
        // bu harbi cok eski ve kotu bi sey named command calistiriyoz zaten

        configureBindings();

        SmartDashboard.putNumber("Tune-Dist_kP", 0.9);
        SmartDashboard.putNumber("Tune-Dist_kI", 0.13);
        SmartDashboard.putNumber("Tune-Dist_kD", 0.5);
        
        SmartDashboard.putNumber("Tune-Head_kP", 0.4);
        SmartDashboard.putNumber("Tune-Head_kI", 0.0);
        SmartDashboard.putNumber("Tune-Head_kD", 0.05);
        
    
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
        
        controller.L1().whileTrue(
            new StartEndCommand(
                () -> ballSub.runShooter(Limits.clampShootSpeedLimit),
                () -> ballSub.runShooter(0),
                ballSub
                )
            );
                
        controller.R1().whileTrue(
            new StartEndCommand(
                () -> ballSub.runIntake(-Limits.clampIntakeSpeedLimit),
                () -> ballSub.runIntake(0),
                ballSub
                )
            );

        controller.R2().whileTrue(
            new StartEndCommand(
                () -> ballSub.runIntake(Limits.clampIntakeSpeedLimit),
                () -> ballSub.runIntake(0),
                ballSub
                )
            );

        /*
        comboSys.registerCommand(
            new StartEndCommand(() -> ballSub.runIntake(Limits.clampIntakeSpeedLimit),
                () -> ballSub.runIntake(0),
                ballSub),
            "Square", "Up", "Down", "Down"
        );
        */

        controller.cross().onTrue(new DriveMetersCommand(1, driveSub));
        controller.circle().onTrue(new rotate90(driveSub));


        
    }

    public Command getAutonomousCommand() {
        System.err.println(NamedCommands.hasCommand("ShootAllBalls"));

        return autoChooser.getSelected();
        //return new DriveMetersCommand(1, driveSub);
    }
}