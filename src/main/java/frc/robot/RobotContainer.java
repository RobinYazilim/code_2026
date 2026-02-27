// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;
import frc.robot.commands.DriveMetersCommand;

import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSub;
    private final BallSubsystem ballSub;
    private final CommandPS4Controller controller;

    private final PathPlannerAuto auto;
    private final SendableChooser<Command> autoChooser;



    public RobotContainer() {
        boolean isCompetition = true;

        driveSub = new DriveSubsystem();
        ballSub = new BallSubsystem();
        controller = new CommandPS4Controller(IDs.controllerPort);

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );
        
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        auto = loadAutos();

        SmartDashboard.putNumber("Tune-Dist_kP", 0.9);
        SmartDashboard.putNumber("Tune-Dist_kI", 0.05);
        SmartDashboard.putNumber("Tune-Dist_kD", 0.1);
        
        SmartDashboard.putNumber("Tune-Head_kP", 0.04);
        SmartDashboard.putNumber("Tune-Head_kI", 0.0);
        SmartDashboard.putNumber("Tune-Head_kD", 0.0);
    
    }

    private PathPlannerAuto loadAutos() {
        return new PathPlannerAuto("abc");
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
    }

    public Command getAutonomousCommand() {
        if (autoChooser != null)
        {
            return autoChooser.getSelected();
        }
        if (auto != null)
        {
            System.err.println("got das auto");
            return auto;
        }
        return new DriveMetersCommand(1, driveSub);
    }
}