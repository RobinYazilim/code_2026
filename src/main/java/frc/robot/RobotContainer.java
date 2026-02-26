// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;

import frc.robot.subsystems.BallSubsystem;
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
                    double xSpeed = controller.getLeftX();
                    double ySpeed = controller.getLeftY();
                    double rotation = controller.getRightX();

                    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
                    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
                    rotation = MathUtil.applyDeadband(rotation, 0.1);

                    xSpeed *= Limits.joystickSpeedLimit;
                    ySpeed *= Limits.joystickSpeedLimit;
                    rotation *= Limits.joystickSpeedLimit;

                    driveSub.drive(xSpeed, ySpeed, rotation);
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
        return null;
    }
}