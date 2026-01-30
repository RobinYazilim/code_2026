// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;
import frc.robot.commands.DriveMetersCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSub;
    private final CommandPS4Controller controller;

    public RobotContainer() {
        driveSub = new DriveSubsystem();
        controller = new CommandPS4Controller(IDs.controllerPort);

        configureBindings();
    }

    private void configureBindings() {
        driveSub.setDefaultCommand(
            new RunCommand(
                () -> {
                    double speed = -controller.getLeftY();
                    double rotation = controller.getRightX();

                    speed *= Limits.joystickSpeedLimit;
                    rotation *= Limits.joystickSpeedLimit;

                    driveSub.drive(speed, rotation);
                },
            driveSub));
        
        controller.circle().onTrue(new DriveMetersCommand(2, driveSub));
    }

    public Command getAutonomousCommand() {
        return new DriveMetersCommand(2, driveSub);
    }
}
 //apply deadband yapcaz!!