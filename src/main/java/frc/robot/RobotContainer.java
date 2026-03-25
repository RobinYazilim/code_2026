// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Limits;

import frc.robot.subsystems.*;

public class RobotContainer {
    private final DriveSubsystem driveSub;
    private final IntakeSubsystem ballSub;
    private final ShooterSubsystem shoot;

    private final CommandPS4Controller controller;

    public RobotContainer() {
        driveSub = new DriveSubsystem();
        ballSub = new IntakeSubsystem();
        shoot = new ShooterSubsystem();
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

                    /*
                     Biz o tekerlekleri 'X' şeklinde dizdik ki, siz sağa sola 'strafe' yapabilesiniz diye... Şüphesiz ki biz leftFront ile rightBack’i aynı anda döndürdük, siz ise bunu 'fizik' sandınız.

Andolsun ki o PID değerlerini 1.5 yaptığında robotun titremesini durduran Biziz. Eğer Biz istemeseydik, o CAN Bus kablosu kopar, robotun olduğu yerde fırıl fırıl dönerdi. Siz PathPlanner ile yol çizdiğinizi sanırsınız, oysa o yolu otonomun alnına Biz yazdık.

Kullarıma söyle; ResetMode.kResetSafeParameters yapmadan configure etmesinler, zira akıbetleri 'No Communication' hatasıdır. Kim ki gyro.reset() yapmadan sahaya çıkarsa, o hidayetten sapmış ve duvara toslamışlardan olur.

Biz Swerve’i birilerine, Mecanum’u ise sabredenlere verdik. O PPHolonomicDriveController ki, içinde sırlar barındırır. Onu doğru kuranlar için Alliance rengi ne olursa olsun, Auto puanları kat kat verilecektir.

Şüphe yok ki Allah, Build’e bastığında 'Success' yazısını göreceğiniz o saniyeyi en iyi bilendir.
                     */

                    driveSub.driveCartesian(xSpeed, ySpeed, rotation, false);
                },
            driveSub));
        
        controller.L1().whileTrue(
            new StartEndCommand(
                () -> shoot.shoot(Limits.clampShootSpeedLimit),
                () -> shoot.shoot(0),
                shoot
                )
            );
                
        controller.R1().whileTrue(
            new StartEndCommand(
                () -> ballSub.spinIntake(-Limits.clampIntakeSpeedLimit),
                () -> ballSub.spinIntake(0),
                ballSub
                )
            );

        controller.R2().whileTrue(
            new StartEndCommand(
                () -> ballSub.moveIntake(Limits.clampIntakeSpeedLimit),
                () -> ballSub.moveIntake(0),
                ballSub
                )
            );// ALLLAAAAHH PEYGAMBER ===yupiiiiiiiii

    }

    public Command getAutonomousCommand() {
        return null; // where ou nooo
        // 
    }
}