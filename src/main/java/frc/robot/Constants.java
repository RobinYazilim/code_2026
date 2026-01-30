package frc.robot;

public class Constants {
    public static final class Motors {
        public static final int leftLeaderID = 7;
        public static final int leftFollowerID = 28;
        public static final int rightLeaderID = 16;
        public static final int rightFollowerID = 8;

        public static final int intakeMotorID = 0;
        public static final int shooterMotorID = 0;
        public static final int shooterMotor1ID = 0; //çıkarazac errorler yüzünden geri koydum
        public static final int shooterMotor2ID = 0;
    }

    public static final class Limits {
        public static final int voltageLimit = 12;
        public static final int motorCurrentLimit = 40;
        public static final double clampSpeedLimit = 0.6;
        public static final double joystickSpeedLimit = 0.6;
    }

    public static final class Measurements {
        public static final double distBetweenWheels = 0.6767;
        public static final double wheelDiameter = 0.1524;
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double gearRatio = 8.45;
        public static final double metersPerMotorRotation = wheelCircumference/gearRatio;
    }

    public static final class IDs {
        public static final int controllerPort = 0;
    }
}
