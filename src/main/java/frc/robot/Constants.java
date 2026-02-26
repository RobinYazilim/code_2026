package frc.robot;

public class Constants {
    public static final class Motors {
        // TODO: idleri degistir 
        // 1 KOYMAYIN ONEMLI ISLER BOZULUYO (2010lular anlar)
        public static final int leftFrontID = 2;
        public static final int leftBackID = 3;
        public static final int rightFrontID = 4;
        public static final int rightBackID = 5;

        public static final int intakeperfectdaire = 6;
        public static final int intakeperfectsilindir = 7;

        public static final int keremdeniz = 8;
        public static final int john = 9;
    }

    public static final class Limits {
        public static final int voltageLimit = 12;
        public static final int motorCurrentLimit = 40;
        public static final double clampDriveSpeedLimit = 0.45;
        public static final double clampShootSpeedLimit = 0.85;
        public static final double clampIntakeSpeedLimit = 0.45;

        public static final double maxPhysicalSpeedMetersPerSecond = 3.5;

        public static final double joystickSpeedLimit = 0.6;
    }

    public static final class Measurements {
        public static final double distBetweenWheels = 0.6767;
        public static final double wheelDiameter = 0.1524;
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double gearRatio = 8.45;
        public static final double metersPerMotorRotation = wheelCircumference/gearRatio;

        public static final double momentOfInertia = 2.8;

    }

    public static final class IDs {
        public static final int controllerPort = 0;
    }
}
