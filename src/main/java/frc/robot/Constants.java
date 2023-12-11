package frc.robot;

public class Constants {
    public static final class DriveConstants {
        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double DEAD_BAND = 0.05;
        public static final double MAX_SPEED = 0.3;
        public static final double MAX_ANGULAR_SPEED = Math.PI / 2.0;
    }

    public static final class RobotConstants {
        public static final double TRACE_WIDTH = 0.62;
        public static final double WHEEL_BASE = 0.62;
        public static final double GEAR_RATIO = 5.0 / 57.0;
        public static final double WHEEL_RADIUS = 0.0508;
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.0;
    }

    public static final class DeviceIDs {
        public static final class Motors {
            public static final int frontLeftDrive = 2;
            public static final int frontRightDrive = 4;
            public static final int backwardLeftDrive = 5;
            public static final int backwardRightDrive = 7;
            public static final int frontLeftTurn = 1;
            public static final int frontRightTurn = 3;
            public static final int backwardLeftTurn = 6;
            public static final int backwardRightTurn = 8;
        }
        public static final class Encoders {
            public static final int frontLeft = 9;
            public static final int frontRight = 10;
            public static final int backwardLeft = 11;
            public static final int backwardRight = 12;
        }
    }

    public static final class Offsets {
        public static final double FRONT_LEFT = 122.080078125;
        public static final double FRONT_RIGHT = 90.791015625;
        public static final double BACK_LEFT = -88.330078125;
        public static final double BACK_RIGHT = -148.798828125;
    }
}
