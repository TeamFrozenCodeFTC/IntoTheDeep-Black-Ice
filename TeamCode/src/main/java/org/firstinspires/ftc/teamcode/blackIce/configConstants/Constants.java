//package org.firstinspires.ftc.teamcode.blackIce.configConstants;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.blackIce.pointToPoint.movement.Function;
//import org.firstinspires.ftc.teamcode.blackIce.localization.Localizer;
//import org.firstinspires.ftc.teamcode.blackIce.localization.localizers.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.blackIce.robot.drivetrain.MecanumDrivetrainConfig;
//
//import java.util.function.DoubleUnaryOperator;
//
//public final class Constants {
//
//    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
//    public static AngleUnit INPUT_ANGLE_UNIT = AngleUnit.DEGREES;
//    // All internal calculations are done in radians, but any unit can be used for input
//
//    public static Function<HardwareMap, Localizer> LOCALIZER = PinpointLocalizer::new;
//
//    public static MecanumDrivetrainConfig DRIVETRAIN_CONFIG = new MecanumDrivetrainConfig(
//        new MotorConfig("frontLeft", DcMotorEx.Direction.REVERSE),
//        new MotorConfig("backLeft", DcMotorEx.Direction.FORWARD),
//        new MotorConfig("frontRight", DcMotorEx.Direction.FORWARD),
//        new MotorConfig("backRight", DcMotorEx.Direction.FORWARD)
//    );
//
//    public static DoubleUnaryOperator inputAngleToRadians = (angle) ->
//        AngleUnit.RADIANS.fromUnit(Constants.INPUT_ANGLE_UNIT, angle);
//
////    public static class Measurement {
////        public static final int TILE = 24;
////        public static final int ROBOT = 18;
////        public static final int HALF_OF_ROBOT = ROBOT / 2;
////        public static final int EDGE_OF_TILE = TILE - ROBOT;
////        public static final double ROBOT_TURN_RADIUS = Math.sqrt(Math.pow(HALF_OF_ROBOT, 2) * 2);
////        public static final double EXTRA_TURN_RADIUS = ROBOT_TURN_RADIUS - HALF_OF_ROBOT;
////        public static final double EDGE_OF_SUBMERSIBLE = 4.25;
////    }
////
////    public static class Curve {
////        public static double INCHES_PER_POINT = 2; // The distance between each point in the curve
////        // (the robot update speed may not be able to keep up with low numbers)
////        public static int LOOK_AHEAD_POINTS_FOR_HEADING = 2; // Looks ahead x points to turn in that direction
////    }
//
//    public static class TurnCorrection {
//        public static final double FINISH_TURN_BY_PERCENT = 0.20;
//        public static final double TURN_POWER = 0.03;
//    }
//}