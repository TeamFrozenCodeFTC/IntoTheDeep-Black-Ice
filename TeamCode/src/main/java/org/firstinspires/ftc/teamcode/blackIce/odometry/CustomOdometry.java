//package org.firstinspires.ftc.teamcode.blackIce.odometry;
//
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
//import org.firstinspires.ftc.teamcode.blackIce.Target;
//import org.firstinspires.ftc.teamcode.blackIce.Vector;
//import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;
//
//public abstract class CustomOdometry {
//    abstract void initialize(HardwareMap hardwareMap);
//
//    // This method needs to be static to update all of the variables below
//    abstract void updateOdometryAndVariables();
//    public static double heading;
//    public static double x;
//    public static double y;
//    public static double velocity;
//    public static double headingVelocity;
//    public static double xVelocity;
//    public static double yVelocity;
//
//    public static double xBrakingDistance; // Automatically calculated
//    public static double yBrakingDistance;
//
//    void update() {
//        updateOdometryAndVariables();
//
//        double[] robotVelocity =
//            Drive.fieldVectorToRobotVector(new double[]{xVelocity, yVelocity});
//        robotVelocity[0] = (Math.abs(robotVelocity[0]) < 0.01) ? 0 : robotVelocity[0];
//        robotVelocity[1] = (Math.abs(robotVelocity[1]) < 0.01) ? 0 : robotVelocity[1];
//        double[] brakingDistances = Drive.robotVectorToFieldVector(new double[]{
//            TuningConstants.FORWARD_BRAKING_DISPLACEMENT.predict(robotVelocity[0]),
//            TuningConstants.LATERAL_BRAKING_DISPLACEMENT.predict(robotVelocity[1])
//        });
//
//        xBrakingDistance = brakingDistances[0];
//        yBrakingDistance = brakingDistances[1];
//    }
//
//    public static void setPosition(double startingHeading, double startingX, double startingY) {
//        odometry.setPosition(new Pose2D(
//            DistanceUnit.INCH,
//            startingX,
//            startingY,
//            AngleUnit.DEGREES,
//            startingHeading
//        ));
//        Target.x = startingX;
//        Target.y = startingY; // this is being set to the previousY by setTarget
//        Target.heading = startingHeading;
////        Target.previousHeading = startingHeading;
////        Target.previousX = startingX;
////        Target.previousY = startingY;
//        Target.updatePosition(); //?
//    }
//
//    public static boolean pointIsWithinBrakingDistance(double pointX, double pointY) {
//        return Vector.getMagnitude(
//            pointX - Odometry.x - Odometry.xBrakingDistance,
//            pointY - Odometry.y - Odometry.yBrakingDistance
//        ) < 1;
//    }
//
//    public static void setHeading(double newHeading) {
//        setPosition(newHeading, x, y);
//    }
//
//    public static void setX(double newX) {
//        setPosition(heading, newX, y);
//    }
//
//    public static void setY(double newY) {
//        setPosition(heading, x, newY);
//    }
//}