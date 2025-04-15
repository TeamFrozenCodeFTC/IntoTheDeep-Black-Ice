package org.firstinspires.ftc.teamcode.blackIce.odometry;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.Vector;
import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;

/**
 * A global class that holds the odometry object and provides methods to update the odometry data.
 */
public final class Odometry {
    public final static GoBildaPinpointDriver odometry;

    // Prevents instantiation
    private Odometry() {}

    public static void initialize(HardwareMap hardwareMap) {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-36, 0);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odometry.resetPosAndIMU();

        update();
    }

    public static double heading;
    public static double x;
    public static double y;

    public static double velocity;
    public static double headingVelocity;
    public static double xVelocity;
    public static double yVelocity;

    public static double xBrakingDistance;
    public static double yBrakingDistance;

    public static void update() {
        odometry.update();

        Pose2D pos = odometry.getPosition();
        heading = pos.getHeading(AngleUnit.DEGREES);
        x = pos.getX(DistanceUnit.INCH);
        y = pos.getY(DistanceUnit.INCH);

        Pose2D velocities = odometry.getVelocity();
        xVelocity = velocities.getX(DistanceUnit.INCH);
        yVelocity = velocities.getY(DistanceUnit.INCH);
        velocity = Math.sqrt(Math.pow(Math.abs(xVelocity), 2) + Math.pow(Math.abs(yVelocity), 2));
        headingVelocity = velocities.getHeading(AngleUnit.DEGREES);

//        double[] robotVelocity =
//            DriveVectors.fieldVectorToRobotVector(new double[]{xVelocity, yVelocity});
//        robotVelocity[0] = (Math.abs(robotVelocity[0]) < 0.01) ? 0 : robotVelocity[0];
//        robotVelocity[1] = (Math.abs(robotVelocity[1]) < 0.01) ? 0 : robotVelocity[1];
//        double[] brakingDistances = DriveVectors.robotVectorToFieldVector(new double[]{
//            TuningConstants.FORWARD_BRAKING_DISPLACEMENT.predict(robotVelocity[0]),
//            TuningConstants.LATERAL_BRAKING_DISPLACEMENT.predict(robotVelocity[1])
//        });
        Vector robotVelocity = new Vector(xVelocity, yVelocity).toRobotVector();
        robotVelocity.x = (Math.abs(robotVelocity.x) < 0.01) ? 0 : robotVelocity.x;
        robotVelocity.y = (Math.abs(robotVelocity.y) < 0.01) ? 0 : robotVelocity.y;
        Vector brakingDistances = new Vector(
            TuningConstants.FORWARD_BRAKING_DISPLACEMENT.predict(robotVelocity.x),
            TuningConstants.LATERAL_BRAKING_DISPLACEMENT.predict(robotVelocity.y)
        ).toFieldVector();

        xBrakingDistance = brakingDistances.x;
        yBrakingDistance = brakingDistances.y;
    }

    public static void setPosition(double startingHeading, double startingX, double startingY) {
        odometry.setPosition(new Pose2D(
            DistanceUnit.INCH,
            startingX,
            startingY,
            AngleUnit.DEGREES,
            startingHeading
        ));
        Target.x = startingX;
        Target.y = startingY; // this is being set to the previousY by setTarget
        Target.heading = startingHeading;
//        Target.previousHeading = startingHeading;
//        Target.previousX = startingX;
//        Target.previousY = startingY;
        Target.updatePosition(); //?
    }

    public static boolean pointIsWithinBrakingDistance(double pointX, double pointY) {
        return Vector.getMagnitude(
            pointX - Odometry.x - Odometry.xBrakingDistance,
            pointY - Odometry.y - Odometry.yBrakingDistance
        ) < 1;
    }

    public static void setHeading(double newHeading) {
        setPosition(newHeading, x, y);
    }

    public static void setX(double newX) {
        setPosition(heading, newX, y);
    }

    public static void setY(double newY) {
        setPosition(heading, x, newY);
    }
}