package org.firstinspires.ftc.teamcode.odometry;

import static org.firstinspires.ftc.teamcode.Robot.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.blackIce.DriveCorrections;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;

public final class Odometry {
    public static GoBildaPinpointDriver odometry;

    // Private constructor to prevent instantiation
    private Odometry() {}

    public static void init() {
        odometry = robot.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

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

//    public static double brakingDistance;

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

//        xBrakingDistance = estimateXStoppingDistance();
//        yBrakingDistance = estimateYStoppingDistance();
//        brakingDistance = estimateStoppingDistance();

        double[] robotVelocity =
            DriveCorrections.fieldVectorToRobotVector(new double[]{xVelocity, yVelocity});
        if (Math.abs(robotVelocity[0]) < 0.01) {
            robotVelocity[0] = 0;
        }
        if (Math.abs(robotVelocity[1]) < 0.01) {
            robotVelocity[1] = 0;
        }
        double[] brakingDistances = DriveCorrections.robotVectorToFieldVector(new double[]{
            TuningConstants.FORWARD_BRAKING_DISPLACEMENT.predict(robotVelocity[0]),
            TuningConstants.LATERAL_BRAKING_DISPLACEMENT.predict(robotVelocity[1])
        });
        // xBrakingDisplacement, yBrakingDisplacement

        xBrakingDistance = brakingDistances[0];
        yBrakingDistance = brakingDistances[1];
    }

    public static void setPosition(double startingHeading, double startingX, double startingY) {
        odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startingX,
                startingY,
                AngleUnit.DEGREES,
                startingHeading
        ));
        Target.previousHeading = startingHeading;
        Target.previousX = startingX;
        Target.previousY = startingY;
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