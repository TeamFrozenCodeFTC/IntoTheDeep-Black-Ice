package org.firstinspires.ftc.teamcode.blackIce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;


public class Follower {
    public static MultipleTelemetry telemetry;
    public static LinearOpMode opMode;

    public static void initAuto(LinearOpMode opMode) {
        Follower.opMode = opMode;
        Odometry.initialize(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    public static Runnable updateHardwareFunction; // TODO never used
    public static void setUpdate(Runnable updateFunction) {
        updateHardwareFunction = updateFunction;
    }

    public static void fieldCentricTeleOpDrive(double y, double x, double turn) {
        Target.updatePosition();
        Drive.power(Drive.combineMax(
            Vector.scaleToMax(Drive.fieldVectorToLocalWheelPowers(
                x, y
            ), 1),
            Drive.turnCounterclockwise(turn),
            1
        ));
    }

    /**
     * Initializes the robot for tele-op mode,
     * using the position from the end of the autonomous period.
     */
    public static void initTeleOp(LinearOpMode opMode) {
        double autoEndingX = Odometry.x; // these static variables last between opModes
        double autoEndingY = Odometry.y;
        double autoEndingHeading = Odometry.heading;

        initAuto(opMode);
        Odometry.setPosition(autoEndingHeading, autoEndingX, autoEndingY);
    }
}
