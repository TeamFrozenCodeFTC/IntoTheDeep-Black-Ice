package org.firstinspires.ftc.teamcode.blackIce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.drive.Drive;
import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;

import java.util.function.BooleanSupplier;

/**
 * The main class that initializes autonomous and tele-op modes for Movements to be used.
 * Also has global setting methods.
 */
public class Follower {
    public final static MultipleTelemetry telemetry;
    public final static LinearOpMode opMode;

    public static void waitUntil(BooleanSupplier condition) {
        while (opMode.opModeIsActive() && !condition.getAsBoolean()) {
            opMode.idle();
        }
    }

    /**
     * Set a function that is called once in every loop.
     */
    public static void setGlobalLoopMethod(Runnable loopMethod) {
        Movement.globalLoopMethod = loopMethod;
    }

    /**
     * Initialize the odometry, drivetrain, and telemetry.
     */
    private static void initOpMode(LinearOpMode opMode) {
        Follower.opMode = opMode;
        Odometry.initialize(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        opMode.telemetry = telemetry; // Does this work?
    }

    /**
     * Initialize an autonomous op-mode.
     */
    public static void initAuto(LinearOpMode opMode) {
        initOpMode(opMode);
    }

    /**
     * Initializes the robot for tele-op mode,
     * using the position from the end of the autonomous period.
     */
    public static void initTeleOp(LinearOpMode opMode) {
        double autoEndingX = Odometry.x; // these static variables last between opModes
        double autoEndingY = Odometry.y;
        double autoEndingHeading = Odometry.heading;

        initOpMode(opMode);
        Odometry.setPosition(autoEndingHeading, autoEndingX, autoEndingY);

        Drive.zeroPowerBrakeMode();
    }

    /**
     * Set a condition that will stop macros in tele-op.
     * Default is the gamepad1 left and right sticks,
     * so drivers can immediately have control.
     * <p>
     * Can also be set to another gamepad button, for example
     * {@code setMacroCancelCondition(() -> gamepad.x)}.
     * When true, the macro will cancel.
     */
    public static void setMacroCancelCondition(Condition gamepadCondition) {
        Movement.macroCancelCondition = gamepadCondition;
    }

    /**
     * Run a basic field-centric tele-op. Use this with {@link Follower#initTeleOp}.
     */
    public static void fieldCentricTeleOpDrive(double y, double x, double turn) {
        Target.updatePosition();
        DrivePowers.fromFieldVector(x, y)
            .add(DrivePowers.turnCounterclockwise(turn))
            .normalizeDown()
            .applyPowers();
//        Drive.power(DrivePowers.normalizeDown(DrivePowers.combine( // TODO definitely make this OOP
//            DrivePowers.scaleToMax(DriveVectors.fieldVectorToLocalWheelPowers(
//                x, y
//            ), 1),
//            Drive.turnCounterclockwise(turn)
//        )));
        // TODO here macro reset position in tele-op / heading

    }
    // TODO add tele-Op velocity constraints

    /**
     * Log debug data to telemetry.
     */
    static void telemetryDebug() {
        telemetry.addLine()
            .addData("odometryX", Odometry.x)
            .addData("odometryY", Odometry.y)
            .addData("heading", Odometry.heading)
            .addData("velocity", Odometry.velocity);
    }
}
