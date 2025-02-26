package org.firstinspires.ftc.teamcode.blackIce;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.Odometry;

public class Robot {
    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        Odometry.init(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);
        // make users be able to add their own init s
    }

    public void loopUpdate() {
        Target.updatePosition();
        // let users put functions in here
    }

    private static Robot robot = null;
    public LinearOpMode opMode;

    private Robot() {

    }

    public static Robot getInstance() {
        if (robot == null) {
            robot = new Robot();
        }
        return robot;
    }

    public static void init(LinearOpMode opMode) {
        Robot robot = Robot.getInstance();
        robot.initialize(opMode);
    }

    public boolean gamepadHasInterrupted() {
        return opMode.gamepad1.dpad_down;
    }

    /**
     * Makes sure opMode is running and that the controller has not canceled the movement.
     */
    public boolean isNotInterrupted() {
        return !gamepadHasInterrupted() && opMode.opModeIsActive();
    }
}
