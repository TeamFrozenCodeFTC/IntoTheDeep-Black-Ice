package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;

public class NewDriveControls {
    Robot robot;

    double x = -1;

    public NewDriveControls(Robot robot) {
        this.robot = robot;
    }

    double powerEquation(double power) {
        return Math.pow(power, NewControls.POWER);
    }

    void control() {
        if (robot.gamepad1.triangle) {
            robot.odometry.setPosition(90,robot.odometry.x, robot.odometry.y);
        }
        // Reset Odometry in observation zone corner
        else if (robot.gamepad1.circle) {
            robot.odometry.setPosition(90,24*3-9,0);
        }
        else if (robot.gamepad1.cross) {
            robot.viperSlide.liftRobot();
        }
        else if (robot.gamepad1.square) {
            robot.viperSlide.liftersIn();
        }
        else if (robot.gamepad1.dpad_up) {
            robot.movement.stopAtPosition(90, TILE + HALF_OF_ROBOT, 3);

            robot.movement.backIntoWall(0.3);

            robot.odometry.setY(0);

            robot.viperSlide.clawGrab();
            robot.sleep(200);
            robot.viperSlide.bottomBasketRaise();
            robot.sleep(250);

            robot.viperSlide.upperChamberRaise();
            robot.movement.stopAtPosition(-90, x, 28);

            robot.viperSlide.waitForExtension();

            // While neither touch sensors are pressed...
            robot.movement.backIntoWall(0.3);

            robot.odometry.setHeading(-90);

            robot.viperSlide.upperChamberPull();
            robot.viperSlide.waitForExtension();
            robot.viperSlide.clawOut();
            robot.viperSlide.lower();

            x += 1.5;
        }

        double speedFactor;

        if (robot.gamepad1.right_bumper) {
            speedFactor = 0.4;
        }
        else {
            speedFactor = 1;
        }

        robot.drive.power(
            robot.drive.multiply(
                robot.drive.combine(
                    pivot(),
                    controllerRelativeMovement()
                ),
                speedFactor
            )
        );
    }

    private double[] pivot() {
        double power = powerEquation(robot.gamepad1.right_stick_x);

        return robot.drive.turnClockwise(power);
    }

    boolean firstInput = false;

    private double[] controllerRelativeMovement() {
        double xStick = powerEquation(robot.gamepad1.left_stick_x);
        double yStick = -powerEquation(robot.gamepad1.left_stick_y);

//        robot.gamepad1.rumble(Math.max(xStick, yStick), Math.max(xStick, yStick), 100);

        if (!firstInput && (xStick != 0 || yStick != 0)) {
            firstInput = true;
            robot.timer.reset();
        }

        return robot.drive.fieldVectorToLocalWheelPowers(xStick, yStick);
    }
}
