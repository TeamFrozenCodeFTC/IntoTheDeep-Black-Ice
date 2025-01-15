package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EDGE_OF_TILE;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.EXTRA_TURN_RADIUS;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.HALF_OF_ROBOT;
import static org.firstinspires.ftc.teamcode.blackIce.Constants.Measurement.TILE;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.Specimen4Math;

public class NewDriveControls {
    Robot robot;

    double x = -1;

    public NewDriveControls(Robot robot) {
        this.robot = robot;
    }

    double powerEquation(double power) {
        return Math.pow(power, 3);
    }

    void control() {
        if (robot.gamepad1.triangle) {
            robot.odometry.setPosition(90,robot.odometry.x, robot.odometry.y);
        }
        // Reset Odometry in observation zone corner
        else if (robot.gamepad1.square) {
            robot.odometry.setPosition(90,24*3-9,0);
        }
        else if (robot.gamepad1.circle) {
            robot.odometry.setPosition(90,24*3-9,0);
        }
        else if (robot.gamepad1.dpad_up) {
            robot.movement.quickBrakeTo(90, TILE + HALF_OF_ROBOT, 3, 10);

            robot.movement.backIntoWall(0.3);

            robot.odometry.setY(0);

            robot.viperSlide.clawGrab();
            robot.sleep(200);
            robot.viperSlide.bottomBasketRaise();
            robot.sleep(250);

            robot.viperSlide.upperChamberRaise();
            robot.movement.quickBrakeTo(-90, x, 28, 10);

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
//        else if (robot.gamepad1.dpad_down) {
//            robot.intake.retract();
//            robot.movement.moveTo(-90+25, TILE*2, TILE);
//            robot.intake.armOut();
//            robot.sleep(250);
//            robot.intake.spinSweeperOut();
//            robot.sleep(250);
//            robot.intake.retract();
//        }
//        else if (robot.gamepad1.dpad_left) {
//            robot.viperSlide.upperBasketRaise();
//            robot.movement.quickBrakeTo(
//                45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 3,
//                EXTRA_TURN_RADIUS, 5);
//            robot.viperSlide.waitForExtension();
//            robot.viperSlide.dump();
//            robot.sleep(750);
//            robot.movement.moveTo(
//                45, -TILE*2 - HALF_OF_ROBOT - EDGE_OF_TILE + EXTRA_TURN_RADIUS + 6,
//                EXTRA_TURN_RADIUS + 6);
//            robot.viperSlide.lower();
//        }

        double speedFactor;

        if (robot.gamepad1.right_bumper) {
            speedFactor = 0.3;
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

        if (!firstInput && (xStick != 0 || yStick != 0)) {
            firstInput = true;
            robot.timer.reset();
        }

        return robot.drive.fieldVectorToLocalWheelPowers(xStick, yStick);
    }
}
