package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.blackIceX.RobotMovement;

import java.util.ArrayList;

public class RelativeWheelControls {
    RobotMovement op;
    boolean firstInput = false;

    public RelativeWheelControls(RobotMovement op) {
        this.op = op;
    }

    ArrayList<Double[][]> controls = new ArrayList<>();

    static double SPEED_FACTOR = 1;

    double powerEquation(double power) {
        return Math.pow(power, 3);
    }

    private double getRadians() {
        op.odometry.update();
        return op.odometry.heading * Math.PI / 180;
    }

    void control() {
        if (op.gamepad1.triangle) {
            op.odometry.resetHeading();
        }

        if (op.gamepad1.dpad_up) {
            op.viperSlide.topBarRaise();
            op.brakeToPosition(-90, 0, 30, op.wideErrorMargin);

            op.viperSlide.waitForExtension();

            // While neither touch sensors are pressed...
            do {
                op.targetY = 24+24-(24-18);
                op.updatePosition();
                op.goStraight(0.4);
            } while (!op.touchRight.isPressed() && !op.touchLeft.isPressed());

            op.viperSlide.topBarPull();
            op.odometry.setHeading(-90);
            //odometry.setY(31);
            op.viperSlide.waitForExtension();
            op.viperSlide.clawOut();
            op.viperSlide.lower();
        }

        relativeSlide();
        pivot();
        slide();

        if (op.gamepad1.right_bumper) {
            SPEED_FACTOR = 0.3;
        }
        else {
            SPEED_FACTOR = 1;
        }

        double frontLeftPower  = 0;
        double backLeftPower   = 0;
        double backRightPower  = 0;
        double frontRightPower = 0;
        for (Double[][] control : controls) {
            frontLeftPower  += powerEquation(control[0][0]) * SPEED_FACTOR;
            backLeftPower   += powerEquation(control[1][0]) * SPEED_FACTOR;
            backRightPower  += powerEquation(control[1][1]) * SPEED_FACTOR;
            frontRightPower += powerEquation(control[0][1]) * SPEED_FACTOR;
        }

        op.frontLeftWheel.setPower(frontLeftPower);
        op.backLeftWheel.setPower(backLeftPower);
        op.frontRightWheel.setPower(frontRightPower);
        op.backRightWheel.setPower(backRightPower);

        controls.clear();
    }

    void pivot() {
        double pivot = op.gamepad1.right_stick_x;

        controls.add(new Double[][] {
                {pivot, -pivot},
                {pivot, -pivot}
        });
    }

    void relativeSlide() {
        double xStick = -op.gamepad1.left_stick_x;
        double yStick = -op.gamepad1.left_stick_y;

        double xPower;
        if (Math.abs(xStick) < 0.5) {
            xPower = 0;
        }
        else {
            xPower = xStick * 0.75;
        }

        double yPower;
        if (Math.abs(yStick) < 0.5) {
            yPower = 0;
        }
        else {
            yPower = yStick * 0.75;
        }

        if (!firstInput && (xStick != 0 || yStick != 0)) {
            firstInput = true;
            op.timer.reset();
        }

        double radians = getRadians();

        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double x = xPower * cos - yPower * sin;
        double y = xPower * sin + yPower * cos;

        controls.add(new Double[][] {
                {y-x, y+x},
                {y+x, y-x}
        });
        op.telemetry.addData("xStick", xStick);
        op.telemetry.addData("yStick", yStick);
    }

    void slide() {
        double x = (-op.gamepad1.right_trigger + op.gamepad1.left_trigger) * 0.7;

        controls.add(new Double[][] {
                {-x, +x},
                {+x, -x}
        });
    }
}
