package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.ArrayList;

public class DriveControls {
    Robot op;
    boolean firstInput = false;

    public DriveControls(Robot op) {
        this.op = op;
    }

    ArrayList<double[]> controls = new ArrayList<>();

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
        if (op.gamepad1.square) {
            op.odometry.setPosition(0,0,0);
        }

        if (op.gamepad1.dpad_up) {
            op.viperSlide.topBarRaise();
            op.movement.quickBrakeTo(-90, 0, 30, 10);

            op.viperSlide.waitForExtension();

            // While neither touch sensors are pressed...
            op.movement.backIntoWall(0.3);

            op.viperSlide.topBarPull();
            //odometry.setHeading(-90);
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
        for (double[] control : controls) {
            frontLeftPower  += powerEquation(control[0]) * SPEED_FACTOR;
            backLeftPower   += powerEquation(control[1]) * SPEED_FACTOR;
            backRightPower  += powerEquation(control[2]) * SPEED_FACTOR;
            frontRightPower += powerEquation(control[3]) * SPEED_FACTOR;
        }

        op.frontLeftWheel.setPower(frontLeftPower);
        op.backLeftWheel.setPower(backLeftPower);
        op.frontRightWheel.setPower(frontRightPower);
        op.backRightWheel.setPower(backRightPower);

        controls.clear();
    }

    void pivot() {
        double pivot = op.gamepad1.right_stick_x;

        controls.add(new double[] {
                pivot, -pivot,
                pivot, -pivot
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
        double x = (xPower * cos - yPower * sin) * 65;
        double y = (xPower * sin + yPower * cos) * 58;

        controls.add(Util.normalize(new double[] {
                y-x, y+x,
                y+x, y-x
        }));


//        controls.add(new Double[][] {
//                {y-x, y+x},
//                {y+x, y-x}
//        });
        op.telemetry.addData("xStick", xStick);
        op.telemetry.addData("yStick", yStick);
    }

    double[] preRelativeSlide() {
        double xStick = -op.gamepad1.left_stick_x;
        double yStick = -op.gamepad1.left_stick_y;

        if (!firstInput && (xStick != 0 || yStick != 0)) {
            firstInput = true;
            op.timer.reset();
        }

        double radians = getRadians();

        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double x = (xStick * cos - yStick * sin) * 65;
        double y = (xStick * sin + yStick * cos) * 58;

        return new double[] {
                y-x, y+x,
                y+x, y-x
        };
    }

    void slide() {
        double x = (-op.gamepad1.right_trigger + op.gamepad1.left_trigger) * 0.7;

        controls.add(new double[] {
                -x, +x,
                +x, -x
        });
    }
}
