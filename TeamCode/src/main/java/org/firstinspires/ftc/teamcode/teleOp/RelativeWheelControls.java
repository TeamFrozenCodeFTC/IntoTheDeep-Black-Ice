package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class RelativeWheelControls {
    Robot op;

    public RelativeWheelControls(Robot op) {
        this.op = op;
    }

    ArrayList<Double[][]> controls = new ArrayList<>();

    static double SPEED_FACTOR = 0.7;

    double powerEquation(double power) {
        return Math.pow(power, 3);
    }

    private double getRadians() {
        op.odometry.update();
        return op.odometry.getHeading();
    }

    void control() {
        if (op.gamepad1.triangle) {
            op.resetRotation();
        }

        relativeSlide();
        pivot();

        if (op.gamepad1.right_bumper) {
            SPEED_FACTOR = 0.2;
        }
        else {
            SPEED_FACTOR = 0.7;
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

        double radians = getRadians();

        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double x = xStick * cos - yStick * sin;
        double y = xStick * sin + yStick * cos;

        controls.add(new Double[][] {
                {y-x, y+x},
                {y+x, y-x}
        });
        op.telemetry.addData("xStick", xStick);
        op.telemetry.addData("yStick", yStick);
    }
}
