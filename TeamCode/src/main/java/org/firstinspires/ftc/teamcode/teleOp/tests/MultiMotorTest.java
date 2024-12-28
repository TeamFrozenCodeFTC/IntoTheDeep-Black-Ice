package org.firstinspires.ftc.teamcode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.LinearEquation;

@TeleOp
public class MultiMotorTest extends GeneralTest {

    class MotorTester implements ComponentTester {
        DcMotor motor;

        @Override
        public void setComponent(String name) {
            motor = hardwareMap.get(DcMotor.class, name); motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void test() {
            double power = gamepad1.left_stick_y;

            motor.setPower(power);
            telemetry.addData("Motor Power", power);
            telemetry.addData("ticks", motor.getCurrentPosition());
        }
    }
// -1098,
    String[] components = {
//            "frontLeft",
//            "backLeft",
//            "frontRight",
//            "backRight",
            "intakeMotor"
    };
    ComponentTester tester = new MotorTester();

    public String[] getComponents() {
        return components;
    }

    public ComponentTester getTester() {
        return tester;
    }
}