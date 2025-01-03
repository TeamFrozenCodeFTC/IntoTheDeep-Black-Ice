package org.firstinspires.ftc.teamcode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.LinearEquation;

@TeleOp(group="Tests")
public class MultiServoTest extends GeneralTest {

    class ServoTester implements ComponentTester {
        LinearEquation servoMap;
        Servo servo;

        public ServoTester() {
            servoMap = new LinearEquation(1, 1, -1, 0);
        }

        @Override
        public void setComponent(String name) {
            servo = hardwareMap.get(Servo.class, name);
        }

        @Override
        public void test() {
            double pos = servoMap.solve(gamepad1.left_stick_y);
            servo.setPosition(pos);
            telemetry.addData("Servo Position", pos);
        }
    }

    String[] components = {
            "sweeperRotator",
            "dumperServo",
            "clawLeft",
            "clawRight"
    };
    ComponentTester tester = new ServoTester();

    public String[] getComponents() {
        return components;
    }

    public ComponentTester getTester() {
        return tester;
    }
}