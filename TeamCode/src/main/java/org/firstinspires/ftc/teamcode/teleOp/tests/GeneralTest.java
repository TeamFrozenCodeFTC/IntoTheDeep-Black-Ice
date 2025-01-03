package org.firstinspires.ftc.teamcode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GeneralTest extends LinearOpMode {
    public abstract String[] getComponents();

    public abstract ComponentTester getTester();

    @Override
    public void runOpMode() throws InterruptedException {

        String[] componentNames = getComponents();
        ComponentTester compTest = getTester();

        waitForStart();

        boolean downProcessed = false;
        boolean upProcessed = false;
        int currentIndex = 0;
        String currentComponent = componentNames[currentIndex];
        compTest.setComponent(currentComponent);

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                if (!downProcessed) {
                    downProcessed = true;

                    currentIndex++;
                    if (currentIndex >= componentNames.length) {
                        currentIndex = 0;
                    }
                    currentComponent = componentNames[currentIndex];
                    compTest.setComponent(currentComponent);
                }
            } else {
                downProcessed = false;
            }

            if (gamepad1.dpad_up) {
                if (!upProcessed) {
                    upProcessed = true;

                    currentIndex--;
                    if (currentIndex < 0) {
                        currentIndex = componentNames.length - 1;
                    }
                    currentComponent = componentNames[currentIndex];
                    compTest.setComponent(currentComponent);
                }
            } else {
                upProcessed = false;
            }

            telemetry.addData("Testing", currentComponent);
            compTest.test();
            telemetry.update();
        }
    }
}