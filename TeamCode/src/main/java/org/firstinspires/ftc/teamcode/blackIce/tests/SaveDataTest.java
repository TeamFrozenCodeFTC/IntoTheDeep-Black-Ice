package org.firstinspires.ftc.teamcode.blackIce.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;

/**
 * Test class for verifying the functionality of the SaveData class.
 * Adds 1 to an integer every time the OpMode is run.
 */
@Autonomous(group="Tests")
public class SaveDataTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(this, new Pose(0,0,0));

        SaveData data = new SaveData(this.hardwareMap.appContext, "testData");

        int testInt;
        if (data.contains("testKey")) {
            testInt = data.getInt("testKey");
        }
        else {
            testInt = 1;
            data.addInt("testKey", testInt);
        }

        follower.telemetry.addData("loadingTestInt", testInt);
        follower.telemetry.update();

        follower.waitUntilOpModeStop();
    }
}