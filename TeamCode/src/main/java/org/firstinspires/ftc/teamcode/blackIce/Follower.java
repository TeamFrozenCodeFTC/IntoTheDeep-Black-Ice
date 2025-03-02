package org.firstinspires.ftc.teamcode.blackIce;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;


public class Follower {
    public static MultipleTelemetry telemetry;
    public static LinearOpMode opMode;

    public static void init(LinearOpMode opMode) {
        Follower.opMode = opMode;
        Odometry.initialize(opMode.hardwareMap);
        Drive.init(opMode.hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }

    public static Runnable updateHardwareFunction; // never used
    public static void setUpdate(Runnable updateFunction) {
        updateHardwareFunction = updateFunction;
    }
}
