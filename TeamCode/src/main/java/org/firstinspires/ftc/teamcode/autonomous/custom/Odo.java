package org.firstinspires.ftc.teamcode.autonomous.custom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Odo extends Autonomous {
    @Override
    public void runOpMode() {
        initOdo();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            x = pos.getX(DistanceUnit.INCH);
            y = pos.getY(DistanceUnit.INCH);
            robotHeading = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", robotHeading);
            double[] powers = normalize(convertToGlobal(
                -robotHeading + 90, // targetHeading?
                -10 - x,
                0 - y));

            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("backLeft", powers[1]);
            telemetry.addData("frontRight", powers[2]);
            telemetry.addData("backRight", powers[3]);

           telemetry.update();
        }

    }
}