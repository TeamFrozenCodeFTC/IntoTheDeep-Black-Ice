package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.DriveCorrections;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(group="Tests")
public class MoreAccurateStopAt extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        clawLeft.getController().pwmDisable();

        Odometry.setPosition(0, 0, 0);

        new Movement(-48, 0, 0)
            .stopAtPosition()
            .setDriveCorrection(() -> DriveCorrections.fieldVectorToLocalWheelPowers(new double[]{
                (Target.xError - Odometry.xBrakingDistance),
                (Target.yError - Odometry.yBrakingDistance) // * 1 higher becomes more unstable but more accurate, lower becomes more stable but less accurate
            }))
            .setMovementExit(() -> false)
            .runTimeout(999);
    }
}// TODO try turn or drive
//