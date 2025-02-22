package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.acmerobotics.dashboard.config.Config;

// r = robot, e = end
// Slides the robot two tiles to the right, shown below:
// |  ^              |
// |  r->>>>>>>>>e   |
// |_________________|

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class LateralBrakingTuner extends DistanceTuner {
    static public int POINTS = 10;

    @Override
    public void runOpMode() {
        initRobot();
        run(90, POINTS);

        while (opModeIsActive()) {
            idle();
        }
        
    }
}