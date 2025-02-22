package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.acmerobotics.dashboard.config.Config;

// r = robot, e = end
// Moves the robot two tiles forward, shown below:
// |                 |
// |  r->>>>>>>>>e   |
// |_________________|

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class ForwardBrakingTuner extends DistanceTuner {
    static public int POINTS = 10;

    @Override
    public void runOpMode() {
        initRobot();
        run(0, POINTS);
        
    }
}