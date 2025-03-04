package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blackIce.Follower;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class LateralBrakingTuner extends DistanceTuner {
    @Override
    public void runOpMode() {
        Follower.init(this);

        run(90);
    }
}