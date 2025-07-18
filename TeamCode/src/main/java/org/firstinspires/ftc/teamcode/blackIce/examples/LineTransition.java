package org.firstinspires.ftc.teamcode.blackIce.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathBehavior;
import org.firstinspires.ftc.teamcode.blackIce.paths.PathSequence;
import org.firstinspires.ftc.teamcode.blackIce.paths.behavior.TrapezoidalVelocityProfile;
import org.firstinspires.ftc.teamcode.blackIce.paths.geometry.Line;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

@Autonomous(group="Black-Ice Examples")
public class LineTransition extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        
        Follower follower = new Follower(this, new Pose(0,0,0));
        
        Line line = new Line(new Vector(48, 0), new Vector(36, 18));
        Logger.debug("line.getEndTangent()", line.getEndPathPoint().getTangentVector());
        Logger.debug("line.getEndTangent() angle", line.getEndPathPoint().getTangentVector().calculateAngle());
        
        // TODO have .toPoint instead of just lineTo because is prob faster than lines when turning
        //  etc
// PIDF target velocity * 0.015 = 30 = 0.5,
        PathSequence path = new PathSequence()
            .lineTo(48, 0)
            
            //.withBehavior(new PathBehavior().)
            //.lineTo(48, 20).withConstantHeading(90)
            .stop();
        
        while (opModeInInit()) {
            double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }
            Logger.debug("CRASH DEBUG Voltage: ", result);
            if (result < 7) {
                Logger.warn("LOW VOLTAGE WARNING of " + result);
            }
        }
        
        follower.beginFollowing(path);

        while (opModeIsActive()) {
            double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }
            Logger.debug("CRASH DEBUG Voltage: ", result);
            if (result < 7) {
                Logger.warn("LOW VOLTAGE WARNING of " + result);
            }
            follower.update();
        }
    }
}