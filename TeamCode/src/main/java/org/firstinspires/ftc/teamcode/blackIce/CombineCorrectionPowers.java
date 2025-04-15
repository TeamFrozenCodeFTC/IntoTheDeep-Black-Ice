package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;

public interface CombineCorrectionPowers {
    DrivePowers combineCorrectionPowers(DrivePowers driveCorrection, DrivePowers headingCorrection);
}
