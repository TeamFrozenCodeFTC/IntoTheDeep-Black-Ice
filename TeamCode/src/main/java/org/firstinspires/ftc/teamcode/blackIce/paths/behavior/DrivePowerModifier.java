package org.firstinspires.ftc.teamcode.blackIce.paths.behavior;

@FunctionalInterface
public interface DrivePowerModifier {
    double modifyDrivePower(double drivePower, boolean isDecelerating);
    
    DrivePowerModifier none = (drivePower, isDecelerating) -> drivePower;
    
    DrivePowerModifier decelerateWithZeroPower =
        (drivePower, isDecelerating) -> (isDecelerating) ? 0 : drivePower;
    
    DrivePowerModifier preventReverseDeceleration =
        (drivePower, isDecelerating) -> Math.max(0, drivePower);
    
    default DrivePowerModifier combineWith(DrivePowerModifier modifier) {
        return (power, isDecelerating) -> modifier.modifyDrivePower(
            this.modifyDrivePower(power, isDecelerating), isDecelerating);
    }
}
