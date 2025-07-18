package org.firstinspires.ftc.teamcode.blackIce.util;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;

public class Validator {
    public static void throwErrorIf(boolean condition, String errorMessage) {
        if (condition) {
            throw new IllegalArgumentException(errorMessage);
        }
    }
    
    public static void warnIf(boolean condition, String message) {
        if (condition) {
            Logger.warnWithStack(message);
            Follower.getInstance().telemetry.addData("WARNING", message);
        }
    }
    
    public static void warnIfNegative(Double value, String variableName) {
        if (value != null && value <= 0) {
            String message = String.format("%s (%s) must be positive or null.", variableName, value);
            Logger.warnWithStack(message);
            Follower.getInstance().telemetry.addData("WARNING", message);
        }
    }
    
    public static double ensurePositiveSign(double value) {
        return Math.abs(value);
    }
    
    public static double ensureGreaterThanZero(double value) {
        return Math.max(0, value);
    }
}
