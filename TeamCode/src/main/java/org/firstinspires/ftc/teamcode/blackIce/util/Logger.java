package org.firstinspires.ftc.teamcode.blackIce.util;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;

public class Logger {
    private static final int LEVEL = Log.DEBUG;
    
    public static void debug(String tag, String key, Object value) {
        if (LEVEL > Log.DEBUG) return;
        Log.d("BlackIce" + tag, key + ": " + value.toString());
    }
    
    public static void debug(String key, Object value) {
        debug("", key, value);
    }
    
    public static void debug(String message) {
        debug("", "", message);
    }
    
    public static void divide(String message) {
        debug("", "", message + " ------------------------");
    }
    
    public static void error(String message) {
        Log.e("BlackIce", message);
    }
    
    public static void warning(String message) {
        Log.w("BlackIce", message);
    }
    
    public static void info(String key, Object value) {
        info(key + ": " + value.toString());
    }
    
    public static void info(String message) {
        if (LEVEL > Log.INFO) return;
        Log.i("BlackIce", message);
    }
}
