package org.firstinspires.ftc.teamcode.blackIce.util;

import android.util.Log;

import androidx.annotation.Nullable;

import java.lang.reflect.Field;
import java.util.Locale;

public class Logger {
    private static final int LEVEL = Log.DEBUG;
    
    public static void debug(String tag, String key, Object value) {
        if (LEVEL > Log.DEBUG) return;
//        if (value instanceof Number) {
//            value =
//        }
        Log.d("BlackIce" + tag, key + ": " + value);
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
    
    public static void warn(String message) {
        Log.w("BlackIce", message);
    }
    
    public static void warnWithStack(String message) {
        Log.w("BlackIce", message + Log.getStackTraceString(new Throwable()));
    }
    
    public static void enforcePositiveValue(double value, String variableName) {
        if (value <= 0) {
            Logger.warnWithStack(String.format(Locale.US,
                variableName + " is negative (%.2f) when it should be positive.",
                value
            ));
        }
    }
    
    public static void info(String key, Object value) {
        info(key + ": " + value.toString());
    }
    
    public static void info(String message) {
        if (LEVEL > Log.INFO) return;
        Log.i("BlackIce", message);
    }
    
    public static void logFields(String tag, Object obj) {
        if (obj == null) {
            debug(tag, "null object");
            return;
        }
        
        Class<?> clazz = obj.getClass();
        for (Field field : clazz.getDeclaredFields()) {
            field.setAccessible(true);
            try {
                Object value = field.get(obj);
                debug(tag, field.getName() + " = " + value);
            } catch (IllegalAccessException e) {
                debug(tag, "Cannot access field: " + field.getName());
            }
        }
    }
}
