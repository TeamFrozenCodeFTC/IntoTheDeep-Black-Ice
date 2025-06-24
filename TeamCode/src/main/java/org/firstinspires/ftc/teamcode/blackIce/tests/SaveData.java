package org.firstinspires.ftc.teamcode.blackIce.tests;

import android.content.Context;
import android.content.SharedPreferences;

public class SaveData {
    private final SharedPreferences prefs;
    private SharedPreferences.Editor editor;

    public SaveData(Context context, String name) {
        this.prefs = context.getSharedPreferences(name, Context.MODE_PRIVATE);
    }

    public SharedPreferences.Editor startAdding() {
        this.editor = prefs.edit();
        return this.editor;
    }

    public void save() {
        this.editor.apply();
    }

    public void addInt(String key, int value) {
        editor.putInt(key, value);
    }

    public void addFloat(String key, float value) {
        editor.putFloat(key, value);
    }

    public void addLong(String key, long value) {
        editor.putLong(key, value);
    }

    public void addBoolean(String key, boolean value) {
        editor.putBoolean(key, value);
    }

    public void addString(String key, String value) {
        editor.putString(key, value);
    }

    public int getInt(String key) {
        return prefs.getInt(key, 0);
    }

    public float getFloat(String key) {
        return prefs.getFloat(key, 0);
    }

    public long getLong(String key) {
        return prefs.getLong(key, 0);
    }

    public boolean getBoolean(String key) {
        return prefs.getBoolean(key, false);
    }

    public String getString(String key) {
        return prefs.getString(key, "");
    }

    public boolean contains(String key) {
        return prefs.contains(key);
    }

    public void delete() {
        prefs.edit().clear().apply();
    }
}
