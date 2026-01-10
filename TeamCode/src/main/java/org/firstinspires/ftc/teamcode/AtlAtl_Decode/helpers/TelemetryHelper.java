package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedHashMap;
import java.util.Map;

public class TelemetryHelper {
    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;
    private final String title;

    private final Map<String, Object> entries = new LinkedHashMap<>();
    private final Map<String, TelemetryHelper> children = new LinkedHashMap<>();

    private long timerStart = 0;
    public static TelemetryHelper create(Telemetry dsTelemetry, String title) {
        Telemetry dash = null;
        try {
            dash = FtcDashboard.getInstance().getTelemetry();
        } catch (Exception ignored) {
        }
        return new TelemetryHelper(dsTelemetry, dash, title);
    }
    private TelemetryHelper(Telemetry dsTelemetry, Telemetry dashTelemetry, String title) {
        this.dsTelemetry = dsTelemetry;
        this.dashTelemetry = dashTelemetry;
        this.title = title;
    }
    //basic add
    public void add(String key, Object value) {
        entries.put(key, value);
    }
    //format add
    public void addf(String key, String format, Object... args) {
        entries.put(key, String.format(format, args));
    }

    /// removing stuff
    //removes a single entry
    public void remove(String key) {
        entries.remove(key);
    }

    // removes a child group by name
    public void removeChild(String name) {
        children.remove(name);
    }

    // clear only entries (not children)
    public void clearEntries() {
        entries.clear();
    }

    // clear only children (not entries)
    public void clearChildren() {
        children.clear();
    }


    // nested group
    public TelemetryHelper child(String name) {
        return children.computeIfAbsent(name, k -> new TelemetryHelper(dsTelemetry, dashTelemetry, k));
    }
    /// timer helpers
    public void startTimer() {
        timerStart = System.nanoTime();
    }
    public double elapsed() {
        return (System.nanoTime() - timerStart) / 1e6; // ms
    }
    public double lap() {
        long now = System.nanoTime();
        double ms = (now - timerStart) / 1e6;
        timerStart = now;
        return ms;
    }

    public void clear() {
        entries.clear();
        children.values().forEach(TelemetryHelper::clear);
    }

    public void push() {
        pushInternal(0);
    }

    private void pushInternal(int indent) {
        String indentStr = indent(indent);

        //light seperator
        dsTelemetry.addLine(indentStr + "+-- " + title + " --+");
        if (dashTelemetry != null)
            dashTelemetry.addLine(indentStr + "[" + title + "]");

        //alignment for DS
        int maxKeyLen = 0;
        int maxValLen = 0;

        for (Map.Entry<String, Object> e : entries.entrySet()) {
            maxKeyLen = Math.max(maxKeyLen, e.getKey().length());
            maxValLen = Math.max(maxValLen, String.valueOf(e.getValue()).length());
        }

        // push ds aligned
        for (Map.Entry<String, Object> e : entries.entrySet()) {
            String key = e.getKey();
            String val = String.valueOf(e.getValue());

            String paddedKey = String.format("%-" + maxKeyLen + "s", key);
            String paddedVal = String.format("%" + maxValLen + "s", val);

            dsTelemetry.addLine(indentStr + "  " + paddedKey + " : " + paddedVal);
        }

        //dash gets raw(unaligned)
        if (dashTelemetry != null) {
            for (Map.Entry<String, Object> e : entries.entrySet()) {
                dashTelemetry.addData(indentStr + "  " + e.getKey(), e.getValue());
            }
        }

        // children groups
        for (TelemetryHelper child : children.values()) {
            child.pushInternal(indent + 1);
        }

    }
    private static String indent(int count) {
        StringBuilder sb = new StringBuilder(count * 2);
        for (int i = 0; i < count; i++) sb.append("  ");
        return sb.toString();
    }

}

/** EXAMPLE USAGE
 * INIT:
 TelemetryHelper drive = TelemetryHelper.create(telemetry, "Drive");
 TelemetryHelper intake = TelemetryHelper.create(telemetry, "Intake");

 TelemetryHelper motors = drive.child("Motors");
 TelemetryHelper loopInfo = drive.child("Loop");

 drive.startTimer(); // optional timer start
 intake.startTimer();

 * LOOP:
 drive.clear();
 intake.clear();

 // drive data
 drive.addf("x", "%.2f", pose.x);
 drive.addf("y", "%.2f", pose.y);
 drive.addf("heading", "%.1f", pose.heading);

 // nested motor data
 motors.addf("fl", "%.2f", flPower);
 motors.addf("fr", "%.2f", frPower);
 motors.addf("bl", "%.2f", blPower);
 motors.addf("br", "%.2f", brPower);

 // timer example
 loopInfo.addf("loop ms", "%.2f", drive.lap());

 // intake data
 intake.add("state", intakeState);
 intake.addf("power", "%.2f", intakePower);

 // push everything
 drive.push();
 intake.push();

 telemetry.update();
 EXAMPLE CLEARING
 drive.remove("heading");        // remove one entry
 drive.removeChild("Motors");    // remove nested group
 drive.clearEntries();           // clear only values
 drive.clearChildren();          // clear only nested groups

 */
