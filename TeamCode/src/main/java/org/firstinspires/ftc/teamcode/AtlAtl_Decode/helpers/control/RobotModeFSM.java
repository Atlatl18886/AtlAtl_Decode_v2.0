package org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.control;

import org.firstinspires.ftc.teamcode.AtlAtl_Decode.helpers.EventLogger;

public class RobotModeFSM {
    public enum RobotMode {
        MANUAL,
        HEADING_LOCK,
        DISABLED
    }

    private RobotMode mode = RobotMode.MANUAL;
    private boolean lastHeadingButton = false;
    private EventLogger events;

    public RobotModeFSM(EventLogger logger) {
        this.events = logger;
    }

    //call once/loop
    public void update(boolean headingButtonPressed) {
        if (headingButtonPressed && !lastHeadingButton) {
            mode = RobotMode.HEADING_LOCK;
            events.add("Mode : HEADING_LOCK");
        } else if (!headingButtonPressed && lastHeadingButton) {
            mode = RobotMode.MANUAL;
            events.add("Mode : MANUAL");
        }
        lastHeadingButton = headingButtonPressed;
    }

    public void setDisabled() {
        mode = RobotMode.DISABLED;
        events.add("Mode → DISABLED");
    }

    public RobotMode getMode() {
        return mode;
    }

}
