// Copyright (c) 2024 FRC 6328 & FRC 246
// https://github.com/Mechanical-Advantage
// https://github.com/lobstahbots
//
// Adapted by FRC 246 from FRC 6328
// Original
// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/Alert.java

// MIT License

// Copyright (c) 2024 FRC 6328 & FRC 246

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package stl.networkalerts;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LoggingConstants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {
    private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();

    private final AlertType type;
    private boolean active = false;
    private double activeStartTime = 0.0;
    private double lastLoggedTime = 0.0;
    private String text;
    private BooleanSupplier cond = () -> false;

    /**
     * Creates a new Alert in the default group - "Alerts". If this is the first to
     * be instantiated, the appropriate entries will be added to NetworkTables. Its
     * condition is by default to never display.
     *
     * @param text Text to be displayed when the alert is active.
     * @param type {@link AlertType} specifying urgency.
     */
    public Alert(String text, AlertType type) {
        this("Alerts", text, type, () -> false);
    }

    /**
     * Creates a new Alert in the default group - "Alerts". If this is the first to
     * be instantiated, the appropriate entries will be added to NetworkTables.
     * 
     * @param text Text to be displayed when the alert is active.
     * @param type {@link AlertType} specifying urgency.
     * @param cond The condition with which this Alert will be displayed, as a
     *             {@link BooleanSupplier}; it will automatically make itself
     *             active/inactive by regularly calling this condition.
     */
    public Alert(String text, AlertType type, BooleanSupplier cond) {
        this("Alerts", text, type, cond);
    }

    /**
     * Creates a new Alert. If this is the first to be instantiated in its group,
     * the appropriate entries will be added to NetworkTables.
     *
     * @param group Group identifier, also used as NetworkTables title
     * @param text  Text to be displayed when the alert is active.
     * @param type  {@link AlertType} specifying urgency.
     * @param cond  The condition with which this Alert will be displayed, as a
     *              {@link BooleanSupplier}; it will automatically make itself
     *              active/inactive by regularly calling this condition.
     */
    public Alert(String group, String text, AlertType type, BooleanSupplier cond) {
        if (!groups.containsKey(group)) {
            groups.put(group, new SendableAlerts());
            SmartDashboard.putData(group, groups.get(group));
        }

        this.text = text;
        this.type = type;
        groups.get(group).alerts.add(this);
        this.cond = cond;
    }

    /**
     * Sets whether the alert should currently be displayed.
     * 
     * @param active Boolean specifying whether or not the alert should be active.
     */
    public void set(boolean active) {
        this.cond = () -> active;
    }

    /**
     * Sets the condition for whether or not this Alert should be displayed.
     * 
     * @param condition A condition as a {@link BooleanSupplier}.
     */
    public void set(BooleanSupplier condition) {
        this.cond = condition;
    }

    /**
     * Updates current alert text. If the text is different from the previous text
     * and the alert is active the alert text will also be sent to the console or
     * reported as an error/warning, similar to {@link #set set}.
     * 
     * @param text The new text.
     */
    public void setText(String text) {
        if (active && !text.equals(this.text)) log();
        this.text = text;
    }

    /**
     * Get whether or not this Alert should be enabled, based on its condition. If
     * it should be active and it wasn't before, also send it to the
     * DriverStation/log.
     * 
     * @return The value of the condition.
     */
    public boolean cond() {
        boolean res = cond.getAsBoolean();
        if (res && !active) activeStartTime = Timer.getFPGATimestamp();
        if (res && Timer.getFPGATimestamp() - lastLoggedTime > LoggingConstants.LOG_ALERT_INTERVAL) log();
        active = res;
        return res;
    }

    private void log() {
        switch (type) {
            case ERROR:
                DriverStation.reportError(text, false);
                break;
            case WARNING:
                DriverStation.reportWarning(text, false);
                break;
            case INFO:
                System.out.println(text);
                break;
        }
        lastLoggedTime = Timer.getFPGATimestamp();
    }

    /**
     * A group of {@link Alert}s which can be sent to a driver dashboard (e.g.
     * Shuffleboard/elastic).
     */
    private static class SendableAlerts implements Sendable {
        public final List<Alert> alerts = new ArrayList<>();

        /**
         * Get text for active alerts, filtered by type and sorted by time.
         * 
         * @param type {@link AlertType} to filter for.
         * @return List of active alert texts.
         */
        public String[] getAlertStrings(AlertType type) {
            Predicate<Alert> activeFilter = (Alert x) -> x.type == type && x.cond();
            Comparator<Alert> timeSorter = (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
            return alerts.stream().filter(activeFilter).sorted(timeSorter).map((Alert a) -> a.text)
                    .toArray(String[]::new);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Alerts");
            builder.addStringArrayProperty("errors", () -> getAlertStrings(AlertType.ERROR), null);
            builder.addStringArrayProperty("warnings", () -> getAlertStrings(AlertType.WARNING), null);
            builder.addStringArrayProperty("infos", () -> getAlertStrings(AlertType.INFO), null);
        }
    }

    /** Represents an alert's level of urgency. */
    public static enum AlertType {
        /**
         * High priority alert - displayed first on the dashboard with a red "X" symbol.
         * Use this type for problems which will seriously affect the robot's
         * functionality and thus require immediate attention.
         */
        ERROR,

        /**
         * Medium priority alert - displayed second on the dashboard with a yellow "!"
         * symbol. Use this type for problems which could affect the robot's
         * functionality but do not necessarily require immediate attention.
         */
        WARNING,

        /**
         * Low priority alert - displayed last on the dashboard with a green "i" symbol.
         * Use this type for problems which are unlikely to affect the robot's
         * functionality, or any other alerts which do not fall under "ERROR" or
         * "WARNING".
         */
        INFO
    }
}