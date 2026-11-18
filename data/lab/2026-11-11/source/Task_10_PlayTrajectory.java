package exercises;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

import robot.SunriseConnector;
import utility.TrajectoryChooser;
import utility.SingleInstanceChecker;

/**
 * Task 10: a dialog lets you pick one of the recordings in data/trajectories (newest first, written by
 * Task_09_RecordTrajectory); the chosen JSON is then and replay it as a blended chain
 * of PTP motions. The PATH is reproduced, not the original speed (speed is set by VELOCITY_REL).
 * Use PAUSE / ABORT in the ControlGUI to stop.
 */
public class Task_10_PlayTrajectory extends RoboticsAPIApplication {

    /** keep a waypoint only if some joint moved at least this much [rad] since the last kept one */
    private static final double MIN_JOINT_STEP = 0.05;
    /** blending radius (relative, 0..1) between waypoints -> smooth, no stop at every point */
    private static final double BLENDING_REL = 0.2;
    /** START WITH A LOW VALUE (T1 mode!) */
    private static final double VELOCITY_REL = 0.1;
    private static final double ACCELERATION_REL = 0.1;

    public Task_10_PlayTrajectory(RoboticsAPIContext context) {
        super(context);
    }

    public static void main(String[] args) {
        RoboticsAPIContext.useGracefulInitialization(true);
        new SingleInstanceChecker().start();

        Task_10_PlayTrajectory app = new Task_10_PlayTrajectory(
                RoboticsAPIContext.createFromResource(Task_10_PlayTrajectory.class, "RoboticsAPI.config.xml"));
        SunriseConnector.initialize(app);
        app.run();
    }

    LBR robot;

    @Override
    public void run() {
        robot = SunriseConnector.getRobot();
        System.out.println("Starting " + this.getClass().getSimpleName());

        /** let the user pick the recording (nothing moves before this) */
        File chosen = new TrajectoryChooser().choose();
        if (chosen == null) {
            System.out.println("No trajectory selected, exiting.");
            System.exit(0);
        }
        System.out.println("Selected trajectory: " + chosen.getName());

        List<JointPosition> waypoints = loadJson(chosen);
        if (waypoints.size() < 2) {
            System.err.println("No usable trajectory found.");
            System.exit(1);
        }
        System.out.println(waypoints.size() + " waypoints loaded.");

        /** go to the start of the trajectory first (no blending), slowly */
        robot.move(BasicMotions.ptp(waypoints.get(0)).setJointVelocityRel(0.1).setJointAccelerationRel(0.1));

        Vector<RobotMotion<?>> motions = new Vector<RobotMotion<?>>();
        for (int i = 1; i < waypoints.size(); i++) {
            motions.add(BasicMotions.ptp(waypoints.get(i)).setBlendingRel(i < waypoints.size() - 1 ? BLENDING_REL : 0));
        }
        MotionBatch batch = new MotionBatch(motions.toArray(new RobotMotion<?>[motions.size()]));
        batch.setJointVelocityRel(VELOCITY_REL);
        batch.setJointAccelerationRel(ACCELERATION_REL);

        System.out.println("Playing trajectory...");
        robot.move(batch);
        System.out.println("Done.");
        System.exit(0);
    }

    /** Minimal JSON reading: extracts every "q": [7 numbers] entry in file order. */
    private List<JointPosition> loadJson(File file) {
        List<JointPosition> result = new ArrayList<JointPosition>();
        String path = file.getPath();
        String text;
        try {
            text = new String(Files.readAllBytes(Paths.get(path)), StandardCharsets.UTF_8);
        } catch (IOException e) {
            System.err.println("could not read " + path + ": " + e.getMessage());
            return result;
        }

        Matcher m = Pattern.compile("\"q\"\\s*:\\s*\\[([^\\]]*)\\]").matcher(text);
        double[] last = null;
        double[] lastRow = null;
        while (m.find()) {
            String[] p = m.group(1).split(",");
            if (p.length != 7)
                continue;
            double[] q = new double[7];
            for (int i = 0; i < 7; i++)
                q[i] = Double.parseDouble(p[i].trim());
            lastRow = q;
            if (last == null || maxDiff(q, last) >= MIN_JOINT_STEP) {
                result.add(new JointPosition(q));
                last = q;
            }
        }
        /** always include the final pose */
        if (lastRow != null && last != null && maxDiff(lastRow, last) > 1e-4) {
            result.add(new JointPosition(lastRow));
        }
        return result;
    }

    private static double maxDiff(double[] a, double[] b) {
        double m = 0;
        for (int i = 0; i < a.length; i++)
            m = Math.max(m, Math.abs(a[i] - b[i]));
        return m;
    }
}