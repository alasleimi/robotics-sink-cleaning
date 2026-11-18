package exercises;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingConstants;
import javax.swing.Timer;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;

import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

/**
 * Task 9: record a hand-guided trajectory into a JSON file.
 *
 * Guiding is EXACTLY the same as in Task_01_TeachIn: goFront() followed by SunriseConnector.gravCompImpedanceLoop().
 * In addition, a separate thread samples the joint positions every SAMPLE_MS (15 ms). Click "STOP RECORDING" in the
 * small window to finish (or wait for MAX_SECONDS): the JSON file is written first, then the program exits (the same
 * way Task 1 ends).
 *
 * Output: data/trajectories/traj_yyyy-MM-dd_HH-mm-ss.json (new file each time, never overwritten)
 * Format: { "name": "traj_...", "unit": "rad", "sampleMs": 15, "samples": [ {"t": 0.000, "q": [j1,...,j7]}, ... ] }
 *
 * Play it back with Task_10_PlayTrajectory.
 */
public class Task_09_RecordTrajectory extends RoboticsAPIApplication {

    private static final int SAMPLE_MS = 15;
    /** safety limit so that the program can never record forever */
    private static final int MAX_SECONDS = 600;

    public Task_09_RecordTrajectory(RoboticsAPIContext context) {
        super(context);
    }

    public static void main(String[] args) {
        RoboticsAPIContext.useGracefulInitialization(true);

        // check if another robot application is already running
        new SingleInstanceChecker().start();

        // initialization
        Task_09_RecordTrajectory app = new Task_09_RecordTrajectory(
                RoboticsAPIContext.createFromResource(Task_09_RecordTrajectory.class, "RoboticsAPI.config.xml"));

        SunriseConnector.initialize(app);

        app.run();
    }

    LBR robot;
    Tool tool;

    private volatile boolean stopRequested = false;
    private final AtomicBoolean finishing = new AtomicBoolean(false);
    private final List<double[]> samples = Collections.synchronizedList(new ArrayList<double[]>());
    private JFrame stopWindow;

    /** Small always-on-top window with a STOP button and a running timer. */
    private JFrame createStopWindow(final long t0) {
        final JFrame frame = new JFrame("Recording trajectory");
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        frame.setAlwaysOnTop(true);
        frame.setSize(320, 160);
        frame.setLocation(450, 200);
        frame.setLayout(new BorderLayout());

        final JLabel label = new JLabel("Recording... 0 s", SwingConstants.CENTER);
        frame.add(label, BorderLayout.CENTER);

        JButton stop = new JButton("STOP RECORDING");
        stop.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                requestFinish();
            }
        });
        frame.add(stop, BorderLayout.SOUTH);

        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                requestFinish();
            }
        });

        new Timer(200, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                label.setText("Recording... " + (System.currentTimeMillis() - t0) / 1000 + " s (" + samples.size()
                        + " samples)");
            }
        }).start();

        frame.setVisible(true);
        return frame;
    }

    /** Runs in its own thread, so it can never be blocked by the robot-control loop. */
    private void requestFinish() {
        stopRequested = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                finish();
            }
        }).start();
    }

    private void finish() {
        if (!finishing.compareAndSet(false, true))
            return;
        /** save FIRST so the data is never lost */
        System.out.println("Recording stopped, " + samples.size() + " samples.");
        saveJson(new ArrayList<double[]>(samples));
        if (stopWindow != null)
            stopWindow.dispose();
        System.exit(0);
    }

    @Override
    public void run() {
        robot = SunriseConnector.getRobot();
        tool = SunriseConnector.getTool();
        System.out.println("Starting " + this.getClass().getSimpleName());

        // move to a crane position
        SunriseConnector.goFront();

        final long t0 = System.currentTimeMillis();
        stopWindow = createStopWindow(t0);

        /** sampler thread: records the joint positions every SAMPLE_MS */
        Thread sampler = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!stopRequested) {
                    try {
                        JointPosition jp = robot.getCurrentJointPosition();
                        double[] row = new double[8];
                        row[0] = (System.currentTimeMillis() - t0) / 1000.0;
                        for (int i = 0; i < 7; i++)
                            row[i + 1] = jp.get(i);
                        samples.add(row);
                    } catch (RuntimeException e) {
                        System.err.println("Sampling error: " + e);
                    }
                    ThreadUtil.milliSleep(SAMPLE_MS);
                }
            }
        });
        sampler.setDaemon(true);
        sampler.start();

        /** watchdog: stop automatically after MAX_SECONDS */
        Thread watchdog = new Thread(new Runnable() {
            @Override
            public void run() {
                ThreadUtil.milliSleep(MAX_SECONDS * 1000);
                System.out.println("Max recording time reached.");
                requestFinish();
            }
        });
        watchdog.setDaemon(true);
        watchdog.start();

        System.out.println(">>> Recording started. Guide the robot by hand. Click STOP RECORDING to finish. <<<");

        // start gravity compensation (same as Task 1; runs until the program exits)
        SunriseConnector.gravCompImpedanceLoop();

        /** only reached if the loop ever returns */
        finish();
    }

    private void saveJson(List<double[]> data) {
        File dir = new File(DataHandler.DataPath + "/trajectories");
        dir.mkdirs();
        String stamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date());
        String name = "traj_" + stamp;
        File out = new File(dir, name + ".json");
        int n = 2;
        while (out.exists()) { // never overwrite an existing recording
            name = "traj_" + stamp + "_" + n++;
            out = new File(dir, name + ".json");
        }
        String path = out.getPath();
        try (BufferedWriter w = new BufferedWriter(
                new OutputStreamWriter(new FileOutputStream(path), StandardCharsets.UTF_8))) {
            w.write("{\n");
            w.write("  \"name\": \"" + name + "\",\n");
            w.write("  \"unit\": \"rad\",\n");
            w.write("  \"sampleMs\": " + SAMPLE_MS + ",\n");
            w.write("  \"samples\": [\n");
            for (int s = 0; s < data.size(); s++) {
                double[] r = data.get(s);
                StringBuilder sb = new StringBuilder("    {\"t\": ");
                sb.append(String.format(Locale.US, "%.3f", r[0])).append(", \"q\": [");
                for (int i = 1; i < 8; i++) {
                    if (i > 1)
                        sb.append(", ");
                    sb.append(String.format(Locale.US, "%.5f", r[i]));
                }
                sb.append("]}").append(s < data.size() - 1 ? ",\n" : "\n");
                w.write(sb.toString());
            }
            w.write("  ]\n}\n");
            System.out.println("Trajectory written to '" + path + "'");
        } catch (IOException e) {
            System.err.println("Could not write trajectory: " + e.getMessage());
        }
    }
}