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
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.SensorForExternalForce;

import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

/**
 * Task 11: like Task 9, but records MORE data in every sample (every SAMPLE_MS = 15 ms).
 *
 * Guiding is exactly the same as in Task_01_TeachIn: goFront() followed by SunriseConnector.gravCompImpedanceLoop().
 * Click "STOP RECORDING" to finish (or wait for MAX_SECONDS): the JSON file is written first, then the program exits.
 *
 * Output: data/trajectories/traj_yyyy-MM-dd_HH-mm-ss.json (new file each time, never overwritten). Task_10 can replay
 * it too, because every sample still contains "t" and "q".
 *
 * Per sample:
 *   t         time since start [s]
 *   q         7 joint angles A1..A7 [rad]
 *   tcp       TCP pose [X, Y, Z in mm, A, B, C in degrees]
 *   torque    7 measured joint torques [Nm]
 *   extTorque 7 external joint torques [Nm]
 *   extForce  external TCP load [Tx, Ty, Tz in Nm, Fx, Fy, Fz in N]
 */
public class Task_11_RecordFullData extends RoboticsAPIApplication {

    private static final int SAMPLE_MS = 15;
    /** safety limit so that the program can never record forever */
    private static final int MAX_SECONDS = 600;

    public Task_11_RecordFullData(RoboticsAPIContext context) {
        super(context);
    }

    public static void main(String[] args) {
        RoboticsAPIContext.useGracefulInitialization(true);

        // check if another robot application is already running
        new SingleInstanceChecker().start();

        // initialization
        Task_11_RecordFullData app = new Task_11_RecordFullData(
                RoboticsAPIContext.createFromResource(Task_11_RecordFullData.class, "RoboticsAPI.config.xml"));

        SunriseConnector.initialize(app);

        app.run();
    }

    LBR robot;
    Tool tool;

    private volatile boolean stopRequested = false;
    private final AtomicBoolean finishing = new AtomicBoolean(false);
    private final List<double[]> samples = Collections.synchronizedList(new ArrayList<double[]>());
    private JFrame stopWindow;
    private SensorForExternalForce sensorForceExt;

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
        sensorForceExt = robot.getSensorForExternalForce();
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
                        double[] row = new double[34];
                        row[0] = (System.currentTimeMillis() - t0) / 1000.0;
                        JointPosition jp = robot.getCurrentJointPosition();
                        for (int i = 0; i < 7; i++)
                            row[1 + i] = jp.get(i);
                        Frame f = SunriseConnector.getTcpFrame();
                        row[8] = f.getX();
                        row[9] = f.getY();
                        row[10] = f.getZ();
                        row[11] = Math.toDegrees(f.getAlphaRad());
                        row[12] = Math.toDegrees(f.getBetaRad());
                        row[13] = Math.toDegrees(f.getGammaRad());
                        double[] mt = robot.getMeasuredTorque().getTorqueValues();
                        double[] et = robot.getExternalTorque().getTorqueValues();
                        for (int i = 0; i < 7; i++) {
                            row[14 + i] = mt[i];
                            row[21 + i] = et[i];
                        }
                        ForceSensorData fd = sensorForceExt.getSensorData();
                        row[28] = fd.getTorque().getX();
                        row[29] = fd.getTorque().getY();
                        row[30] = fd.getTorque().getZ();
                        row[31] = fd.getForce().getX();
                        row[32] = fd.getForce().getY();
                        row[33] = fd.getForce().getZ();
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

    private static String arr(double[] r, int from, int n, String fmt) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < n; i++) {
            if (i > 0)
                sb.append(", ");
            sb.append(String.format(Locale.US, fmt, r[from + i]));
        }
        return sb.append("]").toString();
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
            w.write("  \"fields\": \"q=joint angles A1..A7 [rad]; tcp=X,Y,Z [mm],A,B,C [deg]; torque=measured joint torques [Nm]; extTorque=external joint torques [Nm]; extForce=Tx,Ty,Tz [Nm],Fx,Fy,Fz [N]\",\n");
            w.write("  \"samples\": [\n");
            for (int s = 0; s < data.size(); s++) {
                double[] r = data.get(s);
                StringBuilder sb = new StringBuilder("    {\"t\": ");
                sb.append(String.format(Locale.US, "%.3f", r[0]));
                sb.append(", \"q\": ").append(arr(r, 1, 7, "%.5f"));
                sb.append(", \"tcp\": ").append(arr(r, 8, 6, "%.3f"));
                sb.append(", \"torque\": ").append(arr(r, 14, 7, "%.3f"));
                sb.append(", \"extTorque\": ").append(arr(r, 21, 7, "%.3f"));
                sb.append(", \"extForce\": ").append(arr(r, 28, 6, "%.3f"));
                sb.append("}").append(s < data.size() - 1 ? ",\n" : "\n");
                w.write(sb.toString());
            }
            w.write("  ]\n}\n");
            System.out.println("Trajectory written to '" + path + "'");
        } catch (IOException e) {
            System.err.println("Could not write trajectory: " + e.getMessage());
        }
    }
}