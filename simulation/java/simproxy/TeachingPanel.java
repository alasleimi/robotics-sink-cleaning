package simproxy;

import java.awt.*;
import java.util.concurrent.*;
import javax.swing.*;

/** Simulation-only teaching and disturbance controls. */
final class TeachingPanel {
  static void open(Bridge bridge) {
    SwingUtilities.invokeLater(
        () -> {
          JFrame window = new JFrame("Simulation — teach joints / apply force");
          window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
          JPanel panel = new JPanel(new GridLayout(0, 2, 6, 4));
          JSpinner[] joints = new JSpinner[7];
          double[] q = Bridge.array(bridge.state(), "q");
          int[] limits = {170, 120, 170, 120, 170, 120, 175};
          for (int i = 0; i < 7; i++) {
            panel.add(new JLabel("Joint " + (i + 1) + " (degrees)"));
            joints[i] =
                new JSpinner(
                    new SpinnerNumberModel(Math.toDegrees(q[i]), -limits[i], limits[i], 1));
            panel.add(joints[i]);
          }
          ExecutorService worker =
              Executors.newSingleThreadExecutor(
                  r -> {
                    Thread t = new Thread(r, "sim-teaching");
                    t.setDaemon(true);
                    return t;
                  });
          JLabel status = new JLabel("Teach in Exercise 1; save using RobotControl.");
          java.util.function.Consumer<Runnable> run =
              action ->
                  worker.submit(
                      () -> {
                        try {
                          action.run();
                          SwingUtilities.invokeLater(() -> status.setText("Done"));
                        } catch (Exception e) {
                          SwingUtilities.invokeLater(() -> status.setText(e.getMessage()));
                        }
                      });
          JButton read = new JButton("Read current joints");
          read.addActionListener(
              e ->
                  run.accept(
                      () -> {
                        double[] current = Bridge.array(bridge.state(), "q");
                        SwingUtilities.invokeLater(
                            () -> {
                              for (int i = 0; i < 7; i++)
                                joints[i].setValue(Math.toDegrees(current[i]));
                            });
                      }));
          panel.add(read);
          JButton teach = new JButton("Move teaching target");
          teach.addActionListener(
              e -> {
                double[] goal = new double[7];
                for (int i = 0; i < 7; i++)
                  goal[i] = Math.toRadians(((Number) joints[i].getValue()).doubleValue());
                run.accept(() -> bridge.call("teach", "q", goal));
              });
          panel.add(teach);
          JButton release = new JButton("Release teaching target");
          release.addActionListener(e -> run.accept(() -> bridge.call("releaseTeach")));
          panel.add(release);
          JButton mouse = new JButton("Follow scene TeachTarget");
          mouse.addActionListener(e -> run.accept(() -> bridge.call("teachTarget")));
          panel.add(mouse);
          JSpinner[] forces = new JSpinner[3];
          for (int i = 0; i < 3; i++) {
            panel.add(new JLabel("World " + "XYZ".charAt(i) + " force (N)"));
            forces[i] = new JSpinner(new SpinnerNumberModel(0, -100, 100, 1));
            panel.add(forces[i]);
          }
          JButton push = new JButton("Apply force");
          push.addActionListener(
              e -> {
                double[] f = new double[3];
                for (int i = 0; i < 3; i++) f[i] = ((Number) forces[i].getValue()).doubleValue();
                run.accept(() -> bridge.call("force", "force", f));
              });
          panel.add(push);
          JButton clear = new JButton("Clear force");
          clear.addActionListener(
              e -> run.accept(() -> bridge.call("force", "force", new double[3])));
          panel.add(clear);
          window.add(panel, BorderLayout.CENTER);
          window.add(status, BorderLayout.SOUTH);
          window.pack();
          window.setLocation(50, 50);
          window.setVisible(true);
        });
  }
}
