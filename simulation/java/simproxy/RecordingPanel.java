package simproxy;

import java.awt.*;
import java.awt.event.*;
import java.nio.file.Path;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.swing.*;

/** A small simulator-only front end; the recorder itself still uses Sunrise. */
final class RecordingPanel {
    private final JFrame window=new JFrame("Record demonstration — simulation");
    private final JCheckBox guiding=new JCheckBox("Enable hand-guiding");
    private final JButton save=new JButton("Stop & save");
    private final JLabel status=new JLabel("Recording — enable hand-guiding to move the arm.");
    private final CountDownLatch closed=new CountDownLatch(1);
    private final ExecutorService worker=Executors.newSingleThreadExecutor(r -> {
        Thread thread=new Thread(r,"sim-recording-controls"); thread.setDaemon(true); return thread;
    });
    private boolean finished, closeRequested;

    private RecordingPanel(Bridge bridge, AtomicBoolean stop, String folder) {
        window.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        JPanel panel=new JPanel(new GridLayout(0,1,6,8));
        panel.setBorder(BorderFactory.createEmptyBorder(12,12,12,12));
        JLabel destination=new JLabel("Take: "+Path.of(folder).getFileName());
        destination.setToolTipText(Path.of(folder).toAbsolutePath().toString());
        panel.add(destination); panel.add(status); panel.add(guiding);
        panel.add(new JLabel("The green target is selected automatically."));
        panel.add(new JLabel("Ctrl + Shift + right-drag: horizontal; Ctrl + right-drag: vertical."));
        panel.add(new JLabel("Ctrl + left-drag: rotate. Releasing hand-guiding saves this take."));
        panel.add(save);
        guiding.addActionListener(e -> {
            boolean enabled=guiding.isSelected();
            guiding.setEnabled(false);
            worker.submit(() -> {
                try {
                    bridge.call("handguide","enabled",enabled);
                    if(enabled)bridge.call("teachTarget");
                    SwingUtilities.invokeLater(() -> {
                        if(!finished) {
                            guiding.setEnabled(enabled);
                            status.setText(enabled ? "Recording — drag the green target to guide the arm." : "Saving…");
                        }
                    });
                } catch(Exception error) {
                    stop.set(true);
                    SwingUtilities.invokeLater(() -> status.setText(error.getMessage()));
                }
            });
        });
        save.addActionListener(e -> {
            stop.set(true); save.setEnabled(false); guiding.setEnabled(false); status.setText("Saving…");
        });
        JButton close=new JButton("Close simulator");
        Runnable requestClose=() -> {
            if(finished)window.dispose();
            else {closeRequested=true; stop.set(true); save.setEnabled(false); guiding.setEnabled(false); status.setText("Saving before closing…");}
        };
        close.addActionListener(e -> requestClose.run());
        panel.add(close);
        window.addWindowListener(new WindowAdapter() {
            @Override public void windowClosing(WindowEvent e) {requestClose.run();}
            @Override public void windowClosed(WindowEvent e) {worker.shutdownNow(); closed.countDown();}
        });
        window.add(panel); window.pack(); window.setLocation(50,50); window.setVisible(true);
    }

    static RecordingPanel open(Bridge bridge, AtomicBoolean stop, String folder) throws Exception {
        RecordingPanel[] result=new RecordingPanel[1];
        SwingUtilities.invokeAndWait(() -> result[0]=new RecordingPanel(bridge,stop,folder));
        return result[0];
    }

    void finished(Exception error) throws Exception {
        SwingUtilities.invokeAndWait(() -> {
            finished=true; guiding.setSelected(false); guiding.setEnabled(false); save.setEnabled(false);
            status.setText(error==null ? "Saved. The scene stays open until you close it." : "Recording failed — incomplete take. See terminal.");
            if(closeRequested)window.dispose();
        });
    }

    void awaitClose() throws InterruptedException {closed.await();}
}
