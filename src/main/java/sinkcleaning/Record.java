package sinkcleaning;

import com.google.gson.JsonObject;
import com.kuka.roboticsAPI.executionModel.CancelledException;
import com.kuka.roboticsAPI.motionModel.HRCMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.LockSupport;

/** One take: prepare files, enable native hand-guiding, sample, then finish. */
public final class Record {
    private static final long SAMPLE_PERIOD_NS = 20_000_000L;

    public static void main(String[] args) {
        try {
            record(args, new AtomicBoolean());
            System.exit(0);
        } catch (Exception | LinkageError error) {
            error.printStackTrace();
            System.exit(1);
        }
    }

    /** Shared by the robot CLI and the simulator's Stop & save button. */
    public static void record(String[] args, AtomicBoolean stop) throws Exception {
        if (args.length < 1) throw new IllegalArgumentException("Record OUTPUT_DIRECTORY [SECONDS] [SINK_JSON]");
        double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 0;
        if (!Double.isFinite(seconds) || seconds < 0) throw new IllegalArgumentException("Invalid duration");
        Path folder = Paths.get(args[0]).toAbsolutePath();
        Files.createDirectories(folder.getParent());
        Files.createDirectory(folder); // Never overwrite an earlier take.

        JsonObject metadata = initialMetadata();
        DemoFiles.metadata(folder, metadata);
        Counts counts = new Counts();
        OptionalTelemetry optional = null;
        boolean complete = false;
        try {
            DemoRobot app = DemoRobot.connect();
            try (BufferedWriter samples = Files.newBufferedWriter(folder.resolve("samples.csv"), StandardCharsets.UTF_8);
                 BufferedWriter extras = Files.newBufferedWriter(folder.resolve("telemetry.jsonl"), StandardCharsets.UTF_8)) {
                optional = new OptionalTelemetry(extras);
                RecordingTelemetry telemetry = new RecordingTelemetry(app, optional);
                telemetry.configuration(folder, args.length > 2 ? args[2] : null, metadata);
                metadata.add("optional_channels", optional.summary());
                DemoFiles.metadata(folder, metadata);
                extras.flush();
                samples.write(DemoFiles.HEADER);
                samples.newLine();
                samples.flush();
                stopOnEnter(stop);
                recordSamples(app, telemetry, optional, samples, extras, stop, seconds, counts);
            }
            complete = true; // Only after motion cleanup and both files close successfully.
        } catch (Exception | LinkageError error) {
            metadata.addProperty("failure", error.getClass().getSimpleName() + ": " + error.getMessage());
            throw error;
        } finally {
            metadata.addProperty("complete", complete);
            metadata.addProperty("samples", counts.samples);
            metadata.addProperty("missed_deadlines", counts.missedDeadlines);
            metadata.addProperty("duration_s", counts.duration);
            if (optional != null) metadata.add("optional_channels", optional.summary());
            DemoFiles.metadata(folder, metadata);
        }
        System.out.println("SAVED " + counts.samples + " samples to " + folder
                + "; " + counts.missedDeadlines + " missed 20 ms deadlines.");
    }

    private static void recordSamples(DemoRobot app, RecordingTelemetry telemetry, OptionalTelemetry optional,
                                      BufferedWriter samples, BufferedWriter extras, AtomicBoolean stop,
                                      double seconds, Counts counts) throws Exception {
        IMotionContainer guidance = app.robot.moveAsync(HRCMotions.handGuiding()
                .setJointVelocityLimit(.3).setCartVelocityLimit(100));
        long started = System.nanoTime();
        long nextSample = started;
        long nextFlush = started + 500_000_000L;
        optional.start(started);
        System.out.println("RECORDING — guide the arm; Enter or releasing the guiding switch ends this take.");
        try {
            while (!stop.get() && !guidance.isFinished()) {
                long began = System.nanoTime();
                double time = (began - started) * 1e-9;
                if (seconds > 0 && time >= seconds) break;

                double[] row = telemetry.sample(time, counts.samples);
                row[21] = (System.nanoTime() - began) * 1e-9;
                samples.write(DemoFiles.csv(row));
                samples.newLine();
                counts.samples++;
                if (System.nanoTime() >= nextFlush) {
                    samples.flush();
                    extras.flush();
                    nextFlush = System.nanoTime() + 500_000_000L;
                }

                nextSample += SAMPLE_PERIOD_NS;
                long remaining = nextSample - System.nanoTime();
                if (remaining > 0) LockSupport.parkNanos(remaining);
                else {
                    counts.missedDeadlines++;
                    nextSample = System.nanoTime();
                }
            }
            guidance.validate(); // External cancellation/robot errors are not successful completion.
            if (counts.samples < 2) throw new IllegalStateException("No useful recording; check the guiding switch.");
        } finally {
            counts.duration = (System.nanoTime() - started) * 1e-9;
            if (!guidance.isFinished()) {
                guidance.cancel();
                try {
                    guidance.await();
                } catch (CancelledException expected) {
                    // Enter, Stop & save, or the time limit intentionally cancels guidance.
                }
            }
        }
    }

    private static JsonObject initialMetadata() {
        JsonObject metadata = new JsonObject();
        metadata.addProperty("schema", 2);
        metadata.addProperty("source", System.getProperty("demo.source", "robot"));
        metadata.addProperty("created", Instant.now().toString());
        metadata.addProperty("java_version", System.getProperty("java.version"));
        metadata.addProperty("os", System.getProperty("os.name") + " " + System.getProperty("os.arch"));
        metadata.addProperty("complete", false);
        metadata.addProperty("units", "seconds, metres, XYZW quaternion, radians, N, Nm");
        metadata.addProperty("pose_frame", "robot_base");
        metadata.addProperty("wrench_frame", "robot_base; moment about TCP");
        metadata.addProperty("guidance", "KUKA HRCMotions.handGuiding");
        metadata.addProperty("requested_hz", 50);
        metadata.addProperty("extra_requested_hz", 10);
        metadata.addProperty("status_requested_hz", 1);
        metadata.addProperty("tool_template", "EXERCISE_TOOL");
        metadata.addProperty("missing_optional_values", "NaN in samples.csv; null/error in telemetry.jsonl");
        metadata.addProperty("telemetry_file", "telemetry.jsonl");
        metadata.addProperty("telemetry_clock", "t: local monotonic seconds; SDK timestamps: separate system-dependent nanosecond clock");
        return metadata;
    }

    private static void stopOnEnter(AtomicBoolean stop) {
        Thread input = new Thread(() -> {
            try {
                if (new BufferedReader(new InputStreamReader(System.in)).readLine() != null) stop.set(true);
            } catch (IOException ignored) {
                // Closed stdin does not discard a timed/headless take.
            }
        }, "recording-console");
        input.setDaemon(true);
        input.start();
    }

    private static final class Counts {
        int samples;
        int missedDeadlines;
        double duration;
    }
}
