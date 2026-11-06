package sinkcleaning;

import com.google.gson.*;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.executionModel.CancelledException;
import com.kuka.roboticsAPI.motionModel.HRCMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.LockSupport;

/** Record one demonstration using KUKA's dedicated hand-guiding motion. */
public final class Record {
    public static void main(String[] args) {
        int code=0;
        try { record(args); } catch (Exception e) { e.printStackTrace(); code=1; }
        System.exit(code);
    }

    static void record(String[] args) throws Exception {
        if (args.length<1) throw new IllegalArgumentException("Record OUTPUT_DIRECTORY [SECONDS] [SINK_JSON]");
        Path folder=Paths.get(args[0]).toAbsolutePath();
        double seconds=args.length>1 ? Double.parseDouble(args[1]) : 0;
        if (!Double.isFinite(seconds) || seconds<0) throw new IllegalArgumentException("Invalid duration");
        Files.createDirectories(folder.getParent());
        Files.createDirectory(folder); // Never overwrite a previous demonstration.
        JsonObject meta=new JsonObject();
        meta.addProperty("schema",1);
        meta.addProperty("source",System.getProperty("demo.source","robot"));
        meta.addProperty("created",Instant.now().toString());
        meta.addProperty("complete",false);
        meta.addProperty("units","seconds, metres, XYZW quaternion, radians, N, Nm");
        meta.addProperty("pose_frame","robot_base");
        meta.addProperty("wrench_frame","robot_base; moment about TCP");
        meta.addProperty("guidance","KUKA HRCMotions.handGuiding");
        meta.addProperty("requested_hz",50);
        meta.addProperty("tool_template","EXERCISE_TOOL");
        if (args.length>2 && !args[2].equals("-")) {
            try (Reader r=Files.newBufferedReader(Paths.get(args[2]),StandardCharsets.UTF_8)) {
                JsonObject calibration=JsonParser.parseReader(r).getAsJsonObject();
                meta.add("sink_pose",calibration.get("sink_pose"));
                meta.addProperty("calibration_file",Paths.get(args[2]).toAbsolutePath().toString());
            }
        }
        try (InputStream in=Record.class.getResourceAsStream("/RoboticsAPI.data.xml")) {
            Files.copy(in,folder.resolve("tool-and-application-data.xml"));
        }
        DemoFiles.metadata(folder,meta);
        DemoRobot app=DemoRobot.connect();
        meta.add("tcp",new Gson().toJsonTree(Pose.pose(Transformation.of(
            app.tool.getDefaultMotionFrame().getTransformationProvider().getTransformation()))));
        DemoFiles.metadata(folder,meta);
        AtomicBoolean stop=new AtomicBoolean();
        Thread input=new Thread(() -> {
            try { if(new BufferedReader(new InputStreamReader(System.in)).readLine()!=null)stop.set(true); }
            catch(IOException ignored) { /* A closed console does not discard a timed/headless take. */ }
        });
        input.setDaemon(true);
        input.start();
        IMotionContainer guidance=null;
        int count=0, gaps=0;
        long start=System.nanoTime(), next=start;
        try (BufferedWriter writer=Files.newBufferedWriter(folder.resolve("samples.csv"),StandardCharsets.UTF_8)) {
            writer.write(DemoFiles.HEADER); writer.newLine(); writer.flush();
            guidance=app.robot.moveAsync(HRCMotions.handGuiding().setJointVelocityLimit(.3).setCartVelocityLimit(100));
            System.out.println("RECORDING " + folder + " — guide the arm; Enter or releasing the guiding switch ends this take.");
            while (!stop.get() && !guidance.isFinished() && (seconds==0 || (System.nanoTime()-start)*1e-9<seconds)) {
                long began=System.nanoTime();
                Frame frame=app.robot.getCurrentCartesianPosition(app.tool.getDefaultMotionFrame(),app.robot.getRootFrame());
                double[] pose=Pose.pose(Transformation.of(frame.getTransformationProvider().getTransformation()));
                double[] joints=app.robot.getCurrentJointPosition().get();
                ForceSensorData wrench=app.robot.getExternalForceTorque(app.tool.getDefaultMotionFrame(),app.robot.getRootFrame());
                double[] row=new double[22];
                row[0]=(began-start)*1e-9;
                System.arraycopy(pose,0,row,1,7); System.arraycopy(joints,0,row,8,7);
                for(int i=0;i<3;i++) { row[15+i]=wrench.getForce().get(i); row[18+i]=wrench.getTorque().get(i); }
                row[21]=(System.nanoTime()-began)*1e-9;
                writer.write(DemoFiles.csv(row)); writer.newLine();
                if (++count%50==0) writer.flush();
                next+=20_000_000L;
                long remaining=next-System.nanoTime();
                if (remaining>0) LockSupport.parkNanos(remaining);
                else { gaps++; next=System.nanoTime(); }
            }
            guidance.validate();
            if(count<2) throw new IllegalStateException("No useful recording; check the hand-guiding enabling switch.");
            meta.addProperty("complete",true);
        } finally {
            try {
                if(guidance!=null && !guidance.isFinished()) {
                    guidance.cancel();
                    try { guidance.await(); } catch(CancelledException expected) { /* Enter/time limit intentionally cancelled guidance. */ }
                }
            } catch(RuntimeException error) {
                meta.addProperty("complete",false);
                throw error;
            } finally {
                meta.addProperty("samples",count); meta.addProperty("missed_deadlines",gaps);
                meta.addProperty("duration_s",(System.nanoTime()-start)*1e-9);
                DemoFiles.metadata(folder,meta);
            }
        }
        System.out.println("SAVED " + count + " samples; " + gaps + " missed 20 ms deadlines.");
    }
}
