package sinkcleaning;

import com.google.gson.JsonObject;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import java.nio.file.*;
import java.util.*;

/** Basic Cartesian path replay. Original timing and forces are not commanded. */
public final class Replay {
    public static void main(String[] args) {
        int code=0;
        try { replay(args); } catch(Exception e) { e.printStackTrace(); code=1; }
        System.exit(code);
    }

    static void replay(String[] args) throws Exception {
        if(args.length<1)throw new IllegalArgumentException("Replay DIRECTORY [--approach]");
        Path folder=Paths.get(args[0]);
        JsonObject meta=DemoFiles.metadata(folder);
        List<double[]> raw=DemoFiles.read(folder), rows=new ArrayList<>();
        for(double[] row:raw)
            if(rows.isEmpty() || row[0]-rows.get(rows.size()-1)[0]>=.2 || row==raw.get(raw.size()-1))rows.add(row);
        DemoRobot app=DemoRobot.connect();
        if(meta.has("tcp")) {
            double[] saved=new com.google.gson.Gson().fromJson(meta.get("tcp"),double[].class);
            Frame recordedTool=Pose.frame(saved);
            Frame currentTool=new Frame(app.tool.getDefaultMotionFrame().getTransformationProvider().getTransformation());
            if(recordedTool.distanceTo(currentTool)>.01 || recordedTool.rotationalDistanceTo(currentTool)>.0001)
                throw new IllegalArgumentException("Recorded TCP differs from the configured tool TCP");
        }
        List<Frame> frames=new ArrayList<>();
        for(double[] row:rows) {
            Frame f=new Frame(app.robot.getRootFrame(),Pose.transformation(Arrays.copyOfRange(row,1,8)));
            if(!meta.has("learned_model") && Double.isFinite(row[10]))
                f.setRedundancyInformation(app.robot,new LBRE1Redundancy().setE1(row[10]));
            frames.add(f);
        }
        Frame current=app.robot.getCurrentCartesianPosition(app.tool.getDefaultMotionFrame(),app.robot.getRootFrame());
        Frame first=frames.get(0);
        boolean approach=Arrays.asList(args).contains("--approach");
        if(!approach && (first.distanceTo(current)>10 || first.rotationalDistanceTo(current)>.15))
            throw new IllegalStateException("Guide the TCP within 10 mm / 8 degrees of the first pose, or explicitly use --approach for a slow PTP approach.");
        if(approach)app.tool.move(ptp(first).setJointVelocityRel(.1));
        IMotionContainer previous=null;
        try {
            for(int i=1;i<frames.size();i++) {
                IMotionContainer next=app.tool.moveAsync(lin(frames.get(i)).setCartVelocity(30).setJointVelocityRel(.1)
                    .setBlendingCart(i==frames.size()-1 ? 0 : 2));
                if(previous!=null) {
                    try { previous.await(); previous.validate(); }
                    catch(Exception error) { next.cancel(); throw error; }
                }
                previous=next;
            }
            if(previous!=null) { previous.await(); previous.validate(); }
        } finally {
            if(previous!=null && !previous.isFinished())previous.cancel();
        }
        System.out.println("REPLAYED " + frames.size() + " waypoints at at most 30 mm/s (path replay, not recorded timing/force).");
    }
}
