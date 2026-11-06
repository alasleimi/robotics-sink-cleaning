package sinkcleaning;

import com.google.gson.JsonObject;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import org.junit.Test;
import static org.junit.Assert.*;

public class DemoFilesTest {
    @Test public void realRobotFormatAndInvalidTimestamp() throws Exception {
        Path folder=Files.createTempDirectory("demo-format-");
        JsonObject meta=new JsonObject();meta.addProperty("schema",1);meta.addProperty("complete",true);
        meta.addProperty("pose_frame","robot_base");meta.addProperty("source","robot");
        DemoFiles.metadata(folder,meta);
        double[] a=new double[22],b=new double[22];a[7]=b[7]=1;b[0]=.02;b[1]=.001;
        Files.write(folder.resolve("samples.csv"),(DemoFiles.HEADER+"\n"+DemoFiles.csv(a)+"\n"+DemoFiles.csv(b)+"\n").getBytes(StandardCharsets.UTF_8));
        assertEquals(2,DemoFiles.read(folder).size());
        b[0]=0;
        Files.write(folder.resolve("samples.csv"),(DemoFiles.HEADER+"\n"+DemoFiles.csv(a)+"\n"+DemoFiles.csv(b)+"\n").getBytes(StandardCharsets.UTF_8));
        try { DemoFiles.read(folder);fail("Duplicate time accepted"); }
        catch(IllegalArgumentException expected) { assertTrue(expected.getMessage().contains("Timestamps")); }
    }

    @Test public void quaternionRoundtripAcrossBranches() {
        for(double[] p:new double[][] {{.4,.2,.3,0,0,0,1},{.4,.2,.3,1,0,0,0},
                                     {.4,.2,.3,0,1,0,0},{.4,.2,.3,0,0,1,0}}) {
            double[] back=Pose.pose(Pose.frame(p));
            for(int i=0;i<3;i++)assertEquals(p[i],back[i],1e-12);
            double dot=0;for(int i=3;i<7;i++)dot+=p[i]*back[i];
            assertEquals(1,Math.abs(dot),1e-12);
        }
    }
}
