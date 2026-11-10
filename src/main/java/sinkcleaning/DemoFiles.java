package sinkcleaning;

import com.google.gson.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/** The only interchange format: samples.csv plus metadata.json. SI units. */
public final class DemoFiles {
    public static final String HEADER="t,x,y,z,qx,qy,qz,qw,j1,j2,j3,j4,j5,j6,j7,fx,fy,fz,mx,my,mz,read_seconds";
    private static final Gson JSON=new GsonBuilder().setPrettyPrinting().create();

    public static String csv(double[] row) {
        StringBuilder s=new StringBuilder();
        for(double v:row) { if(s.length()>0)s.append(','); s.append(Double.toString(v)); }
        return s.toString();
    }

    public static void metadata(Path folder,JsonObject meta) throws IOException {
        try(Writer w=Files.newBufferedWriter(folder.resolve("metadata.json"),StandardCharsets.UTF_8)) { JSON.toJson(meta,w); }
    }

    public static JsonObject metadata(Path folder) throws IOException {
        try(Reader r=Files.newBufferedReader(folder.resolve("metadata.json"),StandardCharsets.UTF_8)) {
            JsonObject meta=JsonParser.parseReader(r).getAsJsonObject();
            int schema = meta.get("schema").getAsInt();
            if((schema != 1 && schema != 2) || !meta.get("complete").getAsBoolean())
                throw new IllegalArgumentException("Recording is incomplete or has an unsupported schema");
            if(!"robot_base".equals(meta.get("pose_frame").getAsString()))
                throw new IllegalArgumentException("Replay needs poses expressed in robot_base");
            return meta;
        }
    }

    public static List<double[]> read(Path folder) throws IOException {
        int schema = metadata(folder).get("schema").getAsInt();
        List<double[]> rows=new ArrayList<>();
        try(BufferedReader reader=Files.newBufferedReader(folder.resolve("samples.csv"),StandardCharsets.UTF_8)) {
            if(!HEADER.equals(reader.readLine())) throw new IllegalArgumentException("Unexpected recording columns");
            String line;
            while((line=reader.readLine())!=null) {
                String[] cells=line.split(",",-1);
                if(cells.length!=22) throw new IllegalArgumentException("Truncated recording row");
                double[] row=new double[22];
                for(int i=0;i<22;i++) {
                    row[i]=Double.parseDouble(cells[i]);
                    boolean missingOptional = schema == 2 && i >= 8 && i <= 20 && Double.isNaN(row[i]);
                    if(!Double.isFinite(row[i]) && !missingOptional)throw new IllegalArgumentException("Nonfinite sample");
                }
                if(row[0]<0 || (!rows.isEmpty() && row[0]<=rows.get(rows.size()-1)[0]))throw new IllegalArgumentException("Timestamps must increase");
                double norm=0; for(int i=4;i<8;i++)norm+=row[i]*row[i];
                if(Math.abs(norm-1)>.01)throw new IllegalArgumentException("Invalid quaternion");
                rows.add(row);
            }
        }
        if(rows.size()<2)throw new IllegalArgumentException("Need at least two samples");
        return rows;
    }
}
