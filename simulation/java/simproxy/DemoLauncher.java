package simproxy;

import java.util.Arrays;

/** The same Record/Replay mains, with hardware calls intercepted in this JVM only. */
public final class DemoLauncher {
    public static void main(String[] args) {
        try {
            if(args.length<1 || !(args[0].equals("record") || args[0].equals("replay")))
                throw new IllegalArgumentException("Choose record or replay");
            System.setProperty("demo.source","simulation");
            Bridge bridge=new Bridge();
            try(SimulatorRuntime runtime=new SimulatorRuntime(bridge)) {
                if(args[0].equals("record") && !java.awt.GraphicsEnvironment.isHeadless())TeachingPanel.open(bridge);
                String name=args[0].equals("record") ? "Record" : "Replay";
                Class.forName("sinkcleaning."+name).getMethod("main",String[].class)
                    .invoke(null,(Object)Arrays.copyOfRange(args,1,args.length));
            }
        } catch(Throwable error) { error.printStackTrace(); System.exit(1); }
    }
}
