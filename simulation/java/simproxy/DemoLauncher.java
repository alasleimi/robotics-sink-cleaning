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
                String[] commandArgs=Arrays.copyOfRange(args,1,args.length);
                if(args[0].equals("record") && !java.awt.GraphicsEnvironment.isHeadless()) {
                    java.util.concurrent.atomic.AtomicBoolean stop=new java.util.concurrent.atomic.AtomicBoolean();
                    RecordingPanel panel=RecordingPanel.open(bridge,stop,commandArgs[0]);
                    Exception failure=null;
                    try {sinkcleaning.Record.record(commandArgs,stop);}
                    catch(Exception error) {failure=error; error.printStackTrace();}
                    panel.finished(failure);
                    panel.awaitClose();
                    if(failure!=null)throw failure;
                } else {
                    String name=args[0].equals("record") ? "Record" : "Replay";
                    Class.forName("sinkcleaning."+name).getMethod("main",String[].class).invoke(null,(Object)commandArgs);
                }
            }
            System.exit(0);
        } catch(Throwable error) { error.printStackTrace(); System.exit(1); }
    }
}
