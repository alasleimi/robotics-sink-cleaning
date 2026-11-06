package simproxy;

import com.google.gson.*;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.*;
import com.kuka.roboticsAPI.controllerModel.sunrise.*;
import com.kuka.roboticsAPI.deviceModel.*;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.*;
import com.kuka.roboticsAPI.sensorModel.*;
import java.util.concurrent.*;
import org.mockito.*;
import org.mockito.invocation.InvocationOnMock;
import static org.mockito.Mockito.*;
import utility.DataHandler;

/** Interception lives exclusively in the simulator source set and JVM. */
final class SimulatorRuntime implements AutoCloseable {
    final Bridge bridge;
    final Tool tool;
    final LBR robot;
    final RoboticsAPIContext context;
    final ObjectFrame base=World.Current.getRootFrame();
    private final Tool flange=new Tool("SIM_FLANGE");
    private final MockedStatic<RoboticsAPIContext> factory;
    private final MockedConstruction<ApplicationEngine> construction;

    SimulatorRuntime(Bridge bridge) {
        NetworkGuard.install();
        this.bridge=bridge;
        flange.getRootFrame().setTransformationProvider(new ITransformationProvider(){
            public boolean isStatic(){return false;}
            public String getUniqueName(){return "SIM_FLANGE";}
            public Transformation getTransformation(){return Poses.transformation(Bridge.array(bridge.state(),"flange"));}
            public PoseInformation getPoseInformation(){
                JsonObject state=bridge.state();Transformation t=Poses.transformation(Bridge.array(state,"flange"));
                JointPosition q=new JointPosition(Bridge.array(state,"q"));
                return new PoseInformation(t,null,q,t,null,q);
            }
        });
        Tool actualTool=new Tool("EXERCISE_TOOL");
        double[] offset=Bridge.array(bridge.state(),"tcpOffset");
        actualTool.addChildFrame("TCP",Transformation.ofTranslation(offset[0]*1000,offset[1]*1000,offset[2]*1000));
        tool=mock(Tool.class,i->{
            if(i.getMethod().getName().equals("move")||i.getMethod().getName().equals("moveAsync")) {
                if(i.getArguments().length!=1||actualTool.getDefaultMotionFrame()!=actualTool.getFrame("/TCP"))throw unsupported(i);
                return move(i.getArgument(0),"tcp",i.getMethod().getName().equals("move"));
            }
            return AdditionalAnswers.delegatesTo(actualTool).answer(i);
        });
        SunriseExecutionService execution=mock(SunriseExecutionService.class,i->{
            switch(i.getMethod().getName()) {
                case "getActiveContainerCount":return bridge.state().get("activeCount").getAsInt();
                case "isPaused":return bridge.state().get("paused").getAsBoolean();
                case "startPause":bridge.call("pause");return null;
                case "resumeExecution":bridge.call("resume");return null;
                case "cancelAll":case "clearExecutionContainerQueues":bridge.call("abort");return null;
                default:throw unsupported(i);
            }
        });
        SunriseController controller=mock(SunriseController.class,i->{
            switch(i.getMethod().getName()) {
                case "getExecutionService":return execution;
                case "acknowledgeError":return null;
                default:throw unsupported(i);
            }
        });
        SensorForExternalForce sensor=mock(SensorForExternalForce.class,i->{
            if(i.getMethod().getName().equals("getSensorData"))return force(i.getArguments());
            throw unsupported(i);
        });
        robot=mock(LBR.class,i->{
            switch(i.getMethod().getName()) {
                case "getFlange":return flange.getRootFrame();
                case "getRootFrame":return base;
                case "getName":return "LBR_iiwa_7_R800_1";
                case "getJointCount":return 7;
                case "getController":return controller;
                case "getCurrentJointPosition":return new JointPosition(Bridge.array(bridge.state(),"q"));
                case "getCurrentCartesianPosition":return currentFrame(i.getArguments());
                case "getMeasuredTorque":return new TorqueSensorData(Bridge.array(bridge.state(),"measuredTorque"));
                case "getExternalTorque":return new TorqueSensorData(Bridge.array(bridge.state(),"externalTorque"));
                case "getExternalForceTorque":return force(i.getArguments());
                case "getSensorForExternalForce":return sensor;
                case "getForwardKinematic": {
                    JsonObject result=bridge.call("forward","q",((JointPosition)i.getArgument(0)).get(),"which","flange");
                    return Poses.frame(Bridge.array(result,"pose"));
                }
                case "getInverseKinematicFromFrameAndRedundancy": {
                    JsonObject target=new JsonObject();MotionCodec.target(target,i.getArgument(0));
                    JsonObject result=bridge.call("inverse","pose",target.get("pose"),"seed",target.has("seed")?target.get("seed"):bridge.state().get("q"),"which","flange");
                    return new JointPosition(Bridge.array(result,"q"));
                }
                case "move":return move(i.getArgument(0),"flange",true);
                case "moveAsync":return move(i.getArgument(0),"flange",false);
                default:throw unsupported(i);
            }
        });
        IApplicationData data=mock(IApplicationData.class,i->{
            if(i.getMethod().getName().equals("createFromTemplate")&&"EXERCISE_TOOL".equals(i.getArgument(0)))return tool;
            throw unsupported(i);
        });
        ApplicationEngine engine=mock(ApplicationEngine.class,i->{
            if(i.getMethod().getName().equals("getApplicationData"))return data;
            throw unsupported(i);
        });
        context=mock(RoboticsAPIContext.class,i->{
            switch(i.getMethod().getName()) {
                case "getDefaultController":return controller;
                case "getDeviceFromType":if(i.getArgument(0)==LBR.class)return robot;throw unsupported(i);
                case "registerEngine":return null;
                case "getEngine":if(i.getArgument(0)==ApplicationEngine.class)return engine;throw unsupported(i);
                default:throw unsupported(i);
            }
        });
        factory=mockStatic(RoboticsAPIContext.class,i->{
            switch(i.getMethod().getName()) {
                case "createFromResource":return context;
                case "useGracefulInitialization":return null;
                default:throw unsupported(i);
            }
        });
        construction=mockConstruction(ApplicationEngine.class);
    }
    private String which(Object frame) {
        if(frame==null||frame==flange.getRootFrame())return "flange";
        if(frame==tool.getDefaultMotionFrame()||frame==tool.getFrame("/TCP"))return "tcp";
        throw new UnsupportedOperationException("Only the configured flange and EXERCISE_TOOL/TCP are supported as moving frames");
    }
    private Frame currentFrame(Object[] args) {
        JsonObject state=bridge.state();String which=which(args.length>0?args[0]:null);
        Frame f=Poses.frame(Bridge.array(state,which));
        if(args.length>1&&args[1]!=null&&args[1]!=base)f=f.copy((AbstractFrame)args[1]);
        f.setAdditionalParameter(DataHandler.SIM_JOINT_SEED,Bridge.array(state,"q"));
        return f;
    }
    private ForceSensorData force(Object[] args) {
        JsonObject state=bridge.state();double[] f=Bridge.array(state,"force"),t=Bridge.array(state,"torque");
        String origin=which(args.length>0?args[0]:null);
        if(origin.equals("flange")) {
            double[] p=Bridge.array(state,"tcp"),b=Bridge.array(state,"flange");
            double x=p[0]-b[0],y=p[1]-b[1],z=p[2]-b[2];
            t[0]+=y*f[2]-z*f[1];t[1]+=z*f[0]-x*f[2];t[2]+=x*f[1]-y*f[0];
        }
        if(args.length>1&&args[1]!=null) {
            double[] r=Poses.matrix((AbstractFrame)args[1]);f=transpose(r,f);t=transpose(r,t);
        }
        return new ForceSensorData(Vector.of(f[0],f[1],f[2]),Vector.of(t[0],t[1],t[2]),Vector.of(0,0,0),Vector.of(0,0,0));
    }
    private static double[] transpose(double[] r,double[] v) {return new double[]{r[0]*v[0]+r[3]*v[1]+r[6]*v[2],r[1]*v[0]+r[4]*v[1]+r[7]*v[2],r[2]*v[0]+r[5]*v[1]+r[8]*v[2]};}
    IMotionContainer move(IMotion motion,String which,boolean sync) {
        MotionCodec codec=new MotionCodec(f->f==flange.getRootFrame()?"flange":f==tool.getFrame("/TCP")?"tcp":null);
        JsonObject spec=codec.encode(motion,which);
        long id=bridge.call("submit","motion",spec).get("id").getAsLong();
        class Handle {
            JsonObject terminal;
            JsonObject status(){if(terminal!=null)return terminal;JsonObject s=bridge.call("motion","id",id);if(done(s))terminal=s;return s;}
            boolean done(JsonObject s){return !java.util.Set.of("queued","running").contains(s.get("state").getAsString());}
            void await(long nanos) throws InterruptedException,TimeoutException {
                long start=System.nanoTime();
                while(!done(status())) {if(nanos>=0&&System.nanoTime()-start>=nanos)throw new TimeoutException("Simulator motion "+id+" timed out");Thread.sleep(10);}
            }
        }
        Handle h=new Handle();
        IMotionContainer container=mock(IMotionContainer.class,i->{
            switch(i.getMethod().getName()) {
                case "await":h.await(i.getArguments().length==0?-1:((TimeUnit)i.getArgument(1)).toNanos(i.getArgument(0)));return null;
                case "isFinished":return h.done(h.status());
                case "hasError":return "error".equals(h.status().get("state").getAsString());
                case "getErrorMessage":JsonElement e=h.status().get("error");return e.isJsonNull()?null:e.getAsString();
                case "validate":if("error".equals(h.status().get("state").getAsString()))throw new IllegalStateException(h.status().get("error").getAsString());return null;
                case "cancel":bridge.call("cancel","id",id);return null;
                case "getCurrentMotion":case "getLastExecutedMotion":return motion;
                case "hasFired":return h.status().getAsJsonArray("fired").contains(new JsonPrimitive(codec.conditions.indexOf(i.getArgument(0))));
                default:throw unsupported(i);
            }
        });
        if(sync) {container.await();container.validate();}
        return container;
    }
    private static UnsupportedOperationException unsupported(InvocationOnMock i){return new UnsupportedOperationException("Simulator API not implemented: "+i.getMethod());}
    @Override public void close(){construction.close();factory.close();}
}
