package sinkcleaning;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import utility.SingleInstanceChecker;

/** Small course-compatible connection; connecting never commands a posture. */
public final class DemoRobot extends RoboticsAPIApplication {
    public final LBR robot;
    public final Tool tool;

    private DemoRobot(RoboticsAPIContext context) {
        super(context);
        robot = context.getDeviceFromType(LBR.class);
        tool = getApplicationData().createFromTemplate("EXERCISE_TOOL");
        tool.attachTo(robot.getFlange());
        tool.setDefaultMotionFrame(tool.getFrame("/TCP"));
    }

    public static DemoRobot connect() {
        new SingleInstanceChecker().start();
        RoboticsAPIContext.useGracefulInitialization(true);
        return new DemoRobot(RoboticsAPIContext.createFromResource(DemoRobot.class, "/RoboticsAPI.config.xml"));
    }

    @Override public void run() { }
}
