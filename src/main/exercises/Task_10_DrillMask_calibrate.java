package exercises;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import robot.ExecutionController;
import robot.SunriseConnector;
import utility.SingleInstanceChecker;

public class Task_10_DrillMask_calibrate extends RoboticsAPIApplication {

	public Task_10_DrillMask_calibrate(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();

		// initialization
		Task_10_DrillMask_calibrate app = new Task_10_DrillMask_calibrate(
				RoboticsAPIContext.createFromResource(Task_10_DrillMask_calibrate.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);

		app.run();

		ExecutionController.waitForAllMotionsFinished();

		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	private LBR robot;
	private Tool tool;
	private ObjectFrame toolFrame;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		// starting position
		robot.move(BasicMotions
				.ptp(new JointPosition(0, Math.toRadians(30), 0, Math.toRadians(-60), 0, Math.toRadians(90), 0))
				.setJointVelocityRel(0.3).setJointAccelerationRel(0.3));
		
		
		
		

		// infinite motion example
		// IMotion motion = BasicMotions.xxx;
		// IMotionContainer activeMove = tool.moveAsync(motion);
		// IMotionContainer nextMove = null;
		// while (true) {
		// nextMove = tool.moveAsync(motion);
		// activeMove.await();
		// activeMove = nextMove;
		// }
	}

}
