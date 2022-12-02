package exercises;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import robot.SunriseConnector;
import utility.SingleInstanceChecker;

public class Task_06_BreakConditions extends RoboticsAPIApplication {
	
	public Task_06_BreakConditions(RoboticsAPIContext context){
		super(context); 
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_06_BreakConditions app = new Task_06_BreakConditions(RoboticsAPIContext.createFromResource(Task_06_BreakConditions.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);
		SunriseConnector.createInfoGui();

		app.run();
	}

	LBR robot;
	Tool tool;
	ObjectFrame toolFrame;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/**
		 * load joint positions and create frames out of these. The robot will use redundancy information from its
		 * current [at runtime when using a frame] configuration
		 */
		Frame firstFrame = new Frame(Transformation.ofDeg(400, 300, 350, 180, 0, -180));
		Frame secondFrame = new Frame(Transformation.ofDeg(400, -300, 350, 180, 0, -180));

		// create and configure CartesianImpedanceControlMode
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();

		cartImpMode.parametrize(CartDOF.X).setStiffness(2500);
		cartImpMode.parametrize(CartDOF.X).setDamping(0.9);

		cartImpMode.parametrize(CartDOF.Y).setStiffness(1500);
		cartImpMode.parametrize(CartDOF.Y).setDamping(0.3);

		cartImpMode.parametrize(CartDOF.Z).setStiffness(2500);
		cartImpMode.parametrize(CartDOF.Z).setDamping(0.9);

		cartImpMode.setNullSpaceDamping(0.3);
		cartImpMode.setNullSpaceStiffness(0);

		/** break condition which stops at 10% of a joints maximum torque */
		// ICondition cond = SunriseConnector.createJointTorqueCondition(0.1);

		/** create a break condition for the y axis */
		ForceCondition cond = ForceCondition.createNormalForceCondition(null, CoordinateAxis.Y, 10);

		IMotionContainer mc = null;
		Vector force = null;

		double cartVelocity = 150;
		
		tool.move(BasicMotions.lin(secondFrame).breakWhen(cond).setMode(cartImpMode)
				.setCartVelocity(cartVelocity));
		
		while (true) {
			if (!SunriseConnector.getCabinet().getExecutionService().isPaused()) {
				/** move robot to right side */
				mc = tool.move(BasicMotions.lin(firstFrame).breakWhen(cond).setMode(cartImpMode)
						.setCartVelocity(cartVelocity));

				/** check if movement was aborted by break condition */
				if (mc.hasFired(cond)) {
					force = robot.getExternalForceTorque(toolFrame).getForce();
					System.out.println("movement to right side aborted due to break condition. Force values: " + force);
				}
				ThreadUtil.milliSleep(1000); // small sleep so that we do not spam new motions while in contact

				/** if move to right side finished or interrupted, move back to left side */
				mc = tool.move(BasicMotions.lin(secondFrame).breakWhen(cond).setMode(cartImpMode)
						.setCartVelocity(cartVelocity));

				/** check if movement was aborted by break condition */
				if (mc.hasFired(cond)) {
					force = robot.getExternalForceTorque(toolFrame).getForce();
					System.out.println("movement to left side aborted due to break condition. Force values: " + force);
				}
				ThreadUtil.milliSleep(1000); // small sleep so that we do not spam new motions while in contact

			} else {
//				break;
				System.out.println("### velocity limit exceeded, pausing for 1 second");
				ThreadUtil.milliSleep(1000);
			}
		}

	}
}
