package exercises;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.RoboticsAPIContext;

import robot.SunriseConnector;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

public class Task_05a_JointImpedanceControl extends RoboticsAPIApplication {

	public Task_05a_JointImpedanceControl(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_05a_JointImpedanceControl app = new Task_05a_JointImpedanceControl(
				RoboticsAPIContext.createFromResource(Task_05a_JointImpedanceControl.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);
		SunriseConnector.createInfoGui();

		app.run();
	}

	LBR robot;
	Tool tool;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** move robot into a crane position and save configuration as target */
		SunriseConnector.goFront();
		JointPosition targetPosition = robot.getCurrentJointPosition();

		/** create and configure JointImpedanceControlMode */

		/** --- parameter settings --- */
		double stiffness = 2500;
		double damping = 0.7;
		/** -------------------------- */

		JointImpedanceControlMode jointImpMode = new JointImpedanceControlMode(stiffness, stiffness, 0.5 * stiffness,
				0.5 * stiffness, 0.5 * stiffness, 0.25 * stiffness, 0.25 * stiffness);
		jointImpMode.setDamping(damping, damping, damping, damping, damping, damping, damping);

		/** start first movement */
		IMotionContainer activeMove = robot.moveAsync(ptp(targetPosition).setMode(jointImpMode));
		IMotionContainer nextMove = null;

		/**
		 * always create a new movement to the target position before the active movement is finished and then wait for
		 * the active movement
		 */
		while (true) {
			nextMove = robot.moveAsync(ptp(targetPosition).setMode(jointImpMode));
			activeMove.await();
			if (activeMove.hasError()) {
				System.out.println("### velocity limit exceeded, moving back to front position");
				nextMove.cancel();
				SunriseConnector.goFront();
				nextMove = robot.moveAsync(ptp(targetPosition).setMode(jointImpMode));
			}
			
			activeMove = nextMove;
		}
	}
}