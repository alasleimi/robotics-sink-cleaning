package exercises;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.RoboticsAPIContext;

import robot.SunriseConnector;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

public class Task_05b_CartesianImpedanceControl extends RoboticsAPIApplication {
	
	public Task_05b_CartesianImpedanceControl(RoboticsAPIContext context){
		super(context); 
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_05b_CartesianImpedanceControl app = new Task_05b_CartesianImpedanceControl(RoboticsAPIContext.createFromResource(Task_05b_CartesianImpedanceControl.class, "RoboticsAPI.config.xml")); 
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

		/** move robot into a crane position and save position as target frame */
		SunriseConnector.goFront();
		Frame currentFrame = SunriseConnector.getTcpFrame();

		/** create and configure CartesianImpedanceControlMode */
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();

		/** --- parameter settings --- */
		cartImpMode.parametrize(CartDOF.TRANSL).setStiffness(1500);
		cartImpMode.parametrize(CartDOF.TRANSL).setDamping(0.7);

		cartImpMode.parametrize(CartDOF.Z).setStiffness(200);

		cartImpMode.parametrize(CartDOF.ROT).setStiffness(200);
		cartImpMode.parametrize(CartDOF.ROT).setDamping(0.7);

		cartImpMode.setNullSpaceDamping(0.3);
		cartImpMode.setNullSpaceStiffness(5);
		/** -------------------------- */

		/** change the reference frame (tool, world, ...) */
		cartImpMode.setReferenceSystem(robot.getRootFrame());

		/** define a maximum force/torque applied by the controller */
		// cartImpMode.setMaxControlForce(30, 30, 30, 10, 10, 10, false);

		/** start first movement */
		IMotionContainer activeMove = tool.moveAsync(BasicMotions.lin(currentFrame).setMode(cartImpMode));
		IMotionContainer nextMove = null;

		/**
		 * always create a new movement to the target position before the active movement is finished and then wait for
		 * the active movement to finish
		 */
		while (true) {
				nextMove = tool.moveAsync(BasicMotions.lin(currentFrame).setMode(cartImpMode));
				activeMove.await();
				if (activeMove.hasError()) {
					System.out.println("### velocity limit exceeded, moving back to front position");
					nextMove.cancel();
					SunriseConnector.goFront();
					nextMove = tool.moveAsync(BasicMotions.lin(currentFrame).setMode(cartImpMode));
				}
				
				activeMove = nextMove;
		}
	}
}
