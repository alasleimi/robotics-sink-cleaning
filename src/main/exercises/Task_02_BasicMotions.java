package exercises;

import java.util.Vector;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

public class Task_02_BasicMotions extends RoboticsAPIApplication {

	public Task_02_BasicMotions(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);
		
		// check if another robot application is already running
		new SingleInstanceChecker().start();

		// initialization
		Task_02_BasicMotions app = new Task_02_BasicMotions(
				RoboticsAPIContext.createFromResource(Task_02_BasicMotions.class, "RoboticsAPI.config.xml"));

		SunriseConnector.initialize(app);

		app.run();
	}

	LBR robot;
	Tool tool;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** load saved Frames and JointPositons with the DataHandler class */
		Frame F0 = DataHandler.loadFrame("F0");
		Frame F1 = DataHandler.loadFrame("F1");
		Frame F2 = DataHandler.loadFrame("F2");
		JointPosition J0 = DataHandler.loadJointPos("J0");
		JointPosition J1 = DataHandler.loadJointPos("J1");
		JointPosition J2 = DataHandler.loadJointPos("J2");

		/**
		 * PTP movements to frames can be executed using different ObjectFrames.
		 * Two of these are the LBR class and the Tool class. Using
		 * robot.move(Frame) will move the *flange* of the robot to the target
		 * frame while using tool.move(Frame) will move the *tcp* of the tool to
		 * the target frame.
		 */
		System.out.println("Moving to F0");
		tool.move(BasicMotions.ptp(F0));

		/**
		 * HINT: uncommenting (also multiple) lines in eclipse is easily done with
		 * CTRL+7
		 */
		
		/**
		 * PTP movements with blending, movements are async so that blending is
		 * possible.
		 */
		// double blendingRadiusJoints = 0.1;
		// System.out.println("Moving to F0");
		// tool.moveAsync(BasicMotions.ptp(F0));
		// System.out.println("Moving to F1 with blending");
		// tool.moveAsync(BasicMotions.ptp(F1).setBlendingRel(blendingRadiusJoints));
		// System.out.println("Moving to F2");
		// tool.moveAsync(BasicMotions.ptp(F2));

		/** LIN movements with or without blending */
		// double blendingRadiusCart = 10;
		// System.out.println("Moving LIN to F0 with blending");
		// tool.moveAsync(BasicMotions.lin(F0).setBlendingCart(blendingRadiusCart));

		/** LINREL movements */
		// System.out.println("Moving to front");
		// SunriseConnector.goFront();
		// System.out.println("Moving LINREL +x in tool frame");
		// tool.move(BasicMotions.linRel(50, 0, 0));

		/** create a motion batch movement */
		// Vector<RobotMotion<?>> motionVector = new Vector<RobotMotion<?>>();
		// motionVector.add(BasicMotions.lin(F0));
		// RobotMotion<?>[] motionArray = motionVector.toArray(new RobotMotion<?>[motionVector.size()]);
		// MotionBatch mb = new MotionBatch(motionArray);

		 
		/** set parameters for mb */
		// mb.setJointVelocityRel(0.2);
		// mb.setBlendingCart(10);
		// System.out.println("Executing motion batch now");
		// tool.move(mb);

		/** wait until movements are finished [if there are async movements] */
		ExecutionController.waitForAllMotionsFinished();
		System.exit(0);
	}
}
