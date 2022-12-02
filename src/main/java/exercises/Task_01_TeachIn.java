package exercises;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;

import robot.ExecutionController;
import robot.SunriseConnector;
import utility.SingleInstanceChecker;

public class Task_01_TeachIn extends RoboticsAPIApplication {

	public Task_01_TeachIn(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();

		// initialization
		Task_01_TeachIn app = new Task_01_TeachIn(
				RoboticsAPIContext.createFromResource(Task_01_TeachIn.class, "RoboticsAPI.config.xml"));
		
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

		// move to a crane position
		SunriseConnector.goFront();

		// start gravity compensation
		SunriseConnector.gravCompImpedanceLoop();
		
		/** wait until movements are finished [if there are async movements] */
		ExecutionController.waitForAllMotionsFinished();
		System.exit(0);
	}
}
