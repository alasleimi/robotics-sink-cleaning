package robot;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class Test_RobotController extends RoboticsAPIApplication {

	public static void main(String[] args) {
		Test_RobotController app = new Test_RobotController();
		SunriseConnector.initializeAndRun(app);
	}

	@Override
	public void run() {
		long start = 0;
		
		while (true) {
			start = System.currentTimeMillis();
			SunriseConnector.getRobot().getCurrentJointPosition();
			System.out.println(System.currentTimeMillis() - start);
		}
	}
}
