package robot;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.controllerModel.sunrise.ResumeMode;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseController;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;

public class ExecutionController {
	private static ExecutionController instance = null;

	private SunriseController cabinet = null;

	private SunriseExecutionService sunExecService = null;

	public SunriseExecutionService getSunExecService() {
		return sunExecService;
	}

	private ExecutionController() {
		cabinet = SunriseConnector.getCabinet();
		sunExecService = cabinet.getExecutionService();
	}

	/**
	 * Returns the singleton.
	 */
	public static ExecutionController getInstance() {
		if (instance == null) {
			instance = new ExecutionController();			
//			System.err.println("RobotController not initialized! call RobotController.initialize(...) in your main!");
//			System.exit(1);
		}
		return instance;
	}

	/**
	 * Resume exection.
	 */
	public void resume() {
		// sunExecService.doPausing();
		// sunExecService.cancelPause();
		sunExecService.resumeExecution(ResumeMode.OnPath);
	}

	/**
	 * Pause execution.
	 */
	public void pause() {
		// if (!sunExecService.isPaused()) {
		sunExecService.startPause();
		// sunExecService.doPausing();
		// }

	}

	/**
	 * Abort current movement and start pause. Still often buggy!
	 */
	public void abort() {
		// if (!sunExecService.isPaused())
		// try {
		// sunExecService.startPause();
		// } catch (Exception e) {
		// System.err.println(e.getMessage());
		// }
		// if (sunExecService.getActiveContainerCount() > 0) {
		sunExecService.resumeExecution(ResumeMode.OnPath);
		ThreadUtil.milliSleep(50);
		sunExecService.cancelAll();
		sunExecService.clearExecutionContainerQueues();
		// ThreadUtil.milliSleep(100);
		// }
		cabinet.acknowledgeError();

		// System.out.println("Abort finished");
	}

	public static void waitForAllMotionsFinished() {
		SunriseExecutionService sunExecService = (SunriseExecutionService) SunriseConnector.getCabinet()
				.getExecutionService();
		while (sunExecService.getActiveContainerCount() > 0)
			;
	}

}