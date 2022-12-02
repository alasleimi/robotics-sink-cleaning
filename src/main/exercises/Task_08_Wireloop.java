package exercises;

import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

public class Task_08_Wireloop extends RoboticsAPIApplication {

	public Task_08_Wireloop(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_08_Wireloop app = new Task_08_Wireloop(
				RoboticsAPIContext.createFromResource(Task_08_Wireloop.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);
		
		SunriseConnector.createInfoGui();
		
		SunriseConnector.getControlGui().setNextFrameName("wireloop_start");

		app.run();

		ExecutionController.waitForAllMotionsFinished();

		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	/**
	 * This class can be used to track the movement direction during the exploration. It can be set to POSITIVE and
	 * NEGATIVE and be toggled between both. Using value() returns the 1 or -1, respectively.
	 * 
	 * @author seid_da
	 *
	 */
	public static class Direction {

		public static final int POSITIVE = 1;
		public static final int NEGATIVE = -1;

		private int value;

		private Direction(int value) {
			this.value = value;
		}

		/**
		 * Create a new Direction instace with POSITIVE value.
		 * 
		 * @return Direction instance with value() returning 1.
		 */
		public static Direction POSITIVE() {
			return new Direction(POSITIVE);
		}

		/**
		 * Create a new Direction instace with NEGATIVE value.
		 * 
		 * @return Direction instance with value() returning -1.
		 */
		public static Direction NEGATIVE() {
			return new Direction(NEGATIVE);
		}

		/**
		 * Toggle the value of this Direction instance between 1 and -1.
		 */
		public void invert() {
			this.value *= -1;
		}

		/**
		 * Set the value of this Direction instance to 1.
		 */
		public void setPositive() {
			this.value = 1;
		}

		/**
		 * Set the value of this Direction instance to -1.
		 */
		public void setNegative() {
			this.value = -1;
		}

		/**
		 * Returns the current direction as an integer factor (1 or -1).
		 * 
		 * @return Signed int representing the direction.
		 */
		public final int value() {
			return value;
		}

		/**
		 * Returns the String representation of the current direction.
		 */
		public String toString() {
			return "" + value;
		}
	}

	private Direction xDirection = Direction.POSITIVE();
	private Direction yDirection = Direction.POSITIVE();

	private final double lowCartVel = 100;
	private final double highCartVel = 200;

	private final double moveUpDownDistance = 30;

	private Frame wireloopStart = null;

	private CartesianImpedanceControlMode cicm;

	LBR robot;
	Tool tool;
	ObjectFrame toolFrame;

	private void moveUp() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}

	private void moveDown() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, -moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** load start position [perviously teached] and define starting direction */
		wireloopStart = DataHandler.loadFrame("wireloop_start").setBetaRad(0).setGammaRad(Math.PI);

		// xDirection = Direction.NEGATIVE();
		xDirection = Direction.POSITIVE();

		/** configure impedance control mode */
		cicm = new CartesianImpedanceControlMode();
		cicm.parametrize(CartDOF.TRANSL).setStiffness(2500);
		cicm.parametrize(CartDOF.ROT).setStiffness(300);

		/** for safety, move up */
		try {
			moveUp();
		} catch (Exception e) {
			SunriseConnector.goFront();
		}

		/** move to start position in two steps: first to the position 5 cm above the start and then vertical */
		Frame aboveWireloopStart = new Frame(Transformation.ofTranslation(0, 0, 50)
				.compose(wireloopStart.getTransformationProvider().getTransformation()));
		tool.moveAsync((BasicMotions.ptp(aboveWireloopStart).setJointVelocityRel(0.6).setBlendingCart(10)));
		tool.move(BasicMotions.lin(wireloopStart).setCartVelocity(150));

		/** insert code to move along the course here */
		while (true) {

			break;
		}

	}
}
