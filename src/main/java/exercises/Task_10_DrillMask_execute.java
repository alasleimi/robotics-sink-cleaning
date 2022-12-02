package exercises;

import java.util.ArrayList;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

public class Task_10_DrillMask_execute extends RoboticsAPIApplication {

	Transformation calibrationPoint1 = Transformation.ofTranslation(-89, -50, 0);
	Transformation calibrationPoint2 = Transformation.ofTranslation(-51, -229, 0);

	public Task_10_DrillMask_execute(RoboticsAPIContext context) {
		super(context);

		// drillHoles.add(calibrationPoint1);
		// drillHoles.add(calibrationPoint2);

		drillHoles.add(Transformation.ofTranslation(-122 + 2, 50 + 2, 0));
		drillHoles.add(Transformation.ofTranslation(-90, -1 + 2, 0));
		drillHoles.add(Transformation.ofTranslation(-80 + 2, 107 + 1, 0));
		drillHoles.add(Transformation.ofTranslation(-32 + 1, 57 + 1, 0));
		drillHoles.add(Transformation.ofTranslation(36 + 1, 53, 0));
		drillHoles.add(Transformation.ofTranslation(89, -10 - 1, 0));
		drillHoles.add(Transformation.ofTranslation(124 + 1, 38 - 2, 0));
		drillHoles.add(Transformation.ofTranslation(85 + 2, 97 - 1, 0));
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();

		// initialization
		Task_10_DrillMask_execute app = new Task_10_DrillMask_execute(RoboticsAPIContext
				.createFromResource(Task_10_DrillMask_execute.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);

		app.run();

		ExecutionController.waitForAllMotionsFinished();

		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	private ArrayList<Transformation> drillHoles = new ArrayList<Transformation>();

	private final double moveUpDownDistance = 30;

	private LBR robot;
	private Tool tool;
	private ObjectFrame toolFrame;

	private Frame plateOrigin = DataHandler.loadFrame("O");
	private Frame C1 = DataHandler.loadFrame("C1");
	private Frame C2 = DataHandler.loadFrame("C2");

	private double planeParameters[] = new double[4];
	private double planeRotationZ = 0;

	private void moveUp() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(250));
	}

	private void closeProgramm() {
		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	private Frame calculateFrameInPlane(Transformation f) {
		Transformation plateOriginTrafo = (Transformation) plateOrigin.getTransformationProvider().getTransformation();

		double xOff = f.getX();
		double yOff = f.getY();
		double zOff = (-planeParameters[0] * xOff - planeParameters[1] * yOff - planeParameters[3])
				/ planeParameters[2];

		// rotate the offset and use only the translation part
		Transformation offset = Transformation.ofRad(0, 0, 0, -planeRotationZ, 0, 0)
				.compose(Transformation.ofTranslation(xOff, yOff, zOff));
		offset = Transformation.ofTranslation(offset.getTranslation());

		// compose final target frame and set z to z offset from plane
		Frame target = new Frame(offset.compose(plateOriginTrafo));
		target.setZ(zOff);

		return target;
	}

	private void calculatePlaneParameters(Frame f1, Frame f2, Frame f3) {
		double a = 0, b = 0, c = 0, d = 0;

		double a1 = f1.getX(), a2 = f1.getY(), a3 = f1.getZ();
		double b1 = f2.getX(), b2 = f2.getY(), b3 = f2.getZ();
		double c1 = f3.getX(), c2 = f3.getY(), c3 = f3.getZ();

		a = (b2 - a2) * (c3 - a3) - (c2 - a2) * (b3 - a3);
		b = (b3 - a3) * (c1 - a1) - (c3 - a3) * (b1 - a1);
		c = (b1 - a1) * (c2 - a2) - (c1 - a1) * (b2 - a2);
		d = -(a * a1 + b * a2 + c * a3);

		planeParameters[0] = a;
		planeParameters[1] = b;
		planeParameters[2] = c;
		planeParameters[3] = d;

		// get position of C1 without rotation
		Transformation C1_aligned = Transformation.ofTranslation(plateOrigin.getX() + calibrationPoint1.getX(),
				plateOrigin.getY() + calibrationPoint1.getY(), plateOrigin.getZ() + calibrationPoint1.getZ());

		// calculate vector from plate origin to C1
		Vector vec_aligned = Vector.of(C1_aligned.getX() - plateOrigin.getX(), C1_aligned.getY() - plateOrigin.getY(),
				C1_aligned.getZ() - plateOrigin.getZ()).normalize();

		// vector from plate origin to C1 measured
		Vector vec_measured = Vector
				.of(C1.getX() - plateOrigin.getX(), C1.getY() - plateOrigin.getY(), C1.getZ() - plateOrigin.getZ())
				.normalize();

		// calculate angle between both vectors
		double dotProduct = vec_aligned.getX() * vec_measured.getX() + vec_aligned.getY() * vec_measured.getY()
				+ vec_aligned.getZ() * vec_measured.getZ();
		planeRotationZ = Math.acos(dotProduct);
		// System.out.println("angle C1: " + Math.toDegrees(planeRotationZ));

		// check angle of C2 as well
		Transformation C2_aligned = Transformation.ofTranslation(plateOrigin.getX() + calibrationPoint2.getX(),
				plateOrigin.getY() + calibrationPoint2.getY(), plateOrigin.getZ() + calibrationPoint2.getZ());

		vec_aligned = Vector.of(C2_aligned.getX() - plateOrigin.getX(), C2_aligned.getY() - plateOrigin.getY(),
				C2_aligned.getZ() - plateOrigin.getZ()).normalize();

		vec_measured = Vector
				.of(C2.getX() - plateOrigin.getX(), C2.getY() - plateOrigin.getY(), C2.getZ() - plateOrigin.getZ())
				.normalize();

		dotProduct = vec_aligned.getX() * vec_measured.getX() + vec_aligned.getY() * vec_measured.getY()
				+ vec_aligned.getZ() * vec_measured.getZ();
		// System.out.println("angle C2: " + Math.toDegrees(Math.acos(dotProduct)));
		planeRotationZ = 0.5 * (planeRotationZ + Math.acos(dotProduct));
		System.out.println("rotation angle: " + Math.toDegrees(planeRotationZ));

	}

	private Frame getFrameAbove(Frame f) {
		Frame above = new Frame(Transformation.ofTranslation(0, 0, moveUpDownDistance)
				.compose(f.getTransformationProvider().getTransformation()));
		return above;
	}

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());
		
		// for safety, move calibration points a bit upwards
		plateOrigin.setZ(plateOrigin.getZ() + 15);
		C1.setZ(C1.getZ() + 15);
		C2.setZ(C2.getZ() + 15);

		// calculate plane parameters (used in calculateFrameInPlane method)
		calculatePlaneParameters(plateOrigin, C1, C2);

		// configure impedance control mode
		CartesianImpedanceControlMode impPaint = new CartesianImpedanceControlMode();
		// impPaint.parametrize(xxx);

		// this is the most important setting to not destroy pens!
		// impPaint.setMaxControlForce(x, y, z, a, b, c, false);

		
		
		// move to center position
		robot.move(BasicMotions
				.ptp(new JointPosition(0, Math.toRadians(30), 0, Math.toRadians(-60), 0, Math.toRadians(90), 0))
				.setJointVelocityRel(0.3).setJointAccelerationRel(0.3));

		/** actual program starts here */
		
		

		// example to move to origin first
		// tool.move(BasicMotions.lin(getFrameAbove(plateOrigin)).setJointVelocityRel(0.1));
		// tool.move(xxx);
		// moveUp();
		
		
		

		for (Transformation f : drillHoles) {
			Frame target = calculateFrameInPlane(f);
			System.out.println("next target: " + target);

			/** move to target and "drill the hole" */ 
		}



		closeProgramm();
	}

}
