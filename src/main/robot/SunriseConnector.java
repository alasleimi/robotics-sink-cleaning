package robot;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
//import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
//import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.sunrise.ParameterConfiguration;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseController;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

public class SunriseConnector {
    private static SunriseConnector instance = null;

    private static ControlGUI controlGui = null;

    public static ControlGUI getControlGui() {
        return controlGui;
    }

    private static InfoGUI infoGui = null;

    private LBR robot;
    private Tool tool;
    private SunriseController cabinet;
    private RoboticsAPIApplication app;

    /**
     * Maximum joint value allowed. Absolut values, movement possible in [-value, value].
     */
    public static final double[] JOINT_LIMITS = {Math.toRadians(170), Math.toRadians(120), Math.toRadians(170),
            Math.toRadians(120), Math.toRadians(170), Math.toRadians(120), Math.toRadians(175)};

    /**
     * Maximum allowed joint torques.
     */
    public static final double[] MAX_JOINT_TORQUES = {176, 176, 110, 110, 110, 40, 40};

    /**
     * Maximum Torques/Forces at the tcp: TX, TY, TZ, FX, FY, FZ.
     */
    public static final double[] TCP_FORCES = {30, 30, 30, 200, 200, 200};

    /**
     * Initializes the robot application. This function has to be called from the static main function. Further, the
     * class containing the main function must be derived from RoboticsAPIApplication.
     */
    public static void initialize(RoboticsAPIApplication app) {
        instance = new SunriseConnector(app);

        controlGui = new ControlGUI();
    }

    public static void initializeAndRun(RoboticsAPIApplication app) {
        initialize(app);
        app.initialize();
        try {
            app.run();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Creates a GUI that displays joint torques, tcp torques / forces and the tcp position
     */
    public static void createInfoGui() {
        if (infoGui == null) {
            infoGui = new InfoGUI();
        }
    }

    public static InfoGUI getInfoGui() {
        return infoGui;
    }

    public static void stopInfoGui() {
        if (infoGui != null)
            infoGui.stop();
    }

    /**
     * Returns the singleton.
     */
    public static SunriseConnector getInstance() {
        if (instance == null) {
            System.err.println("RobotController not initialized! call RobotController.initialize(...) in your main!");
            System.exit(1);
        }
        return instance;
    }

    public static RoboticsAPIApplication getRobotApp() {
        return getInstance().app;
    }

    /**
     * Get the robot object.
     *
     * @return robot reference
     */
    public static LBR getRobot() {
        return getInstance().robot;
    }

    /**
     * Get the tool object.
     *
     * @return tool reference
     */
    public static Tool getTool() {
        return getInstance().tool;
    }

    /**
     * Get the Sunrise controller.
     *
     * @return cabinet reference
     */
    public static SunriseController getCabinet() {
        return getInstance().cabinet;
    }

    public static IApplicationData getApplicationData() {
        return getInstance().app.getApplicationData();
    }

    /**
     * Get the Frame for the current tcp position.
     *
     * @return Frame containing the translation/rotation of the tcp
     */
    public static Frame getTcpFrame() {
        return getInstance().robot.getCurrentCartesianPosition(getInstance().tool.getDefaultMotionFrame());
    }

    private SunriseConnector(RoboticsAPIApplication app) {
        this.app = app;
        // this.cabinet = (SunriseController) app.getController("KUKA_Sunrise_Cabinet_1");
        this.cabinet = (SunriseController) app.getContext().getDefaultController();
        // this.robot = (LBR) app.getRobot(cabinet, "LBR_iiwa_14_R820_1");
        robot = (LBR) app.getContext().getDeviceFromType(LBR.class);

        // tool = app.getApplicationData().createFromTemplate("NO_TOOL");
        tool = app.getApplicationData().createFromTemplate("EXERCISE_TOOL");
        tool.attachTo(robot.getFlange());
        tool.setDefaultMotionFrame(tool.getFrame("/TCP"));

        // increase timeout for joint brakes for position control...
        ParameterConfiguration.Current.addParameter("JntStateSpaceCtrl@LBR_iiwa_7_R800_1", "MAINTAINTIMEOUT", 30.0);
    }

    /**
     * Start a pseudo grav comp using the robotics API positionHold command. No joint limit avoidance.
     */
    public static void gravCompImpedance() {
        getInstance();
        double stiffness = 0;
        double damping = 0.3;
        // double maxJointDelta = Math.PI;

        // configure joint impedance controller
        JointImpedanceControlMode jointImpMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness,
                stiffness, stiffness, stiffness, stiffness);
        jointImpMode.setDamping(damping, damping, damping, damping, damping, damping, damping);
        // jointImpMode.setMaxJointDeltas(maxJointDelta, maxJointDelta, maxJointDelta, maxJointDelta, maxJointDelta,
        // maxJointDelta, maxJointDelta);

        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss.SSS");

        // start positionHold
        IMotionContainer motion = getInstance().tool.moveAsync(positionHold(jointImpMode, 1000, TimeUnit.SECONDS));

        int cd = 0;
        while (true) {
            if (getInstance().cabinet.getExecutionService().isPaused()) {
                // getInstance().cabinet.getExecutionService().cancelAll();
                motion.cancel();
                motion.await();
                System.out.println("GravComp ended");
                return;
            }

            ThreadUtil.milliSleep(100);

            if (cd == 10) {
                cd = 0;
                System.out.println(sdf.format(Calendar.getInstance().getTime()));
            }
            cd++;
        }
    }

    /**
     * Start a pseudo grav comp using the robotics API positionHold command. No joint limit avoidance.
     */
    public static void gravCompImpedanceLoop() {
        JointPosition targetPosition = SunriseConnector.getRobot().getCurrentJointPosition();


        /** --- parameter settings --- */
        double stiffness = 0;
        double damping = 0.7;
        /** -------------------------- */

        JointImpedanceControlMode jointImpMode = new JointImpedanceControlMode(stiffness, stiffness, 0.5 * stiffness,
                0.5 * stiffness, 0.5 * stiffness, 0.25 * stiffness, 0.25 * stiffness);
        jointImpMode.setDamping(damping, damping, damping, damping, damping, damping, damping);

        /** start first movement */
        IMotionContainer activeMove = SunriseConnector.getRobot().moveAsync(ptp(targetPosition).setMode(jointImpMode));
        IMotionContainer nextMove = null;

        /**
         * always create a new movement to the target position before the active movement is finished and then wait for
         * the active movement
         */
        while (true) {
            nextMove = SunriseConnector.getRobot().moveAsync(ptp(targetPosition).setMode(jointImpMode));
            activeMove.await();
            if (activeMove.hasError()) {
                System.out.println("### velocity limit exceeded, pausing for 1 second");
                nextMove.cancel();
                ThreadUtil.milliSleep(1000);
                SunriseConnector.getRobot();
                nextMove = SunriseConnector.getRobot().moveAsync(ptp(targetPosition).setMode(jointImpMode));
            }

            activeMove = nextMove;
        }
    }


    public void advancedGravcomp() {
        FriSessionConnector.getSession().ifPresent(fri -> {

            JointPosition jp = robot.getCurrentJointPosition();
            robot.move(ptp(jp));

            jp = robot.getCurrentJointPosition();
            robot.move(ptp(jp)); // Switches back to position controller

            JointImpedanceControlMode freeFloatingRobotJicm = new JointImpedanceControlMode(7);
            freeFloatingRobotJicm.setStiffnessForAllJoints(0);
            freeFloatingRobotJicm.setDampingForAllJoints(0.7);

            int timeout = 100000;
            IMotionContainer imc;
            FRIJointOverlay torqueOverlay = new FRIJointOverlay(fri, ClientCommandMode.TORQUE);
            try {
                imc = robot.moveAsync(
                        BasicMotions.positionHold(freeFloatingRobotJicm, -1, TimeUnit.MILLISECONDS)
                                .addMotionOverlay(torqueOverlay)
                );
                ThreadUtil.milliSleep(timeout);
                imc.cancel();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
    }

    public void advancedGravcompLoop() {
        FriSessionConnector.getSession().ifPresent(fri -> {

            JointPosition jp = robot.getCurrentJointPosition();
            robot.move(ptp(jp));

            jp = robot.getCurrentJointPosition();
            robot.move(ptp(jp)); // Switches back to position controller

            JointImpedanceControlMode freeFloatingRobotJicm = new JointImpedanceControlMode(7);
            freeFloatingRobotJicm.setStiffnessForAllJoints(0);
            freeFloatingRobotJicm.setDampingForAllJoints(0.7);
            FRIJointOverlay torqueOverlay = new FRIJointOverlay(fri, ClientCommandMode.TORQUE);


            JointPosition targetPosition = SunriseConnector.getRobot().getCurrentJointPosition();
            IMotionContainer activeMove = SunriseConnector.getRobot().moveAsync(
                    ptp(targetPosition).setMode(freeFloatingRobotJicm).addMotionOverlay(torqueOverlay));
            IMotionContainer nextMove = null;

            while (true) {
                nextMove = SunriseConnector.getRobot().moveAsync(
                        positionHold(freeFloatingRobotJicm, -1, TimeUnit.MILLISECONDS)
                                .setMode(freeFloatingRobotJicm)
                                .addMotionOverlay(torqueOverlay));
                activeMove.await();
                if (activeMove.hasError()) {
                    System.out.println("### velocity limit exceeded, pausing for 1 second");
                    nextMove.cancel();
                    ThreadUtil.milliSleep(1000);
                    nextMove = SunriseConnector.getRobot().moveAsync(
                            positionHold(freeFloatingRobotJicm, -1, TimeUnit.MILLISECONDS)
                                    .setMode(freeFloatingRobotJicm)
                                    .addMotionOverlay(torqueOverlay));
                }
                activeMove = nextMove;
            }
        });
    }

    /**
     * Create a condition to detect collisions based on joint torques.
     *
     * @param threshold Percentage of the maximum joint torque of each joint before the condition fires.
     * @return Condition object to be used as break condition in movements.
     */
    public static ICondition createJointTorqueCondition(double threshold) {
        double maxTrq_J1 = MAX_JOINT_TORQUES[0];
        double maxTrq_J2 = MAX_JOINT_TORQUES[1];
        double maxTrq_J3 = MAX_JOINT_TORQUES[2];
        double maxTrq_J4 = MAX_JOINT_TORQUES[3];
        double maxTrq_J5 = MAX_JOINT_TORQUES[4] * 2.0; // because it doesn't work properly...
        // double maxTrq_J5 = MAX_JOINT_TORQUES[4];
        double maxTrq_J6 = MAX_JOINT_TORQUES[5];
        double maxTrq_J7 = MAX_JOINT_TORQUES[6];

        JointTorqueCondition cond_J1 = new JointTorqueCondition(JointEnum.J1, -threshold * maxTrq_J1,
                threshold * maxTrq_J1);
        JointTorqueCondition cond_J2 = new JointTorqueCondition(JointEnum.J2, -threshold * maxTrq_J2,
                threshold * maxTrq_J2);
        JointTorqueCondition cond_J3 = new JointTorqueCondition(JointEnum.J3, -threshold * maxTrq_J3,
                threshold * maxTrq_J3);
        JointTorqueCondition cond_J4 = new JointTorqueCondition(JointEnum.J4, -threshold * maxTrq_J4,
                threshold * maxTrq_J4);
        JointTorqueCondition cond_J5 = new JointTorqueCondition(JointEnum.J5, -threshold * maxTrq_J5,
                threshold * maxTrq_J5);
        JointTorqueCondition cond_J6 = new JointTorqueCondition(JointEnum.J6, -threshold * maxTrq_J6,
                threshold * maxTrq_J6);
        JointTorqueCondition cond_J7 = new JointTorqueCondition(JointEnum.J7, -threshold * maxTrq_J7,
                threshold * maxTrq_J7);

        return cond_J1.or(cond_J2, cond_J3, cond_J4, cond_J5, cond_J6, cond_J7);
    }

    /**
     * Move the robot to the home posture with 0.3 joint velocity.
     */
    public static IMotionContainer goHome() {
        return getInstance().robot.move(ptpHome().setJointVelocityRel(0.3).setJointAccelerationRel(0.3));
    }

    /**
     * Move the robot asynchronously to the home posture with 0.3 joint velocity.
     */
    public static IMotionContainer goHomeAsync() {
        return getInstance().robot.moveAsync(ptpHome().setJointVelocityRel(0.3).setJointAccelerationRel(0.3));
    }

    /**
     * Move the robot into a crane posture with 0.3 joint velocity.
     */
    public static IMotionContainer goFront() {
        return getInstance().robot
                .move(ptp(new JointPosition(0, Math.toRadians(30), 0, Math.toRadians(-60), 0, Math.toRadians(45), 0))
                        .setJointVelocityRel(0.3).setJointAccelerationRel(0.3));
    }

    /**
     * Move the robot asynchronously into a crane posture with 0.3 joint velocity.
     */
    public static IMotionContainer goFrontAsync() {
        return getInstance().robot.moveAsync(
                ptp(new JointPosition(0, Math.toRadians(30), 0, Math.toRadians(-60), 0, Math.toRadians(90), 0))
                        .setJointVelocityRel(0.3).setJointAccelerationRel(0.3));
    }

}
