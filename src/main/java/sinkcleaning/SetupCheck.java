package sinkcleaning;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;

/** Checks the bundled SDK without initializing a controller or issuing motion. */
public final class SetupCheck {
    private SetupCheck() {}

    public static void main(String[] args) {
        JointPosition joints = new JointPosition(0, 0, 0, 0, 0, 0, 0);
        Frame frame = new Frame(0, 0, 500, 0, 0, 0);
        Tool tool = new Tool("setup_check");
        if (BasicMotions.ptp(joints) == null || BasicMotions.lin(frame) == null || tool == null) {
            throw new IllegalStateException("SDK object construction failed");
        }
        System.out.println("SDK geometry and motion objects OK; no robot connection attempted.");
    }
}
