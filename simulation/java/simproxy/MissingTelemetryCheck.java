package simproxy;

import static org.mockito.Mockito.*;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import java.util.concurrent.atomic.AtomicBoolean;

/** Exercise unavailable SDK channels through the real recording loop. No hardware. */
public final class MissingTelemetryCheck {
    public static void main(String[] args) throws Exception {
        System.setProperty("demo.source", "simulation");
        try (SimulatorRuntime runtime = new SimulatorRuntime(new Bridge())) {
            doThrow(new UnsupportedOperationException("test: joint telemetry unavailable"))
                    .when(runtime.robot).getCurrentJointPosition();
            doThrow(new NoSuchMethodError("test: unavailable wrench API"))
                    .when(runtime.robot).getExternalForceTorque(any(AbstractFrame.class), any(AbstractFrame.class));
            doThrow(new IllegalStateException("test: unavailable torque sensor"))
                    .when(runtime.robot).getMeasuredTorque();
            sinkcleaning.Record.record(args, new AtomicBoolean());
        }
        System.exit(0);
    }
}
