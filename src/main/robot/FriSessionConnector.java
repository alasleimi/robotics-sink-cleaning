package robot;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.deviceModel.LBR;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class FriSessionConnector {

    private static FriSessionConnector instance = null;

    private static final String controllerHost = "192.168.42.106";
    private static final int port = 30200;

    private FRISession friSession = createSession();

    private static FriSessionConnector getInstance() {
        if (instance == null) {
            instance = new FriSessionConnector();
        }
        return instance;
    }

    private FriSessionConnector() {}

    public static Optional<FRISession> getSession() {
        return Optional.ofNullable(getInstance().friSession);
    }

    public void close() {
        getSession().ifPresent(FRISession::close);
    }

    private FRISession createSession() {
        LBR iiwa = SunriseConnector.getRobot();

        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(iiwa, controllerHost);
        friConfiguration.setSendPeriodMilliSec(3);
        friConfiguration.setReceiveMultiplier(1);
        friConfiguration.setPortOnRemote(port);
        FRISession session = new FRISession(friConfiguration);
        System.out.println("Trying to establish FRI connection on " + controllerHost + ":" + port);

        try {
            session.await(10, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            System.err.println("Timeout establishing FRI session --> no advanced controller avaiblable");
            session.close();
            return null;
        }
        System.out.println("FRI session started");
        return session;
    }


}
