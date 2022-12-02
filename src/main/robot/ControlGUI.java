package robot;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JTextField;

import com.kuka.roboticsAPI.motionModel.IMotionContainer;

import utility.DataHandler;

public class ControlGUI {

	public static final int xSize = 300;
	public static final int ySize = 200;

	public static final int xPosition = 200;
	public static final int yPosition = ySize;

	private ExecutionController execControl;

	private JTextField jointposField = new JTextField();
	private JTextField frameField = new JTextField();

	public String getFrameName() {
		return frameField.getText();
	}

	public String getJointPosName() {
		return jointposField.getText();
	}

	public void setNextFrameName(String name) {
		frameField.setText(name);
	}

	public void setNextJointPosName(String name) {
		jointposField.setText(name);
	}

	enum ButtonCommands {
		PAUSE, RESUME, ABORT, RESET, GRAV_COMP, GO_HOME, GO_FRONT, SAVE_FRAME, SAVE_JOINT_POS;
	}

	ActionListener buttonListener = new ActionListener() {
		@Override
		public void actionPerformed(ActionEvent e) {
			System.out.println(e.getActionCommand().toString());
			if (e.getActionCommand().equals(ButtonCommands.PAUSE.toString())) {
				execControl.pause();
			} else if (e.getActionCommand().equals(ButtonCommands.RESUME.toString())) {
				execControl.resume();
			} else if (e.getActionCommand().equals(ButtonCommands.ABORT.toString())) {
				execControl.abort();
			} else if (e.getActionCommand().equals(ButtonCommands.GRAV_COMP.toString())) {
				Thread t = new Thread(new Runnable() {
					@Override
					public void run() {
						SunriseConnector.gravCompImpedance();
					}
				});
				t.start();
			} else if (e.getActionCommand().equals(ButtonCommands.GO_HOME.toString())) {
				Thread t = new Thread(new Runnable() {
					@Override
					public void run() {
						System.out.println(execControl.getSunExecService().getActiveContainerCount());
						IMotionContainer motion = SunriseConnector.goHome();
						motion.await();
					}
				});
				t.start();
			} else if (e.getActionCommand().equals(ButtonCommands.GO_FRONT.toString())) {
				Thread t = new Thread(new Runnable() {
					@Override
					public void run() {
						System.out.println(execControl.getSunExecService().getActiveContainerCount());
						IMotionContainer motion = SunriseConnector.goFront();
						motion.await();
					}
				});
				t.start();
			} else if (e.getActionCommand().equals(ButtonCommands.SAVE_FRAME.toString())) {
				DataHandler.saveFrame();
			} else if (e.getActionCommand().equals(ButtonCommands.SAVE_JOINT_POS.toString())) {
				DataHandler.saveJointPosition();
			} else {
				System.err.println("Button not yet implemented");
			}
		}
	};

	public ControlGUI() {
		this.execControl = ExecutionController.getInstance();

		JFrame guiFrame = new JFrame();
		guiFrame.setLayout(new GridLayout(5, 2));

		// make sure the program exits when the frame closes
		guiFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		guiFrame.setTitle("RobotControl");
		// This will center the JFrame in the middle of the screen
		guiFrame.setLocationRelativeTo(null);
		guiFrame.setSize(xSize, ySize);

		// This will center the JFrame in the middle of the screen
		guiFrame.setLocationRelativeTo(null);
		guiFrame.setLocation(xPosition, yPosition);

		JButton pauseButton = new JButton(ButtonCommands.PAUSE.toString());
		pauseButton.addActionListener(buttonListener);
		guiFrame.add(pauseButton);

		JButton resumeButton = new JButton(ButtonCommands.RESUME.toString());
		resumeButton.addActionListener(buttonListener);
		guiFrame.add(resumeButton);

		JButton abortButton = new JButton(ButtonCommands.ABORT.toString());
		abortButton.addActionListener(buttonListener);
		guiFrame.add(abortButton);

		JButton gravcompButton = new JButton(ButtonCommands.GRAV_COMP.toString());
		gravcompButton.addActionListener(buttonListener);
		guiFrame.add(gravcompButton);

		JButton gohomeButton = new JButton(ButtonCommands.GO_HOME.toString());
		gohomeButton.addActionListener(buttonListener);
		guiFrame.add(gohomeButton);

		JButton gofrontButton = new JButton(ButtonCommands.GO_FRONT.toString());
		gofrontButton.addActionListener(buttonListener);
		guiFrame.add(gofrontButton);

		JButton touchupFrameButton = new JButton(ButtonCommands.SAVE_FRAME.toString());
		touchupFrameButton.addActionListener(buttonListener);
		guiFrame.add(touchupFrameButton);

		JButton touchupJointPosButton = new JButton(ButtonCommands.SAVE_JOINT_POS.toString());
		touchupJointPosButton.addActionListener(buttonListener);
		guiFrame.add(touchupJointPosButton);

		this.setNextFrameName(DataHandler.getNextFrameName());
		frameField.setAlignmentX(0.5f);
		guiFrame.add(frameField);
		this.setNextJointPosName(DataHandler.getNextJointPosName());
		jointposField.setAlignmentX(0.5f);
		guiFrame.add(jointposField);

		// make sure the JFrame is visible
		guiFrame.setVisible(true);
	}

}