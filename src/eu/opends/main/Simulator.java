/*
*  This file is part of OpenDS (Open Source Driving Simulator).
*  Copyright (C) 2015 Rafael Math
*
*  OpenDS is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  OpenDS is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
*/

package eu.opends.main;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

import javax.smartcardio.Card;

import eu.opends.analyzer.MyCarLogging;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import com.jme3.app.StatsAppState;
import com.jme3.app.state.VideoRecorderAppState;
import com.jme3.input.Joystick;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.niftygui.NiftyJmeDisplay;
import com.sun.javafx.application.PlatformImpl;

import de.lessvoid.nifty.Nifty;
import edu.cmu.sphinx.demo.helloworld.AudioActivity;
import eu.opends.analyzer.DrivingTaskLogger;
import eu.opends.LabViewTcpIp.TcpIpClient;
import eu.opends.analyzer.DataWriter;
import eu.opends.audio.AudioCenter;
import eu.opends.basics.InternalMapProcessing;
import eu.opends.basics.SimulationBasics;
import eu.opends.camera.SimulatorCam;
import eu.opends.cameraFlight.CameraFlight;
import eu.opends.cameraFlight.NotEnoughWaypointsException;
import eu.opends.canbus.CANClient;
import eu.opends.car.ResetPosition;
import eu.opends.car.SteeringCar;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.effects.EffectCenter;
import eu.opends.environment.TrafficLightCenter;
import eu.opends.eyetracker.EyetrackerCenter;
import eu.opends.headTracker.HeadTracker;
import eu.opends.hmi.HMICenter;
import eu.opends.input.ForceFeedbackJoystickController;
import eu.opends.input.KeyBindingCenter;
import eu.opends.knowledgeBase.KnowledgeBase;
import eu.opends.multiDriver.MultiAdapter;
import eu.opends.multiDriver.MultiCar;
import eu.opends.multiDriver.MultiDriverClient;
import eu.opends.niftyGui.DrivingTaskSelectionGUIController;
import eu.opends.oculusRift.OculusRift;
import eu.opends.reactionCenter.ReactionCenter;
import eu.opends.settingsController.SettingsControllerServer;
import eu.opends.taskDescription.contreTask.SteeringTask;
import eu.opends.taskDescription.tvpTask.MotorwayTask;
import eu.opends.taskDescription.tvpTask.ThreeVehiclePlatoonTask;
import eu.opends.tools.CollisionListener;
import eu.opends.tools.ObjectManipulationCenter;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.SpeedControlCenter;
import eu.opends.tools.Util;
import eu.opends.tools.XMLparser;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;
import eu.opends.trigger.TriggerCenter;
import eu.opends.visualization.LightningClient;
import eu.opends.visualization.MoviePlayer;
import eu.opends.weka.Weka;
import eu.opends.car.LightTexturesContainer;
import eu.opends.tools.Hud;

//import edu.cmu.sphinx.demo.helloworld.HelloWorld;
/**
 * 
 * @author Rafael Math
 */
public class Simulator extends SimulationBasics {
	private final static Logger logger = Logger.getLogger(Simulator.class);

	private boolean flag_vel =false;
	private boolean flag_acc = false;
	private float vel_x;
	private float vel_z;
	Vector3f temp = null;
	private float speed;
	private Hud hud = new Hud();
	
	private Nifty nifty;
	private int frameCounter = 0;
	boolean drivingTaskGiven = false;
	private boolean initializationFinished = false;
	private ArrayList<MultiCar> multiCarList; // KSS
	private ArrayList<Vector3f> multiCarPosition = new ArrayList<Vector3f>();

	private static Float gravityConstant;

	public static Float getGravityConstant() {
		return gravityConstant;
	}

	private SteeringCar car;

	public SteeringCar getCar() {
		return car;
	}

	private PhysicalTraffic physicalTraffic;

	public PhysicalTraffic getPhysicalTraffic() {
		return physicalTraffic;
	}

	private static DrivingTaskLogger drivingTaskLogger;

	public static DrivingTaskLogger getDrivingTaskLogger() {
		return drivingTaskLogger;
	}

	private boolean dataWriterQuittable = false;
	private DataWriter dataWriter;

	public DataWriter getMyDataWriter() {
		return dataWriter;
	}

	private LightningClient lightningClient;

	public LightningClient getLightningClient() {
		return lightningClient;
	}



	private static CANClient canClient;

	public static CANClient getCanClient() {
		return canClient;
	}

	private MultiDriverClient multiDriverClient;

	public MultiDriverClient getMultiDriverClient() {
		return multiDriverClient;
	}
	/*
	 * Start --- KSS
	 * */
	private Weka weka;
	public Weka getWeka() {
		return weka;
	}
	
	public HeadTracker headTraker;
	public HeadTracker getHeadTraker(){
		return headTraker;
	}
	//End KSS
	private TriggerCenter triggerCenter = new TriggerCenter(this);

	public TriggerCenter getTriggerCenter() {
		return triggerCenter;
	}

	private static List<ResetPosition> resetPositionList = new LinkedList<ResetPosition>();

	public static List<ResetPosition> getResetPositionList() {
		return resetPositionList;
	}

	private boolean showStats = false;

	public void showStats(boolean show) {
		showStats = show;
		setDisplayFps(show);
		setDisplayStatView(show);
	}

	public void toggleStats() {
		showStats = !showStats;
		showStats(showStats);
	}

	private CameraFlight cameraFlight;

	public CameraFlight getCameraFlight() {
		return cameraFlight;
	}

	private SteeringTask steeringTask;

	public SteeringTask getSteeringTask() {
		return steeringTask;
	}

	private ThreeVehiclePlatoonTask threeVehiclePlatoonTask;

	public ThreeVehiclePlatoonTask getThreeVehiclePlatoonTask() {
		return threeVehiclePlatoonTask;
	}

	private MotorwayTask motorwayTask;

	public MotorwayTask getMotorwayTask() {
		return motorwayTask;
	}

	private MoviePlayer moviePlayer;

	public MoviePlayer getMoviePlayer() {
		return moviePlayer;
	}

	private ReactionCenter reactionCenter;

	public ReactionCenter getReactionCenter() {
		return reactionCenter;
	}

	private EffectCenter effectCenter;

	public EffectCenter getEffectCenter() {
		return effectCenter;
	}

	private ObjectManipulationCenter objectManipulationCenter;

	public ObjectManipulationCenter getObjectManipulationCenter() {
		return objectManipulationCenter;
	}

	private String instructionScreenID = null;

	public void setInstructionScreen(String ID) {
		instructionScreenID = ID;
	}

	private SettingsControllerServer settingsControllerServer;

	public SettingsControllerServer getSettingsControllerServer() {
		return settingsControllerServer;
	}

	private EyetrackerCenter eyetrackerCenter;

	public EyetrackerCenter getEyetrackerCenter() {
		return eyetrackerCenter;
	}

	private static String outputFolder;

	public static String getOutputFolder() {
		return outputFolder;
	}

	public static boolean oculusRiftAttached = false;
	/*
	 * private static OculusRift oculusRift; public static OculusRift
	 * getOculusRift() { return oculusRift; }
	 */

	private ForceFeedbackJoystickController joystickSpringController;

	public ForceFeedbackJoystickController getJoystickSpringController() {
		return joystickSpringController;
	}

	@Override
	public void simpleInitApp() {
		showStats(false);

		if (drivingTaskGiven)
			simpleInitDrivingTask(SimulationDefaults.drivingTaskFileName, SimulationDefaults.driverName);
		else
			initDrivingTaskSelectionGUI();
	}

	private void initDrivingTaskSelectionGUI() {
		NiftyJmeDisplay niftyDisplay = new NiftyJmeDisplay(assetManager, inputManager, audioRenderer, guiViewPort);

		// Create a new NiftyGUI object
		nifty = niftyDisplay.getNifty();

		String xmlPath = "Interface/DrivingTaskSelectionGUI.xml";

		// Read XML and initialize custom ScreenController
		nifty.fromXml(xmlPath, "start", new DrivingTaskSelectionGUIController(this, nifty));

		// attach the Nifty display to the gui view port as a processor
		guiViewPort.addProcessor(niftyDisplay);

		// disable fly cam
		flyCam.setEnabled(false);
	}

	public void closeDrivingTaskSelectionGUI() {
		nifty.exit();
		inputManager.setCursorVisible(false);
		flyCam.setEnabled(true);
	}

	public void simpleInitDrivingTask(String drivingTaskFileName, String driverName) {
		SimulationDefaults.drivingTaskFileName = drivingTaskFileName;

		Util.makeDirectory("analyzerData");
		outputFolder = "analyzerData/" + Util.getDateTimeString();

		initDrivingTaskLayers();

		// show stats if set in driving task
		showStats(settingsLoader.getSetting(Setting.General_showStats, false));

		// check Oculus Rift mode: auto, enabled, disabled
		String oculusAttachedString = settingsLoader.getSetting(Setting.OculusRift_isAttached,
				SimulationDefaults.OculusRift_isAttached);
		if (oculusAttachedString.equalsIgnoreCase("enabled"))
			oculusRiftAttached = true;
		else if (oculusAttachedString.equalsIgnoreCase("disabled"))
			oculusRiftAttached = false;

		// sets up physics, camera, light, shadows and sky
		super.simpleInitApp();

		// set gravity
		gravityConstant = drivingTask.getSceneLoader().getGravity(SimulationDefaults.gravity);
		getPhysicsSpace().setGravity(new Vector3f(0, -gravityConstant, 0));
		getPhysicsSpace().setAccuracy(0.008f); // TODO comment to set accuracy
												// to 0.0166666 ?

		//PanelCenter.init(this);
		//Hud.init(this);

		Joystick[] joysticks = inputManager.getJoysticks();
		if (joysticks != null)
			for (Joystick joy : joysticks)
				System.out.println("Connected joystick: " + joy.toString());

		// load map model
		new InternalMapProcessing(this);

		// create and place steering car
		car = new SteeringCar(this);

		// initialize physical vehicles
		physicalTraffic = new PhysicalTraffic(this);
		// physicalTraffic.start(); //TODO

		// open TCP connection to KAPcom (knowledge component) [affects the
		// driver name, see below]
		if (settingsLoader.getSetting(Setting.KnowledgeManager_enableConnection,
				SimulationDefaults.KnowledgeManager_enableConnection)) {
			String ip = settingsLoader.getSetting(Setting.KnowledgeManager_ip, SimulationDefaults.KnowledgeManager_ip);
			if (ip == null || ip.isEmpty())
				ip = "127.0.0.1";
			int port = settingsLoader.getSetting(Setting.KnowledgeManager_port,
					SimulationDefaults.KnowledgeManager_port);

			// KnowledgeBase.KB.setConnect(true);
			KnowledgeBase.KB.setCulture("en-US");
			KnowledgeBase.KB.Initialize(this, ip, port);
			KnowledgeBase.KB.start();
		}

		// sync driver name with KAPcom. May provide suggestion for driver name
		// if NULL.
		// driverName = KnowledgeBase.User().initUserName(driverName);

		if (driverName == null || driverName.isEmpty())
			driverName = settingsLoader.getSetting(Setting.General_driverName, SimulationDefaults.driverName);
		SimulationDefaults.driverName = driverName;

		// setup key binding
		keyBindingCenter = new KeyBindingCenter(this);

		AudioCenter.init(this);

		// setup camera settings
		cameraFactory = new SimulatorCam(this, car);
		
		AudioActivity audio = new AudioActivity();

		// start trafficLightCenter
		trafficLightCenter = new TrafficLightCenter(this);

		// init trigger center
		triggerCenter.setup();

		// init HMICenter
		HMICenter.init(this);

		// open TCP connection to Lightning
		if (settingsLoader.getSetting(Setting.ExternalVisualization_enableConnection,
				SimulationDefaults.Lightning_enableConnection)) {
			lightningClient = new LightningClient();
		}

		// open TCP connection to CAN-bus
		if (settingsLoader.getSetting(Setting.CANInterface_enableConnection,
				SimulationDefaults.CANInterface_enableConnection)) {
			canClient = new CANClient(this);
			canClient.start();
		}

		// if(settingsLoader.getSetting(Setting.MultiDriver_enableConnection,
		// SimulationDefaults.MultiDriver_enableConnection))
		// {
		multiDriverClient = new MultiDriverClient(this, driverName);
		multiDriverClient.start();
		// }

		drivingTaskLogger = new DrivingTaskLogger(outputFolder, driverName, drivingTask.getFileName());

//		weka = new Weka(outputFolder);
//		headTraker = new HeadTracker();
		SpeedControlCenter.init(this);

		try {

			// attach camera to camera flight
			cameraFlight = new CameraFlight(this);

		} catch (NotEnoughWaypointsException e) {

			// if not enough way points available, attach camera to driving car
			car.getCarNode().attachChild(cameraFactory.getMainCameraNode());
		}

		reactionCenter = new ReactionCenter(this);

		steeringTask = new SteeringTask(this, driverName);

		threeVehiclePlatoonTask = new ThreeVehiclePlatoonTask(this, driverName);

		motorwayTask = new MotorwayTask(this);

		moviePlayer = new MoviePlayer(this);

		// start effect center
		effectCenter = new EffectCenter(this);

		objectManipulationCenter = new ObjectManipulationCenter(this);

		if (settingsLoader.getSetting(Setting.SettingsControllerServer_startServer,
				SimulationDefaults.SettingsControllerServer_startServer)) {
			settingsControllerServer = new SettingsControllerServer(this);
			settingsControllerServer.start();
		}

		StatsAppState statsAppState = stateManager.getState(StatsAppState.class);
		if (statsAppState != null && statsAppState.getFpsText() != null && statsAppState.getStatsView() != null) {
			statsAppState.getFpsText().setLocalTranslation(3, getSettings().getHeight() - 145, 0);
			statsAppState.getStatsView().setLocalTranslation(3, getSettings().getHeight() - 145, 0);
			statsAppState.setDarkenBehind(false);
		}

		// add physics collision listener
		CollisionListener collisionListener = new CollisionListener();
		getPhysicsSpace().addCollisionListener(collisionListener);

		String videoPath = settingsLoader.getSetting(Setting.General_captureVideo, "");
		if ((videoPath != null) && (!videoPath.isEmpty()) && (Util.isValidFilename(videoPath))) {
			System.err.println("videoPath: " + videoPath);
			File videoFile = new File(videoPath);
			stateManager.attach(new VideoRecorderAppState(videoFile));
		}

		if (settingsLoader.getSetting(Setting.Eyetracker_enableConnection,
				SimulationDefaults.Eyetracker_enableConnection)) {
			eyetrackerCenter = new EyetrackerCenter(this);
		}

		joystickSpringController = new ForceFeedbackJoystickController(this);

		initializationFinished = true;
	}

	private void initDrivingTaskLayers() {
		String drivingTaskFileName = SimulationDefaults.drivingTaskFileName;
		File drivingTaskFile = new File(drivingTaskFileName);
		drivingTask = new DrivingTask(this, drivingTaskFile);

		sceneLoader = drivingTask.getSceneLoader();
		scenarioLoader = drivingTask.getScenarioLoader();
		interactionLoader = drivingTask.getInteractionLoader();
		settingsLoader = drivingTask.getSettingsLoader();
	}

	/**
	 * That method is going to be executed, when the dataWriter is
	 * <code>null</code> and the S-key is pressed.
	 * 
	 * @param trackNumber
	 *            Number of track (will be written to the log file).
	 */
	public void initializeDataWriter(int trackNumber) {
		dataWriter = new DataWriter(outputFolder, car, SimulationDefaults.driverName,
				SimulationDefaults.drivingTaskFileName, trackNumber);
	}

	@Override
	public void simpleUpdate(float tpf) {
		if (initializationFinished) {
			super.simpleUpdate(tpf);

			//PanelCenter.update();
			if(Hud.flag >= 1)
				Hud.update();
			
			// updates camera
			cameraFactory.updateCamera();

			if (!isPause())
				car.getTransmission().updateRPM(tpf);


			triggerCenter.doTriggerChecks();

			
			// logging
			if(flag_vel == false && flag_acc == false) //처음
			{
				MyCarLogging.getInstance().setPosition(car.getPosition());
				temp = car.getPosition();
				flag_vel = true;
				MyCarLogging.getInstance().setSpeed(car.getCurrentSpeedKmh());
				speed = car.getCurrentSpeedKmh();
			}
			else if(flag_vel == true && flag_acc == false) //그후
			{
				MyCarLogging.getInstance().setPosition(car.getPosition());
				MyCarLogging.getInstance().SetPrePosition(temp);
				temp = car.getPosition();
				MyCarLogging.getInstance().SetVelocity_X();
				MyCarLogging.getInstance().SetVelocity_Z();
				vel_x = MyCarLogging.getInstance().GetVelocity_X();
				vel_z = MyCarLogging.getInstance().GetVelocity_Z();
				flag_acc = true;
				MyCarLogging.getInstance().setSpeed(car.getCurrentSpeedKmh());
				MyCarLogging.getInstance().SetPreSpeed(speed);
				speed = car.getCurrentSpeedKmh();
				MyCarLogging.getInstance().SetAcceleration();
			}
			else
			{
				MyCarLogging.getInstance().setPosition(car.getPosition());
				MyCarLogging.getInstance().SetPrePosition(temp);
				temp = car.getPosition();
				MyCarLogging.getInstance().SetVelocity_X();
				MyCarLogging.getInstance().SetVelocity_Z();
				MyCarLogging.getInstance().SetPreVelocity_x(vel_x);
				MyCarLogging.getInstance().SetPreVelocity_z(vel_z);
				vel_x = MyCarLogging.getInstance().GetVelocity_X();
				vel_z = MyCarLogging.getInstance().GetVelocity_Z();
				MyCarLogging.getInstance().SetAcceleration_X();
				MyCarLogging.getInstance().SetAcceleration_Z();
				MyCarLogging.getInstance().setSpeed(car.getCurrentSpeedKmh());
				MyCarLogging.getInstance().SetPreSpeed(speed);
				speed = car.getCurrentSpeedKmh();
				MyCarLogging.getInstance().SetAcceleration();
			}
			
			
			multiCarList = MultiAdapter.getInstance().getMulticarList();
			MyCarLogging.getInstance().setName(SimulationDefaults.driverName);
			MyCarLogging.getInstance().setBrake(car.getBrakePedalIntensity());
			MyCarLogging.getInstance().setSteerState(car.getSteeringWheelState());
			MyCarLogging.getInstance().setSteeringAngle(FastMath.abs(car.getSteeringWheelState()));
			MyCarLogging.getInstance().setSignal(car.getTurnSignal());
			MyCarLogging.getInstance().setRotation(car.getRotation());
			MyCarLogging.getInstance().setRpm(car.getTransmission().getRPM());
		

			updateDataWriter();

			// send camera data via TCP to Lightning
			if (lightningClient != null)
				lightningClient.sendCameraData(cam);

			// send car data via TCP to CAN-bus
			if (canClient != null)
				canClient.sendCarData();

			if (multiDriverClient != null)
				multiDriverClient.update();

			if (!isPause())
				car.update(tpf);

			// TODO start thread in init-method to update traffic
			physicalTraffic.update(tpf);

			SpeedControlCenter.update();

			// update necessary even in pause
			AudioCenter.update(tpf, cam);

			if (!isPause())
				steeringTask.update(tpf);

			// if(!isPause())
			// getCameraFlight().play();

			threeVehiclePlatoonTask.update(tpf);

			motorwayTask.update(tpf);

			moviePlayer.update(tpf);

			if (cameraFlight != null)
				cameraFlight.update();

			reactionCenter.update();

			// update effects
			effectCenter.update(tpf);

			// forward instruction screen if available
			if (instructionScreenID != null) {
				instructionScreenGUI.showDialog(instructionScreenID);
				instructionScreenID = null;
			}

			if (eyetrackerCenter != null)
				eyetrackerCenter.update();

			if (frameCounter == 5) {
				if (settingsLoader.getSetting(Setting.General_pauseAfterStartup,
						SimulationDefaults.General_pauseAfterStartup))
					setPause(true);
			}

			frameCounter++;

			joystickSpringController.update(tpf);
		}
	}

	/*
	 * Start --- KSS 
	 * 운전자차량, 주변트래픽차량, 멀티차량 정보 기록 메소드
	 * */
	private void updateDataWriter() {
		if (dataWriter != null && dataWriter.isDataWriterEnabled()) {
			if (!isPause())
				dataWriter.saveAnalyzerData();

			if (!dataWriterQuittable)
				dataWriterQuittable = true;
		}

		else {
			if (dataWriterQuittable) {
				dataWriter.quit();
				dataWriter = null;
				dataWriterQuittable = false;
			}
		}

		 // 운전자차량 로깅

        String line = System.getProperty("line.separator"); // 줄바꿈
        int signalNumber;
        
        if(MyCarLogging.getInstance().getSignal() == LightTexturesContainer.TurnSignalState.valueOf("LEFT"))
           signalNumber = 1;
        else if(MyCarLogging.getInstance().getSignal() == LightTexturesContainer.TurnSignalState.valueOf("RIGHT"))
           signalNumber = 2;
        else if(MyCarLogging.getInstance().getSignal() == LightTexturesContainer.TurnSignalState.valueOf("BOTH"))
           signalNumber = 3;
        else
           signalNumber = 0;

        
        drivingTaskLogger.reportText( 
              ""+','
              +MyCarLogging.getInstance().getPosition().getX() +"," 
              +MyCarLogging.getInstance().getPosition().getY() +","
              +MyCarLogging.getInstance().getPosition().getZ()+","
              +MyCarLogging.getInstance().getRotation().getX() +","
              +MyCarLogging.getInstance().getRotation().getY()+","
              +MyCarLogging.getInstance().getRotation().getZ()+","
              +MyCarLogging.getInstance().getRotation().getW() +","
              +MyCarLogging.getInstance().getSpeed()+","
              +MyCarLogging.getInstance().getAcceleration()+","
              +MyCarLogging.getInstance().getRpm()+","
              +MyCarLogging.getInstance().GetVelocity_X()+"," 
              +MyCarLogging.getInstance().GetVelocity_Z()+","
              +MyCarLogging.getInstance().GetAcceleration_X() +"," 
              +MyCarLogging.getInstance().GetAcceleration_Z() +","
              +MyCarLogging.getInstance().getSteeringAngle()+","
              +MyCarLogging.getInstance().getBrake()+","+
              +MyCarLogging.getInstance().getDistanceFromCenterLine()+","
              +MyCarLogging.getInstance().getCurrentLine()+","
              +MyCarLogging.getInstance().getLeftLaneDistance()+","
              +MyCarLogging.getInstance().getRightLaneDistance()+","
              +MyCarLogging.getInstance().getOffsetFromLaneCenter()+","
              +signalNumber+","
              +"NORMAL" ,new Date());

   
        // 주변차량 로깅
        float distance = 30.0f; // 일정거리

        for (TrafficObject trafficCar : PhysicalTraffic.getTrafficObjectList()) { // 주변차량
           Vector3f trafficPos = trafficCar.getPosition(); // 주변차량들 위치
           Quaternion trafiicRot = trafficCar.getRotation(); // 주변차량 회전값
           if (Math.abs(MyCarLogging.getInstance().getPosition().distance(trafficPos)) < distance) { // 내차와의
                                                                 // 일정
                                                                 // 간격안에
                                                                 // 들어오면
              drivingTaskLogger.reportText("DriverName : " + trafficCar.getName() + ", Distance_from_myCar : "

                    + Math.abs(MyCarLogging.getInstance().getPosition().distance(trafficPos)), ""); // 기록
           }
        }
        
        

        // 멀티차량 로깅
        for (MultiCar car : MultiAdapter.getInstance().getMulticarList()) {
           Vector3f multiPos = new Vector3f(car.getxPosition(), 0, car.getzPosition());
           Quaternion    multiRot = new Quaternion(car.getxRotation(),car.getyRotation() , car.getzRotation(), car.getwRotation());
           float  multiWheel = car.getSteeringWheel();
           if (Math.abs(MyCarLogging.getInstance().getPosition().distance(multiPos)) < distance) {
              drivingTaskLogger.reportText(car.getId()+","+multiPos.getX() + "," + multiPos.getY() + ","+multiPos.getZ()
                    +","+multiRot.getX() +","+multiRot.getY()+","+multiRot.getZ()+","+multiRot.getW() + ","
                    + car.getSteeringWheel()+","
                    + Math.abs(MyCarLogging.getInstance().getPosition().distance(multiPos)), new Date());
           }
        }
	}
	//End --- KSS

	/**
	 * Cleanup after game loop was left. Will be called when pressing any
	 * close-button. destroy() will be called subsequently.
	 */
	/*
	 * @Override public void stop() { logger.info("started stop()");
	 * super.stop(); logger.info("finished stop()"); }
	 */

	/**
	 * Cleanup after game loop was left Will be called whenever application is
	 * closed.
	 */

	@Override
	public void destroy() {
		logger.info("started destroy()");

		if (initializationFinished) {
			if (lightningClient != null)
				lightningClient.close();

			if (canClient != null)
				canClient.requestStop();

			if (multiDriverClient != null)
				multiDriverClient.close();

			trafficLightCenter.close();

			steeringTask.close();

			threeVehiclePlatoonTask.close();

			moviePlayer.stop();

			reactionCenter.close();

			HMICenter.close();

			KnowledgeBase.KB.disconnect();

			car.close();

			physicalTraffic.close();

			if (settingsControllerServer != null)
				settingsControllerServer.close();

			if (eyetrackerCenter != null)
				eyetrackerCenter.close();

			joystickSpringController.close();
			// initDrivingTaskSelectionGUI();
		}

		super.destroy();
		logger.info("finished destroy()");

		PlatformImpl.exit();
		// System.exit(0);
	}

	public static void main(String[] args) {
		
		Simulator sim = new Simulator();
		TcpIpClient.getinstance().setSetting(sim);
		
		try {
			// copy native files of force feedback joystick driver
			boolean isWindows = System.getProperty("os.name").toLowerCase().indexOf("win") >= 0;
			if (isWindows) {
				boolean is64Bit = System.getProperty("sun.arch.data.model").equalsIgnoreCase("64");
				if (is64Bit) {
					copyFile("lib/ffjoystick/native/win64/ffjoystick.dll", "ffjoystick.dll");
					copyFile("lib/ffjoystick/native/win64/SDL.dll", "SDL.dll");
				} else {
					copyFile("lib/ffjoystick/native/win32/ffjoystick.dll", "ffjoystick.dll");
					copyFile("lib/ffjoystick/native/win32/SDL.dll", "SDL.dll");
				}
			}

			// load logger configuration file
			PropertyConfigurator.configure("assets/JasperReports/log4j/log4j.properties");

			oculusRiftAttached = OculusRift.initialize();

			// only show severe jme3-logs
			java.util.logging.Logger.getLogger("").setLevel(java.util.logging.Level.SEVERE);

		//	PlatformImpl.startup(() -> {
			//});

			StartPropertiesReader startPropertiesReader = new StartPropertiesReader();

			sim.setSettings(startPropertiesReader.getSettings(sim));

			// show/hide settings screen
			sim.setShowSettings(startPropertiesReader.showSettingsScreen());

			if (!startPropertiesReader.getDrivingTaskPath().isEmpty()
					&& DrivingTask.isValidDrivingTask(new File(startPropertiesReader.getDrivingTaskPath()))) {
				SimulationDefaults.drivingTaskFileName = startPropertiesReader.getDrivingTaskPath();
				sim.drivingTaskGiven = true;
			}

			if (!startPropertiesReader.getDriverName().isEmpty())
				SimulationDefaults.driverName = startPropertiesReader.getDriverName();

			if (args.length >= 1) {
				if (DrivingTask.isValidDrivingTask(new File(args[0]))) {
					SimulationDefaults.drivingTaskFileName = args[0];
					sim.drivingTaskGiven = true;
				}
			}

			if (args.length >= 2) {
				SimulationDefaults.driverName = args[1];
			}

			sim.setPauseOnLostFocus(false);

			sim.start();

		}

		catch (Exception e1) {
			logger.fatal("Could not run main method:", e1);
		}
	}

	private static void copyFile(String sourceString, String targetString) {
		try {

			Path source = Paths.get(sourceString);
			Path target = Paths.get(targetString);

			if (Files.exists(source, LinkOption.NOFOLLOW_LINKS))
				Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
			else
				System.err.println("ERROR: '" + sourceString + "' does not exist.");

		} catch (IOException e) {

			e.printStackTrace();
		}
	}
	public ArrayList<Vector3f> getMultiCarPosition(){
		//System.out.println(multiCarPosition.size());
		return this.multiCarPosition;
	}
	
	public SimulatorCam GetSimulatorCam()
	{
		return (SimulatorCam)this.cameraFactory;
	}
	
	public Hud getHud()
	{
		return hud;
	}
	
}
