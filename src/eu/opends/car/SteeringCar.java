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

package eu.opends.car;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.TreeMap;
import java.util.TreeSet;

import com.jme3.bounding.BoundingBox;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Ray;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.sun.scenario.Settings;

import eu.opends.analyzer.MyCarLogging;
import eu.opends.basics.SimulationBasics;
import eu.opends.car.LightTexturesContainer.TurnSignalState;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.scenario.ScenarioLoader;
import eu.opends.drivingTask.scenario.ScenarioLoader.CarProperty;
import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.environment.Crosswind;
import eu.opends.main.SimulationDefaults;
import eu.opends.main.Simulator;
import eu.opends.multiDriver.MultiAdapter;
import eu.opends.multiDriver.MultiCar;
import eu.opends.tools.Hud;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.Point;
import eu.opends.tools.Sort;
import eu.opends.tools.Util;
import eu.opends.traffic.FollowBox;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;
import eu.opends.traffic.Waypoint;
import eu.opends.trafficObjectLocator.TrafficObjectLocator;

/**
 * Driving Car
 * 
 * @author Rafael Math
 */
public class SteeringCar extends Car {
	// minimum steering percentage to be reached for switching off the turn
	// signal automatically
	// when moving steering wheel back towards neutral position
	private float turnSignalThreshold = 0.25f;

	private TrafficObjectLocator trafficObjectLocator;
	private boolean handBrakeApplied = false;

	// Simphynity Motion Seat
	private DatagramSocket socket;
	private int gameTime = 0;
	private long oldTime = 0;
	private Vector3f localSpeedVector = new Vector3f(0, 0, 0);
	private Vector3f globalIndex = new Vector3f(0, 0, 0);

	// adaptive cruise control
	// private boolean isAdaptiveCruiseControl = false;
	private ArrayList<Waypoint> bezierPointList = new ArrayList<Waypoint>();
	private boolean changeAndKeepingFlag = false;
	private float minLateralSafetyDistance;
	private float minForwardSafetyDistance;
	private float emergencyBrakeDistance;
	private boolean suppressDeactivationByBrake = false;
	private int currentLine = 0;
	private int count = 0;

	private boolean originalLane = false; // CYK 원래의 차선으로 돌아왔는지 판단하는 변수

	private boolean leftLaneChangePossible = true; // CYK 왼쪽으로 차선변경이 가능한지 판별하는
													// 변수
	private boolean rightLaneChangePossible = true; // CYK 오른쪽으로 차선변경이 가능한지 판별하는
													// 변수

	private boolean keepingFlag = false; // CYK 킵핑되고있는지 판별하는 변수
	private boolean overTake = false; // CYK 추월 상태인지 판별하는 변수
	private boolean endLeftLaneChange = false;

	private boolean reduceVelocity = false; // CYK 속도가 감소되고 있는지 판별하는 변수(자동 추월 시)

	private float recentTime = 0.0f; // CYK 현재시간 계산
	private float currentTime = 0.0f; // CYK 속도가 감소됨을 감지했을때의 시간이 저장될 변수

	// crosswind (will influence steering angle)
	private Crosswind crosswind = new Crosswind("left", 0, 0);

	private FollowBox followBox;
	private ScenarioLoader scenarioLoader;
	private int loadingCount = 5;

	public static boolean isInitialLKAS = true;

	public SteeringCar(Simulator sim) {
		this.sim = sim;

		DrivingTask drivingTask = SimulationBasics.getDrivingTask();
		scenarioLoader = drivingTask.getScenarioLoader();

		initialPosition = scenarioLoader.getStartLocation();
		if (initialPosition == null)
			initialPosition = SimulationDefaults.initialCarPosition;

		this.initialRotation = scenarioLoader.getStartRotation();
		if (this.initialRotation == null)
			this.initialRotation = SimulationDefaults.initialCarRotation;

		// add start position as reset position
		Simulator.getResetPositionList().add(new ResetPosition(initialPosition, initialRotation));

		mass = scenarioLoader.getChassisMass();

		minSpeed = scenarioLoader.getCarProperty(CarProperty.engine_minSpeed, SimulationDefaults.engine_minSpeed);
		maxSpeed = scenarioLoader.getCarProperty(CarProperty.engine_maxSpeed, SimulationDefaults.engine_maxSpeed);

		decelerationBrake = scenarioLoader.getCarProperty(CarProperty.brake_decelerationBrake,
				SimulationDefaults.brake_decelerationBrake);
		maxBrakeForce = 0.004375f * decelerationBrake * mass;

		decelerationFreeWheel = scenarioLoader.getCarProperty(CarProperty.brake_decelerationFreeWheel,
				SimulationDefaults.brake_decelerationFreeWheel);
		maxFreeWheelBrakeForce = 0.004375f * decelerationFreeWheel * mass;

		engineOn = scenarioLoader.getCarProperty(CarProperty.engine_engineOn, SimulationDefaults.engine_engineOn);
		if (!engineOn)
			showEngineStatusMessage(engineOn);

		Float lightIntensityObj = scenarioLoader.getCarProperty(CarProperty.light_intensity,
				SimulationDefaults.light_intensity);
		if (lightIntensityObj != null)
			lightIntensity = lightIntensityObj;

		transmission = new Transmission(this);
		powerTrain = new PowerTrain(this);

		modelPath = scenarioLoader.getModelPath();

		init();

		// allows to place objects at current position
		trafficObjectLocator = new TrafficObjectLocator(sim, this);

		// load settings of adaptive cruise control
		// isAdaptiveCruiseControl =
		// scenarioLoader.getCarProperty(CarProperty.cruiseControl_acc,
		// SimulationDefaults.cruiseControl_acc);
		minLateralSafetyDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_safetyDistance_lateral,
				SimulationDefaults.cruiseControl_safetyDistance_lateral);
		minForwardSafetyDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_safetyDistance_forward,
				SimulationDefaults.cruiseControl_safetyDistance_forward);

		emergencyBrakeDistance = scenarioLoader.getCarProperty(CarProperty.cruiseControl_emergencyBrakeDistance,
				SimulationDefaults.cruiseControl_emergencyBrakeDistance);
		suppressDeactivationByBrake = scenarioLoader.getCarProperty(
				CarProperty.cruiseControl_suppressDeactivationByBrake,
				SimulationDefaults.cruiseControl_suppressDeactivationByBrake);

		// if initialSpeed > 0 --> cruise control will be on at startup
		targetSpeedCruiseControl = scenarioLoader.getCarProperty(CarProperty.cruiseControl_initialSpeed,
				SimulationDefaults.cruiseControl_initialSpeed);
		isCruiseControl = (targetSpeedCruiseControl > 0);

		SettingsLoader settingsLoader = SimulationBasics.getSettingsLoader();
		if (settingsLoader.getSetting(Setting.Simphynity_enableConnection,
				SimulationDefaults.Simphynity_enableConnection)) {
			try {
				socket = new DatagramSocket(20778);
			} catch (SocketException e) {
				e.printStackTrace();
			}
		}

		followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
	}

	public TrafficObjectLocator getObjectLocator() {
		return trafficObjectLocator;
	}

	public boolean isHandBrakeApplied() {
		return handBrakeApplied;
	}

	public void applyHandBrake(boolean applied) {
		handBrakeApplied = applied;
	}

	// start applying crosswind and return to 0 (computed in update loop)
	public void setupCrosswind(String direction, float force, int duration) {
		crosswind = new Crosswind(direction, force, duration);
	}

	Vector3f lastVelocity = new Vector3f(0, 0, 0);
	long m_nLastChangeTime = 0;

	// ������ �ӵ��� �̰����� �� �����Ӹ��� �ݿ��� �ȴ�. �ٸ� ������ �ӵ��� �÷��� ���⼭
	// �ݿ�
	// will be called, in every frame
	public void update(float tpf) {
		// accelerate
		float pAccel = 0;
		float x = 0;
		float z = 0;
		float xz = 0;

		Vector3f centerPos = centerGeometry.getWorldTranslation();

		if (loadingCount <= 0) {
			float dis = dis_centerline_to_car();
			MyCarLogging.getInstance().setDistanceFromCenterLine(dis);
			;
			currentLaneInfo(dis);
			if (SteeringCar.isInitialLKAS == true) {
				LaneKeeping(centerPos);
				SteeringCar.isInitialLKAS = false;
			}
		} else {
			loadingCount--;
		}
		// update movement of follow box according to vehicle's position

		if (!engineOn) {
			// apply 0 acceleration when engine not running
			pAccel = powerTrain.getPAccel(tpf, 0) * 30f;
		} else if (isAutoAcceleration && (getCurrentSpeedKmh() < minSpeed)) {
			// apply maximum acceleration (= -1 for forward) to maintain minimum
			// speed
			pAccel = powerTrain.getPAccel(tpf, -1) * 30f;
		} else if (isCruiseControl && (getCurrentSpeedKmh() < targetSpeedCruiseControl)) {
			// apply maximum acceleration (= -1 for forward) to maintain target
			// speed
			pAccel = powerTrain.getPAccel(tpf, -1) * 30f;

			if (isAdaptiveCruiseControl) {
				// lower speed if leading car is getting to close
				// PanelCenter.setDistanceImage();
				if (GetHudFlag())
					Hud.setDistanceImage();
				pAccel = getAdaptivePAccel(pAccel);
			}

		} else {
			// apply acceleration according to gas pedal state
			pAccel = powerTrain.getPAccel(tpf, acceleratorPedalIntensity) * 30f;

			/*
			 * CYK 베지에 곡선 마지막 정점과 자신의 차량 위치와의 점과 점사이 거리를 통해 실시간으로 계산
			 */
			x = globalIndex.x - centerPos.x;
			z = globalIndex.z - centerPos.z;
			xz = (float) Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));

		}
		/*
		 * Start CYK speed UP, speed DOWN 명령이 들어왔을 때 10km 씩 속도를 증/감 현재 지정된 타겟
		 * 스피드에 +10 / -10
		 */
		if (speedUp) {
			System.out.println(targetSpeedCruiseControl);
			targetSpeedCruiseControl = targetSpeedCruiseControl + 10;
			setSpeedUpFlag(false);
		}
		if (speedDown) {
			System.out.println(targetSpeedCruiseControl);
			targetSpeedCruiseControl = targetSpeedCruiseControl - 10;
			setSpeedDownFlag(false);
		}
		/*
		 * End CYK
		 */

		if (isLaneKeepingFlag()) {
			LaneKeeping(centerPos);
		}

		if (isTemperalLaneKeepingFlag()) {
			if (!sim.isPause()) {
				// update steering
				Vector3f wayPoint = followBox.getPosition();
				steerTowardsPosition(wayPoint);
			}

			followBox.update(centerPos, getName());
		}

		/*
		 * CYK 변경하려는 레인에 주변 차량과 안전거리 계산, 변경 가능한 거리라면 차선 변경
		 */
		if (isLeftLaneChange() && leftLaneChangePossible) {
			System.out.println("left lane possible");
			LeftLaneChange(centerPos);
		} else if (isRightLaneChange() && rightLaneChangePossible) {
			System.out.println("Right lane possible");
			RightLaneChange(centerPos);
		}

		/*
		 * CYK 생성된 베지에 곡선에서 마지막 정점을 지나면 레인 킵핑 유지
		 */
		if (xz != 0 && 10 < xz && xz < 14) {
			changeAndKeepingFlag = true;
			if (GetHudFlag()) {
				Hud.SetLeftLaneChangeImage(true);
				Hud.SetRightLaneChangeImage(false);
			}
		} else {
			changeAndKeepingFlag = false;
		}

		/*
		 * CYK 라인변경 후 베지에 곡선의 마지막 정점을 지나 갔을 때 changeAndKeepingFlag변수를 true로 변경하며
		 * 레인 킵핑 유지
		 */
		if (changeAndKeepingFlag) {
			changeAfterKeepingFunc(centerPos);
		}
		/*
		 * CYK 추월 명령이 들어왔을 때 첫번째로 overTake 변수를 true로 변경하여 상태 감지
		 */
		if (overTakeFlag) {

			overTakeLeftLaneChange();
			overTake = true;
		}
		/*
		 * CYK 추월 명령이 들어왔고 레인킵핑, 주변 차량과의 안전거리를 확보했을 때 다시 원래 차선으로 레인 변경 overTake와
		 * originalLane을 false 로 만들어 주면서 추월 명령 완료
		 */
		if (overTake && endLeftLaneChange && keepingFlag && originalLane) {
			overTakeRightLaneChange();
			overTake = false;
			originalLane = false;
		}

		/*
		 * CYK 자율 주행 모드일 때 앞차가 너무 느리거나 이상행동을 보일 시 자동적으로 추월 밑의
		 * belowSafetyDistance() 함수에서 자신의 속도가 원래의 속도보다 어느정도 계속 줄어들 때 이상을 감지,
		 * reduceVelocity 변수를 true로 만들어 자동으로 추월
		 */
		if (isAutoLaneChange() && reduceVelocity) {
			autoLaneChange(centerPos);
			setAutoLaneChangeFlag(false);
			reduceVelocity = false;
		}

		transmission.performAcceleration(pAccel);

		// brake lights
		setBrakeLight(brakePedalIntensity > 0);

		if (handBrakeApplied) {
			// hand brake
			carControl.brake(maxBrakeForce);
			// PanelCenter.setHandBrakeIndicator(true);
		} else {
			// brake
			float appliedBrakeForce = brakePedalIntensity * maxBrakeForce;
			float currentFriction = powerTrain.getFrictionCoefficient() * maxFreeWheelBrakeForce;
			carControl.brake(appliedBrakeForce + currentFriction);
			// PanelCenter.setHandBrakeIndicator(false);
		}

		// lights
		leftHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
		leftHeadLight.setPosition(carModel.getLeftLightPosition());
		leftHeadLight.setDirection(carModel.getLeftLightDirection());

		rightHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
		rightHeadLight.setPosition(carModel.getRightLightPosition());
		rightHeadLight.setDirection(carModel.getRightLightDirection());

		// cruise control indicator
		// if (isCruiseControl)
		// PanelCenter.setCruiseControlIndicator(targetSpeedCruiseControl);
		// else
		// PanelCenter.unsetCruiseControlIndicator();

		trafficObjectLocator.update();

		// switch off turn signal after turn
		if (hasFinishedTurn()) {
			lightTexturesContainer.setTurnSignal(TurnSignalState.OFF);
		}

		lightTexturesContainer.update();

		steeringInfluenceByCrosswind = crosswind.getCurrentSteeringInfluence();

		updateFrictionSlip();

		updateWheel();

		SettingsLoader settingsLoader = SimulationBasics.getSettingsLoader();
		if (settingsLoader.getSetting(Setting.Simphynity_enableConnection,
				SimulationDefaults.Simphynity_enableConnection)) {
			String ip = settingsLoader.getSetting(Setting.Simphynity_ip, SimulationDefaults.Simphynity_ip);
			if (ip == null || ip.isEmpty())
				ip = "127.0.0.1";
			int port = settingsLoader.getSetting(Setting.Simphynity_port, SimulationDefaults.Simphynity_port);

			sendSimphynityInstructions(ip, port);
		}
		// currentLaneInfo();
		/*
		 * SWC 랩뷰와 통신 데이터
		 */
		Car.sAutoGear = (isAdaptiveCruiseControl()) ? 1 : 0;
		Car.sAutoSteer = (isLaneKeepingFlag()) ? 1 : 0;
		Car.sGear = super.getTransmission().getGear();
		Car.sRPM = (int) (super.getTransmission().getRPM());
		Car.sSpeed = (int) (super.getCurrentSpeedKmh());
		Car.sSteer = 10;
		// sendNervtehInstructions("127.0.0.1", 20777);
	}

	/*
	 * SWC 현재 내 차가 몇 라인에 있는지 찾는다.
	 */
	private void currentLaneInfo(float dis) {
		if (0.0f < dis && dis < 3.5f) {
			MyCarLogging.getInstance().setCurrentLine(0);
			MyCarLogging.getInstance().setLeftLaneDistance(dis - 0.0f);
			MyCarLogging.getInstance().setRightLaneDistance(3.5f - dis);
			MyCarLogging.getInstance().setOffsetFromLaneCenter(dis - 1.75f);
			currentLine = 0;
		} else if (3.5f <= dis && dis < 7.0f) {
			MyCarLogging.getInstance().setCurrentLine(1);
			MyCarLogging.getInstance().setLeftLaneDistance(dis - 3.5f);
			MyCarLogging.getInstance().setRightLaneDistance(7.0f - dis);
			MyCarLogging.getInstance().setOffsetFromLaneCenter(dis - 5.25f);
			currentLine = 1;
		} else if (7.0f <= dis && dis < 10.5f) {
			MyCarLogging.getInstance().setCurrentLine(2);
			MyCarLogging.getInstance().setLeftLaneDistance(dis - 7.0f);
			MyCarLogging.getInstance().setRightLaneDistance(10.5f - dis);
			MyCarLogging.getInstance().setOffsetFromLaneCenter(dis - 8.75f);
			currentLine = 2;
		} else if (10.5f <= dis && dis < 14.0f) {
			MyCarLogging.getInstance().setCurrentLine(3);
			MyCarLogging.getInstance().setLeftLaneDistance(dis - 10.5f);
			MyCarLogging.getInstance().setRightLaneDistance(14.0f - dis);
			MyCarLogging.getInstance().setOffsetFromLaneCenter(dis - 12.25f);
			currentLine = 3;
		}
	}

	/*
	 * CYK 추월 명령시 호출되어 왼쪽으로 차선 변경
	 */
	public void overTakeLeftLaneChange() {
		setLeftLaneChange(true);
		// Hud.SetLeftLaneChangeImage(true);
		endLeftLaneChange = true;
		setOverTake(false);
	}

	/*
	 * CYK 추월 명령시 호출되어 오른쪽으로 차선 변경
	 */
	public void overTakeRightLaneChange() {
		setRightLaneChange(true);

		// Hud.SetLeftLaneChangeImage(false);
		// Hud.SetRightLaneChangeImage(true);
		endLeftLaneChange = false;
	}

	/*
	 * CYK high auto mode 일때 주변 차량 이상 감지시 호출되는 함수 레인 변경.
	 */
	public void autoLaneChange(Vector3f centerPos) {

		float dis = dis_centerline_to_car();

		if (0.0f < dis && dis < 3.5f) {
			scenarioLoader.createBezierSettings(bezierPointList(centerPos, 0, 1));
			scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 0, 1));
			followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
			followBox.setToWayPoint(getChangePossibleDistance(centerPos));

			currentLine = 1;
		} else if (3.5f <= dis && dis < 7.0f) {
			scenarioLoader.createBezierSettings(bezierPointList(centerPos, 1, 0));
			scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 1, 0));
			followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
			followBox.setToWayPoint(getChangePossibleDistance(centerPos));

			currentLine = 0;
		} else if (7.0f <= dis && dis < 10.5f) {
			scenarioLoader.createBezierSettings(bezierPointList(centerPos, 2, 1));
			scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 2, 1));
			followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
			followBox.setToWayPoint(getChangePossibleDistance(centerPos));

			currentLine = 1;
		} else if (10.5f <= dis && dis < 14.0f) {
			scenarioLoader.createBezierSettings(bezierPointList(centerPos, 3, 2));
			scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 3, 2));
			followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
			followBox.setToWayPoint(getChangePossibleDistance(centerPos));

			currentLine = 2;
		}
	}

	/*
	 * CYK 라인변경 후 레인 킵핑을 하기위한 함수 currentLine -> 현재 자신의 차량이 위치한 가장 가까운 레인을 계산 차량과
	 * 가장 가까운 앞 정점에서부터 계속 레인 킵핑 keepingFlag 변수를 true로 변경 하면서 레인 킵핑 상태 감지
	 */
	private void changeAfterKeepingFunc(Vector3f centerPos) {

		if (temperalLaneKeepingFlag) {
			if (!sim.isPause()) {
				// update steering
				Vector3f wayPoint = followBox.getPosition();
				steerTowardsPosition(wayPoint);
			}
			followBox.update(centerPos, getName());

			scenarioLoader.getSettings().setWayPoint(scenarioLoader.getFileReader().getWayPointList(currentLine));
			followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
			followBox.setToWayPoint(getKeepingPossibleDistance(centerPos));

			keepingFlag = true;
		}
	}

	/*
	 * CYK 왼쪽으로 레인 체인지를 하는 함수 인자로 자신의 차량의 센터포지션을 인자로 받고 현재 위치한 차선을 계산하여 dis 변수에
	 * 저장 1차선을 제외한 다른 차선에서 왼쪽으로 변경, 이때 베지에곡선 공식을 적용하여 10개의 정점 리스트를 생성하고 그 정점위로
	 * followBox가 지나가도록 설정
	 */
	private void LeftLaneChange(Vector3f centerPos) {
		int output_index;

		if (GetHudFlag()) {
			Hud.SetLeftLaneChangeImage(true);
			Hud.SetRightLaneChangeImage(false);
		}
		if (temperalLaneKeepingFlag) {
			if (!sim.isPause()) {
				// update steering
				Vector3f wayPoint = followBox.getPosition();
				steerTowardsPosition(wayPoint);
			}
			followBox.update(centerPos, getName());

			float dis = dis_centerline_to_car();

			if (0.0f < dis && dis < 3.5f) {

			} else if (3.5f <= dis && dis < 7.0f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 1, 0));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 1, 0));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));

				currentLine = 0;
			} else if (7.0f <= dis && dis < 10.5f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 2, 1));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 2, 1));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));

				currentLine = 1;
			} else if (10.5f <= dis && dis < 14.0f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 3, 2));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 3, 2));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));

				currentLine = 2;
			}

			setLeftLaneChange(false);
		}
	}

	/*
	 * CYK 오른쪽으로 레인 체인지를 하는 함수 인자로 자신의 차량의 센터포지션을 인자로 받고 현재 위치한 차선을 계산하여 dis 변수에
	 * 저장 4차선을 제외한 다른 차선에서 왼쪽으로 변경, 이때 베지에곡선 공식을 적용하여 10개의 정점 리스트를 생성하고 그 정점위로
	 * followBox가 지나가도록 설정
	 */
	private void RightLaneChange(Vector3f centerPos) {

		if (GetHudFlag()) {
			Hud.SetLeftLaneChangeImage(false);
			Hud.SetRightLaneChangeImage(true);
		}

		if (temperalLaneKeepingFlag) {
			if (!sim.isPause()) {
				// update steering
				Vector3f wayPoint = followBox.getPosition();
				steerTowardsPosition(wayPoint);
			}

			followBox.update(centerPos, getName());

			float dis = dis_centerline_to_car();

			if (0.0f < dis && dis < 3.5f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 0, 1));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 0, 1));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));
				currentLine = 1;
			} else if (3.5f <= dis && dis < 7.0f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 1, 2));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 1, 2));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));

				currentLine = 2;
			} else if (7.0f <= dis && dis < 10.5f) {
				scenarioLoader.createBezierSettings(bezierPointList(centerPos, 2, 3));
				scenarioLoader.getBezierSettings().setWayPoint(bezierPointList(centerPos, 2, 3));
				followBox = new FollowBox(sim, this, scenarioLoader.getBezierSettings());
				followBox.setToWayPoint(getChangePossibleDistance(centerPos));

				currentLine = 3;
			} else if (10.5f <= dis && dis < 14.0f) {

			}
			setRightLaneChange(false);
			// Hud.SetRightLaneChangeImage(false);
		}
	}

	/*
	 * CYK 현재 자신의 차량의 위치와 현재의 차선, 변경하려는 차선의 정보를 인자로 전달받아 3차 베지에 곡선을 생성하는 함수를
	 * 호출하여 베지에 곡선을 생성하는 10개의 정점의 리스트를 반환하는 함수
	 */
	public ArrayList<Waypoint> bezierPointList(Vector3f centerPos, int currentLane, int nextLane) {

		Vector3f p0;
		Vector3f p1;
		Vector3f p2;
		Vector3f p3;

		float carSpeed = getCurrentSpeedKmh();

		int index = getIndex(centerPos);

		p0 = scenarioLoader.getSettings().getWayPoints().get(index).getPosition();
		p1 = scenarioLoader.getSettings().getWayPoints().get(index + 1).getPosition();
		p2 = scenarioLoader.getFileReader().getWayPointList(nextLane).get(index + 2).getPosition();
		p3 = scenarioLoader.getFileReader().getWayPointList(nextLane).get(index + 3).getPosition();

		globalIndex = p3;

		float time = 0.0f;

		while (time <= 1) {
			bezierPointList.add(new Waypoint("bezierPoint" + (time),
					new Vector3f(bezier(p0, p1, p2, p3, time).x, 0.0f, bezier(p0, p1, p2, p3, time).z), carSpeed, "s",
					0.0f, "s", 0));
			time += 0.1f;
		}

		return bezierPointList;
	}

	/*
	 * CYK 4개의 점을 가지고 3차 베지에 곡선 공식을 적용 주기(t)는 0에서부터 0.1씩 증가하며 1까지 인자로 전달 받은 것들을
	 * 이용하여 10개의 정점들을 각각 계산하여 vector3f로 반환 p0 - 현재 차량이 위치한 차선에서 자신의 차량의 바로 앞 버텍스
	 * p1 - 현재 차량이 위치한 차선에서 p0의 다음 버텍스 p2 - 변경하려는 차선의 p1의 다음 버텍스 p3 - 변경하려는 차선의
	 * p2의 다음 버텍스
	 */
	public Vector3f bezier(Vector3f p0, Vector3f p1, Vector3f p2, Vector3f p3, float t) {
		Vector3f bezierPoint = new Vector3f(0.0f, 0.0f, 0.0f);

		bezierPoint.x = (float) (p0.x * Math.pow(1 - t, 3)) + (float) (3 * p1.x * t * Math.pow(1 - t, 2))
				+ (float) (3 * p2.x * Math.pow(t, 2) * (1 - t)) + (float) (p3.x * Math.pow(t, 3));
		bezierPoint.y = 0.0f;
		bezierPoint.z = (float) (p0.z * Math.pow(1 - t, 3)) + (float) (3 * p1.z * t * Math.pow(1 - t, 2))
				+ (float) (3 * p2.z * Math.pow(t, 2) * (1 - t)) + (float) (p3.z * Math.pow(t, 3));

		return bezierPoint;
	}

	private void sendNervtehInstructions(String ip, int port) {
		long time = System.currentTimeMillis(); // in milliseconds
		long timeDiffLong = time - oldTime;
		float timeDiff = timeDiffLong / 1000f; // in seconds

		// send updates at most 40 times per second (acceleration changes more
		// stable)
		if (timeDiff > 0.0f) {
			Vector3f globalSpeedVector = this.getCarControl().getLinearVelocity();
			float heading = this.getHeadingDegree() * FastMath.DEG_TO_RAD;
			float speedForward = FastMath.sin(heading) * globalSpeedVector.x
					- FastMath.cos(heading) * globalSpeedVector.z;
			float speedLateral = FastMath.cos(heading) * globalSpeedVector.x
					+ FastMath.sin(heading) * globalSpeedVector.z;
			float speedVertical = globalSpeedVector.y;
			Vector3f currentLocalSpeedVector = new Vector3f(speedForward, speedLateral, speedVertical); // in
																										// m/s
			Vector3f currentLocalAccelerationVector = currentLocalSpeedVector.subtract(localSpeedVector)
					.divide(timeDiff); // in m/s^2

			if (getCurrentSpeedKmh() < 3 && this.getAcceleratorPedalIntensity() < 0.1f)
				currentLocalAccelerationVector.x = 0;

			// System.err.println(currentLocalAccelerationVector.x);

			oldTime = time;
			localSpeedVector = currentLocalSpeedVector;

			try {
				InetAddress adress = InetAddress.getByName(ip);

				String result = "[";

				// localAccel
				float ONE_G_MS = 9.80665f;
				Vector3f smoothCurrentLocalAccelerationVector = doAccelerationSmoothing(currentLocalAccelerationVector);
				result += (Math.max(Math.min(-smoothCurrentLocalAccelerationVector.x, ONE_G_MS), -ONE_G_MS)) + ";"; // -1G
																													// -
																													// +1G
				result += (Math.max(Math.min(-smoothCurrentLocalAccelerationVector.y, ONE_G_MS), -ONE_G_MS)) + ";"; // -1G
																													// -
																													// +1G
				result += (ONE_G_MS - Math.max(Math.min(-smoothCurrentLocalAccelerationVector.z, ONE_G_MS), -ONE_G_MS))
						+ ";"; // 0 - 2G

				// localVel
				Vector3f smoothCurrentLocalSpeedVector = doSpeedSmoothing(currentLocalSpeedVector);
				result += (Math.max(Math.min(smoothCurrentLocalSpeedVector.x / 40f, 1f), 0f)) + ";"; // 0.0
																										// -
																										// 1.0
				result += (Math.max(Math.min(smoothCurrentLocalSpeedVector.y / 40f, 1f), 0f)) + ";"; // 0.0
																										// -
																										// 1.0
				result += (Math.max(Math.min(-smoothCurrentLocalSpeedVector.z / 40f, 1f), 0f)) + ";"; // 0.0
																										// -
																										// 1.0

				// time
				result += time + "]";

				// System.err.println(result);

				final byte[] bytes = result.getBytes();

				DatagramPacket packet = new DatagramPacket(bytes, bytes.length, adress, port);
				socket.send(packet);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private void sendSimphynityInstructions(String ip, int port) {
		long time = System.currentTimeMillis(); // in milliseconds
		long timeDiffLong = time - oldTime;
		float timeDiff = timeDiffLong / 1000f; // in seconds

		// send updates at most 40 times per second (acceleration changes more
		// stable)
		if (timeDiff > /* 0.025f */ 0.0f) {
			Vector3f globalSpeedVector = this.getCarControl().getLinearVelocity();
			float heading = this.getHeadingDegree() * FastMath.DEG_TO_RAD;
			float speedForward = FastMath.sin(heading) * globalSpeedVector.x
					- FastMath.cos(heading) * globalSpeedVector.z;
			float speedLateral = FastMath.cos(heading) * globalSpeedVector.x
					+ FastMath.sin(heading) * globalSpeedVector.z;
			float speedVertical = globalSpeedVector.y;
			Vector3f currentLocalSpeedVector = new Vector3f(speedForward, speedLateral, speedVertical); // in
																										// m/s
			Vector3f currentLocalAccelerationVector = currentLocalSpeedVector.subtract(localSpeedVector)
					.divide(timeDiff); // in m/s^2

			if (getCurrentSpeedKmh() < 3 && this.getAcceleratorPedalIntensity() < 0.1f)
				currentLocalAccelerationVector.x = 0;

			// System.err.println(currentLocalAccelerationVector.x);

			oldTime = time;
			localSpeedVector = currentLocalSpeedVector;

			try {
				InetAddress adress = InetAddress.getByName(ip);

				final ByteArrayOutputStream baos = new ByteArrayOutputStream();
				final DataOutputStream daos = new DataOutputStream(baos);

				// useLocalVals
				daos.writeBoolean(true);
				daos.writeBoolean(false);
				daos.writeBoolean(false);
				daos.writeBoolean(false);

				// localAccel
				float ONE_G_MS = 9.80665f;
				Vector3f smoothCurrentLocalAccelerationVector = doAccelerationSmoothing(currentLocalAccelerationVector);
				daos.writeFloat(
						convertFloat(Math.max(Math.min(-smoothCurrentLocalAccelerationVector.x, ONE_G_MS), -ONE_G_MS))); // -1G
																															// -
																															// +1G
				daos.writeFloat(
						convertFloat(Math.max(Math.min(-smoothCurrentLocalAccelerationVector.y, ONE_G_MS), -ONE_G_MS))); // -1G
																															// -
																															// +1G
				daos.writeFloat(convertFloat(
						ONE_G_MS - Math.max(Math.min(-smoothCurrentLocalAccelerationVector.z, ONE_G_MS), -ONE_G_MS))); // 0
																														// -
																														// 2G

				// localVel
				Vector3f smoothCurrentLocalSpeedVector = doSpeedSmoothing(currentLocalSpeedVector);
				daos.writeFloat(convertFloat(Math.max(Math.min(smoothCurrentLocalSpeedVector.x / 40f, 1f), 0f))); // 0.0
																													// -
																													// 1.0
				daos.writeFloat(convertFloat(Math.max(Math.min(smoothCurrentLocalSpeedVector.y / 40f, 1f), 0f))); // 0.0
																													// -
																													// 1.0
				daos.writeFloat(convertFloat(Math.max(Math.min(-smoothCurrentLocalSpeedVector.z / 40f, 1f), 0f))); // 0.0
																													// -
																													// 1.0

				// globalVel
				Vector3f globalVelocity = getCarControl().getLinearVelocity().divide(40f);
				daos.writeFloat(convertFloat(globalVelocity.x));
				daos.writeFloat(convertFloat(globalVelocity.y));
				daos.writeFloat(convertFloat(-globalVelocity.z));

				// rotationMatrix
				Matrix3f rotationMatrix = getRotation().toRotationMatrix();
				daos.writeFloat(convertFloat(rotationMatrix.get(0, 0)));
				daos.writeFloat(convertFloat(rotationMatrix.get(0, 1)));
				daos.writeFloat(convertFloat(rotationMatrix.get(0, 2)));
				daos.writeFloat(convertFloat(rotationMatrix.get(1, 0)));
				daos.writeFloat(convertFloat(rotationMatrix.get(1, 1)));
				daos.writeFloat(convertFloat(rotationMatrix.get(1, 2)));
				daos.writeFloat(convertFloat(rotationMatrix.get(2, 0)));
				daos.writeFloat(convertFloat(rotationMatrix.get(2, 1)));
				daos.writeFloat(convertFloat(rotationMatrix.get(2, 2)));

				// packetTimeMillis
				if (!sim.isPause() && timeDiffLong < 100000)
					gameTime += timeDiffLong;

				daos.writeInt(convertInt(gameTime));

				daos.close();

				final byte[] bytes = baos.toByteArray();

				DatagramPacket packet = new DatagramPacket(bytes, bytes.length, adress, port);
				socket.send(packet);

				// System.err.println("Sim: " + globalSpeedVector);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private int smoothingFactor = 10;
	private LinkedList<Vector3f> speedStorage = new LinkedList<Vector3f>();

	private Vector3f doSpeedSmoothing(Vector3f speed) {
		Vector3f sum = new Vector3f(0, 0, 0);

		speedStorage.addLast(speed);

		for (Vector3f vector : speedStorage)
			sum.addLocal(vector);

		if (speedStorage.size() >= smoothingFactor)
			speedStorage.removeFirst();

		return sum.divide(smoothingFactor);
	}

	private LinkedList<Vector3f> accelerationStorage = new LinkedList<Vector3f>();

	private Vector3f doAccelerationSmoothing(Vector3f acceleration) {
		Vector3f sum = new Vector3f(0, 0, 0);

		accelerationStorage.addLast(acceleration);

		for (Vector3f vector : accelerationStorage)
			sum.addLocal(vector);

		if (accelerationStorage.size() >= smoothingFactor)
			accelerationStorage.removeFirst();

		return sum.divide(smoothingFactor);
	}

	private float convertFloat(float in) {
		final ByteArrayOutputStream baos = new ByteArrayOutputStream();
		final DataOutputStream daos = new DataOutputStream(baos);

		try {
			daos.writeFloat(in);
		} catch (IOException e) {
			e.printStackTrace();
		}
		byte[] byteArray = baos.toByteArray();

		ByteBuffer bb = ByteBuffer.wrap(byteArray);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		return bb.getFloat();
	}

	private int convertInt(int in) {
		final ByteArrayOutputStream baos = new ByteArrayOutputStream();
		final DataOutputStream daos = new DataOutputStream(baos);

		try {
			daos.writeFloat(in);
		} catch (IOException e) {
			e.printStackTrace();
		}
		byte[] byteArray = baos.toByteArray();

		ByteBuffer bb = ByteBuffer.wrap(byteArray);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		return bb.getInt();
	}

	float leftWheelsPos = 2.2f;
	float backAxleHeight = -3.0f;
	float backAxlePos = 2.45f;
	long prevTime = 0;

	private void updateWheel() {
		long time = System.currentTimeMillis();
		if (time - prevTime > 1000) {/*
										 * Vector3f wheelDirection = new
										 * Vector3f(0, -1, 0); Vector3f
										 * wheelAxle = new Vector3f(-1, 0, 0);
										 * float wheelRadius = 0.5f; float
										 * suspensionLenght = 0.2f;
										 * 
										 * carControl.removeWheel(3);
										 * 
										 * backAxlePos += 0.05f;
										 * 
										 * // add back left wheel Geometry
										 * geom_wheel_fl =
										 * Util.findGeom(carNode,
										 * "WheelBackLeft");
										 * geom_wheel_fl.setLocalScale(
										 * wheelRadius*2);
										 * geom_wheel_fl.center(); BoundingBox
										 * box = (BoundingBox)
										 * geom_wheel_fl.getModelBound();
										 * carControl.addWheel(geom_wheel_fl.
										 * getParent(),
										 * box.getCenter().add(leftWheelsPos,
										 * backAxleHeight, backAxlePos),
										 * wheelDirection, wheelAxle,
										 * suspensionLenght, wheelRadius, true);
										 * 
										 * System.out.println("backAxlePos: " +
										 * backAxlePos);
										 * 
										 * prevTime = time;
										 */
		}
		// System.out.println("prevTime: " + prevTime + " time: " + time);
	}

	private void updateFrictionSlip() {
		/*
		 * // ice carControl.setRollInfluence(0, 0.5f);
		 * carControl.setRollInfluence(1, 0.5f); carControl.setRollInfluence(2,
		 * 0.5f); carControl.setRollInfluence(3, 0.5f);
		 * 
		 * carControl.setFrictionSlip(0, 1f); carControl.setFrictionSlip(1, 1f);
		 * carControl.setFrictionSlip(2, 1f); carControl.setFrictionSlip(3, 1f);
		 */
	}

	private boolean hasStartedTurning = false;

	private boolean hasFinishedTurn() {
		TurnSignalState turnSignalState = lightTexturesContainer.getTurnSignal();
		float steeringWheelState = getSteeringWheelState();

		if (turnSignalState == TurnSignalState.LEFT) {
			if (steeringWheelState > turnSignalThreshold)
				hasStartedTurning = true;
			else if (hasStartedTurning) {
				hasStartedTurning = false;
				return true;
			}
		}

		if (turnSignalState == TurnSignalState.RIGHT) {
			if (steeringWheelState < -turnSignalThreshold)
				hasStartedTurning = true;
			else if (hasStartedTurning) {
				hasStartedTurning = false;
				return true;
			}
		}

		return false;
	}

	// Adaptive Cruise Control
	// ***************************************************

	private float getAdaptivePAccel(float pAccel) {
		brakePedalIntensity = 0f;

		// check distance from traffic vehicles
		for (TrafficObject vehicle : PhysicalTraffic.getTrafficObjectList()) {
			if (belowSafetyDistance(vehicle.getPosition(), PanelCenter.distanceSet)) {
				pAccel = 0;
				MyCarLogging.getInstance().setFrontCarName(vehicle.getName());
				MyCarLogging.getInstance().setFrontCarDistance(vehicle.getPosition().distance(getPosition()));
				if (vehicle.getPosition().distance(getPosition()) < emergencyBrakeDistance)
					brakePedalIntensity = 1f;
			}
		}
		/* VERSION UP START */
		for (MultiCar car : MultiAdapter.getInstance().getMulticarList()) {
			Vector3f multiPos = new Vector3f(car.getxPosition(), 0, car.getzPosition());
			if (belowSafetyDistance(multiPos, PanelCenter.distanceSet)) {
				pAccel = 0;
				MyCarLogging.getInstance().setFrontCarName(car.getId());
				MyCarLogging.getInstance().setFrontCarDistance(multiPos.distance(getPosition()));
				if (multiPos.distance(getPosition()) < emergencyBrakeDistance)
					brakePedalIntensity = 1f;
			}
		}

		if (PanelCenter.distanceSet.size() != 0) {
			PanelCenter.distanceSet.clear();
		}
		if (Hud.distanceSet.size() != 0) {
			Hud.distanceSet.clear();
		}
		/* VERSION UP END */
		return pAccel;
	}

	/*
	 * 앞차와의 거리, 양 옆차의 거리를 계산하여 안전거리 이내에 들어왔는지 검사
	 */
	private boolean belowSafetyDistance(Vector3f obstaclePos, TreeSet<Float> distanceSet) {
		float distance = obstaclePos.distance(getPosition());
		// angle between driving direction of traffic car and direction towards
		// obstacle
		// (consider 3D space, because obstacle could be located on a bridge
		// above traffic car)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float angle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, obstaclePos, false);

		float lateralDistance = distance * FastMath.sin(angle);
		float forwardDistance = distance * FastMath.cos(angle);

		/* VERSION UP START */
		float lateralDistanceSignal = lateralDistance;
		lateralDistance = Math.abs(distance * FastMath.sin(angle));

		distanceSet.add(forwardDistance);

		float dis;
		int cnt = 0;
		Iterator iter = distanceSet.iterator();
		while (iter.hasNext()) {
			dis = (float) iter.next();

			if (dis > 0) {
				// PanelCenter.distance = dis;
				if (GetHudFlag())
					Hud.distance = dis;
				break;
			}
			cnt++;
			if (cnt == PhysicalTraffic.getTrafficObjectList().size()) {
				// PanelCenter.distance = 100000;
				if (GetHudFlag())
					Hud.distance = 100000;
			}

		}

		if ((lateralDistance < minLateralSafetyDistance) && (forwardDistance > 0) && (lateralDistance < 2)
				&& (forwardDistance < Math.max(0.5f * getCurrentSpeedKmh(), minForwardSafetyDistance))) {

			/*
			 * Start CYK higly auto 모드일 때 지정된 속도보다 10초 이상 속도 감소를 감지 했을 때 시간을 계속
			 * 업데이트하면서 recentTime, currentTime을 구하고 속도 감소를 감지하면 recentTime은 감지한
			 * 그 시점을 저장하고 있고, currentTime은 계속 업데이트 되면서 그 둘의 차이가 10초 이상 벌어 졌을 때
			 * 자동으로 차선 변경 후 시간 초기화하여 다시 측정
			 */
			if (count == 0)
				recentTime = getTime(new Date());
			else
				currentTime = getTime(new Date());

			count = 1;

			if (currentTime - recentTime > 10) {
				System.out.println("Auto Change!!!!!!!");
				reduceVelocity = true;
				count = 0;
				recentTime = 0.0f;
				currentTime = 0.0f;
			}

			/* End CYK */

			return true;
		}

		/*
		 * CYK 주변 차량과 안전거리 계산하여 6 이상 벌어져 안전거리가 확보 됬을 때 원래의 라인으로 돌아오게 하기 위한 실행문
		 */
		if (Math.abs(forwardDistance * lateralDistance) > 6) {
			originalLane = true;
		}

		/*
		 * Start CYK 일반적인 차선 변경시 변경하려는 차선에 차량과 부딪힐 위험을 판단하는 실행문 4이상 차이가 나면 안전거리로
		 * 판단하여 차선 변경 명령 실시
		 */
		switch (currentLine) {
		case 0:
			leftLaneChangePossible = true;
			rightLaneChangePossible = true;
			if (lateralDistance < 4)
				rightLaneChangePossible = false;
			break;
		case 1:
			leftLaneChangePossible = true;
			rightLaneChangePossible = true;
			if (lateralDistanceSignal < 0 && lateralDistance < 4)
				leftLaneChangePossible = false;
			else if (lateralDistanceSignal > 0 && lateralDistance < 4)
				rightLaneChangePossible = false;
			else {
				leftLaneChangePossible = true;
				rightLaneChangePossible = true;
			}
			break;
		case 2:
			leftLaneChangePossible = true;
			rightLaneChangePossible = true;
			if (lateralDistanceSignal < 0 && lateralDistance < 4)
				leftLaneChangePossible = false;
			else if (lateralDistanceSignal > 0 && lateralDistance < 4)
				rightLaneChangePossible = false;
			else {
				leftLaneChangePossible = true;
				rightLaneChangePossible = true;
			}
			break;
		case 3:
			leftLaneChangePossible = true;
			rightLaneChangePossible = true;
			if (lateralDistance < 4)
				leftLaneChangePossible = false;
			break;
		}
		/* End CYK */

		/* VERSION UP END */
		return false;
	}

	public void increaseCruiseControl(float diff) {
		targetSpeedCruiseControl = Math.min(targetSpeedCruiseControl + diff, 260.0f);
	}

	public void decreaseCruiseControl(float diff) {
		targetSpeedCruiseControl = Math.max(targetSpeedCruiseControl - diff, 0.0f);
	}

	public void disableCruiseControlByBrake() {
		if (!suppressDeactivationByBrake) {
			setCruiseControl(false);
			setAdaptiveCruiseControl(false);
		}
	}

	private float getTime(Date string2) {
		float time = 0.0f;

		int hour = Integer.parseInt(new SimpleDateFormat("HH").format(string2));
		int min = Integer.parseInt(new SimpleDateFormat("mm").format(string2));
		float sec = Float.parseFloat(new SimpleDateFormat("ss.SSS").format(string2));

		time = (hour * 3600) + (min * 60);
		time += sec;

		return time;
	}

	// Adaptive Cruise Control
	// ***************************************************

	public float getDistanceToRoadSurface() {
		// reset collision results list
		CollisionResults results = new CollisionResults();

		// aim a ray from the car's center downwards to the road surface
		Ray ray = new Ray(getPosition(), Vector3f.UNIT_Y.mult(-1));

		// collect intersections between ray and scene elements in results list.
		sim.getSceneNode().collideWith(ray, results);

		// return the result
		for (int i = 0; i < results.size(); i++) {
			// for each hit, we know distance, contact point, name of geometry.
			float dist = results.getCollision(i).getDistance();
			Geometry geometry = results.getCollision(i).getGeometry();

			if (geometry.getName().contains("CityEngineTerrainMate"))
				return dist - 0.07f;
		}

		return -1;
	}
	/* VERSION UP START */

	// Lane Keeping Assist System
	// ***************************************************
	/*
	 * SWC 내 차와 도로의 중앙선으로 부터 떨어진 거리를 계산 후 해당 라인위에 follow box를 생성한다.
	 */
	public void LaneKeeping(Vector3f centerPos) {
		if (!sim.isPause()) {
			// update steering
			Vector3f wayPoint = followBox.getPosition();
			steerTowardsPosition(wayPoint);
		}
		// update movement of follow box according to vehicle's position
		// Vector3f centerPos = centerGeometry.getWorldTranslation();
		followBox.update(centerPos, getName());

		float dis = dis_centerline_to_car();

		if (0.0f < dis && dis < 3.5f) {
			scenarioLoader.getSettings().setWayPoint(scenarioLoader.getFileReader().getWayPointList(0));
			followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
		} else if (3.5f <= dis && dis < 7.0f) {
			scenarioLoader.getSettings().setWayPoint(scenarioLoader.getFileReader().getWayPointList(1));
			followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
		} else if (7.0f <= dis && dis < 10.5f) {
			scenarioLoader.getSettings().setWayPoint(scenarioLoader.getFileReader().getWayPointList(2));
			followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
		} else if (10.5f <= dis && dis < 14.0f) {
			scenarioLoader.getSettings().setWayPoint(scenarioLoader.getFileReader().getWayPointList(3));
			followBox = new FollowBox(sim, this, scenarioLoader.getSettings());
		}

		followBox.setToWayPoint(getCurrentMinimumDistance(centerPos));
		setTemperalLaneKeepingFlag(true);
		setLaneKeepingFlag(false);

	}

	// ���� ����� �� ���� ���Ͽ� �� ����� ���� ������ ���� �Ÿ��� ��� ����� �����
	/*
	 * SWC 중앙선 위의 점들 중 내 차와 가장 가까운 2개의 점을 찾아 중앙선으로 부터 떨어진 거리를 계산한다.
	 */

	private float dis_centerline_to_car() {
		float distance = -1;
		Vector3f centerLinePoint;
		float currentDistance = -1;
		float angle;

		float posdis = Float.MAX_VALUE;
		float negdis = -10000.0f;

		int posindex = -1;
		int negindex = -1;
		short flag = 0;

		// get traffic object's position on xz-plane (ignore y component)
		Vector3f carPosition = new Vector3f();
		carPosition = centerGeometry.getWorldTranslation();
		carPosition.setY(0);

		// Hwancheol question : 왜 중앙선과 차량의 거리를 모든 중앙선에 대해서 재는가?
		// TODO : 낭비되는 코드 줄이기!
		for (int i = 0; i < scenarioLoader.getXmlParser().centerLinePosition.length; i++) {
			centerLinePoint = scenarioLoader.getXmlParser().centerLinePosition[i];

			// distance between box and trafficObject
			currentDistance = centerLinePoint.distance(carPosition);

			// �� ���� ��������Ʈ�� ����
			angle = Util.getAngleBetweenPoints(frontGeometry.getWorldTranslation(), carPosition, centerLinePoint,
					false);


			if (FastMath.cos(angle) >= 0) {
				if (currentDistance < posdis) {
					posdis = currentDistance;
					posindex = i;
				}
			} else {
				if (currentDistance < Math.abs(negdis)) {
					negdis = currentDistance * (-1);
					negindex = i;
				}
			}
		}
		float m = (scenarioLoader.getXmlParser().centerLinePosition[posindex].z
				- scenarioLoader.getXmlParser().centerLinePosition[negindex].z)
				/ (scenarioLoader.getXmlParser().centerLinePosition[posindex].x
						- scenarioLoader.getXmlParser().centerLinePosition[negindex].x);

		distance = (float) (Math
				.abs(m * carPosition.x - carPosition.z
						+ (scenarioLoader.getXmlParser().centerLinePosition[posindex].z
								- m * scenarioLoader.getXmlParser().centerLinePosition[posindex].x))
				/ Math.sqrt(Math.pow(m, 2) + 1));

		return distance;
	}

	@Override
	public String getName() {
		return "mycar";
	}

	private void steerTowardsPosition(Vector3f wayPoint) {
		// get relative position of way point --> steering direction
		// -1: way point is located on the left side of the vehicle
		// 0: way point is located in driving direction
		// 1: way point is located on the right side of the vehicle
		int steeringDirection = getRelativePosition(wayPoint);

		// get angle between driving direction and way point direction -->
		// steering intensity
		// only consider 2D space (projection of WPs to xz-plane)
		Vector3f carFrontPos = frontGeometry.getWorldTranslation();
		Vector3f carCenterPos = centerGeometry.getWorldTranslation();
		float steeringAngle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, wayPoint, true);

		// compute steering intensity in percent
		// 0 degree = 0%
		// 22,5 degree = 50%
		// 45 degree = 100%
		// >45 degree = 100%
		float steeringIntensity = Math.max(Math.min(4 * steeringAngle / FastMath.PI, 1f), 0f);

		// apply steering instruction
		float temp = steeringDirection * steeringIntensity;
		System.out.println(temp);
		if (Math.abs(temp) < 0.50f) {
			steer(temp);
		}
	}

	private int getRelativePosition(Vector3f wayPoint) {
		// get vehicles center point and point in driving direction
		Vector3f frontPosition = frontGeometry.getWorldTranslation();
		Vector3f centerPosition = centerGeometry.getWorldTranslation();

		// convert Vector3f to Point2D.Float, as needed for Line2D.Float
		Point2D.Float centerPoint = new Point2D.Float(centerPosition.getX(), centerPosition.getZ());
		Point2D.Float frontPoint = new Point2D.Float(frontPosition.getX(), frontPosition.getZ());

		// line in direction of driving
		Line2D.Float line = new Line2D.Float(centerPoint, frontPoint);

		// convert Vector3f to Point2D.Float
		Point2D point = new Point2D.Float(wayPoint.getX(), wayPoint.getZ());

		// check way point's relative position to the line
		if (line.relativeCCW(point) == -1) {
			// point on the left --> return -1
			return -1;
		} else if (line.relativeCCW(point) == 1) {
			// point on the right --> return 1
			return 1;
		} else {
			// point on line --> return 0
			return 0;
		}
	}

	/*
	 * SWC 웨이포인트 상의 점들 중 내 차의 전방에 있는 가장 가까운 1개의 점의 인덱스를 찾는다.
	 */
	public int getCurrentMinimumDistance(Vector3f carPos) {
		Vector3f wayPointPosition;
		float currentDistance = -1;
		float angle;
		float posdis = Float.MAX_VALUE;
		int posindex = -1;

		// get traffic object's position on xz-plane (ignore y component)
		Vector3f carPosition = carPos;
		carPosition.setY(0);

		for (int i = 0; i < scenarioLoader.getSettings().getWayPoints().size(); i++) {
			wayPointPosition = scenarioLoader.getSettings().getWayPoints().get(i).getPosition();

			// distance between box and trafficObject
			currentDistance = wayPointPosition.distance(carPosition);

			// �� ���� ��������Ʈ�� ����
			angle = Util.getAngleBetweenPoints(frontGeometry.getWorldTranslation(), carPos, wayPointPosition, false);

			// distance = currentDistance * FastMath.cos(angle);

			if (FastMath.cos(angle) >= 0) {
				if (currentDistance < posdis) {
					posdis = currentDistance;
					posindex = i;
				}
			}
		}

		return posindex;
	}

	public int getChangePossibleDistance(Vector3f carPos) {
		Vector3f wayPointPosition;
		float currentDistance = -1;
		float angle;
		float distance = -1;

		float posdis = Float.MAX_VALUE;

		int posindex = -1;
		Vector3f carPosition = carPos;
		carPosition.setY(0);

		for (int i = 0; i < scenarioLoader.getBezierSettings().getWayPoints().size(); i++) {
			wayPointPosition = scenarioLoader.getBezierSettings().getWayPoints().get(i).getPosition();

			// distance between box and trafficObject
			currentDistance = wayPointPosition.distance(carPosition);

			// �� ���� ��������Ʈ�� ����
			angle = Util.getAngleBetweenPoints(frontGeometry.getWorldTranslation(), carPos, wayPointPosition, false);

			distance = currentDistance * FastMath.cos(angle);

			if (FastMath.cos(angle) >= 0) {
				if (currentDistance < posdis) {
					posdis = currentDistance;
					posindex = i;
				}
			}
		}
		return posindex;
	}

	public int getKeepingPossibleDistance(Vector3f carPos) {
		Vector3f wayPointPosition;
		float currentDistance = -1;
		float angle;
		float distance = -1;

		float posdis = Float.MAX_VALUE;

		int posindex = -1;
		Vector3f carPosition = carPos;
		carPosition.setY(0);

		for (int i = 0; i < scenarioLoader.getSettings().getWayPoints().size(); i++) {
			wayPointPosition = scenarioLoader.getSettings().getWayPoints().get(i).getPosition();

			// distance between box and trafficObject
			currentDistance = wayPointPosition.distance(carPosition);

			// �� ���� ��������Ʈ�� ����
			angle = Util.getAngleBetweenPoints(frontGeometry.getWorldTranslation(), carPos, wayPointPosition, false);

			distance = currentDistance * FastMath.cos(angle);

			if (FastMath.cos(angle) >= 0) {
				if (currentDistance < posdis) {
					posdis = currentDistance;
					posindex = i;
				}
			}
		}
		return posindex;
	}

	public int getIndex(Vector3f carPos) {
		Vector3f wayPointPosition;
		float currentDistance = -1;
		float angle;
		float distance = -1;

		float posdis = Float.MAX_VALUE;

		int posindex = -1;
		Vector3f carPosition = carPos;
		carPosition.setY(0);

		for (int i = 0; i < scenarioLoader.getSettings().getWayPoints().size(); i++) {
			wayPointPosition = scenarioLoader.getSettings().getWayPoints().get(i).getPosition();

			// distance between box and trafficObject
			currentDistance = wayPointPosition.distance(carPosition);

			// �� ���� ��������Ʈ�� ����
			angle = Util.getAngleBetweenPoints(frontGeometry.getWorldTranslation(), carPos, wayPointPosition, false);

			distance = currentDistance * FastMath.cos(angle);

			if (FastMath.cos(angle) >= 0) {
				if (currentDistance < posdis) {
					posdis = currentDistance;
					posindex = i;
				}
			}
		}

		return posindex;
	}

	public int getCurrentLine() {
		return currentLine;
	}

	public void setCurrentLine(int currentLine) {
		this.currentLine = currentLine;
	}
	/* VERSION UP END */

	@Override
	public void setToWayPoint(String wayPointID) {
		// TODO Auto-generated method stub

	}

	@Override
	public void update(float tpf, ArrayList<TrafficObject> vehicleList) {
		// TODO Auto-generated method stub

	}
}