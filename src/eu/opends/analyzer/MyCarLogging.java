package eu.opends.analyzer;

import java.util.ArrayList;
import java.util.Date;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;


import eu.opends.car.LightTexturesContainer.TurnSignalState;


/*
 * Start --- KSS
 *  운전자차량의 정보를 가지고 있는 객체 클래스
 *  */

public class MyCarLogging {

	/*
	 * KSS
	 * 객체생성이 여러번 이루어지지 않게 싱글톤패턴을 적용한다.
	 * 하나의 객체생성을 통해 여러 클래스에서 접근가능하게 한다.
	 * */ 
	private static MyCarLogging instance;

	public static MyCarLogging getInstance() {
		if (instance == null) {
			instance = new MyCarLogging();
		}
		return instance;
	}
	
	
	/*
	 * KSS
	 * 운전자차량 객체 정보들
	 * */
	
	private String name; // 드라이버이름
	private Vector3f position; // 차량 위치
	private float brake; // 브레이크 압력
	private float speed; // 차량 속도
	private float steerState; // steer 상태
	private float steeringAngle; // steer각도
	private TurnSignalState signal; // 방향지시등
	private Quaternion rotation; // 차량 방향
	private float rpm; // rpm
	private float acceleration; // 차량 가속도
	private String frontCarName; // 전방 차량 아이디
	private float frontCarDistance; // 전방 차량과의 상대거리
	private float distanceFromCenterLine;
	private int currentLine = -1; // 현재 차선
	private float leftLaneeDistance; // 왼쪽 차선까지의 거리
	private float rightLaneDistance; // 오른쪽 차선까지의 거리
	private float offsetFromLaneCenter;
	
	private Vector3f prePosition = null;  //이전 위치
	private long preDateForVel;
	private long preDateForAcc;
	private long preDateForAcceleration;
	private float preSpeed;
	private float velocity_x;  //x축 속도
	private float velocity_z;  //z축 속도
	private float preVelocity_x;  //x축 이전 속도
	private float preVelocity_z;  //z축 이전 속도
	private float acceleration_x;  //x축 가속도
	private float acceleration_z;  //z축 가속도
	
	
	private float headTraker_pitch; // 해드트래킹 pitch 
	private float headTraker_yaw; // 해드트래킹 yaw
	private float headTraker_roll; // 해드트래킹 roll
	private float headTraker_x;	// 해드트래킹 x 좌표
	private float headTraker_y; // 해드트래킹 y 좌표
	private float headTraker_z; // 해드트래킹 z 좌표
	
	private STATE state = STATE.NORMAL;  // 차량 운전자 상태
	
	
	public enum STATE // 보통, 난폭, 졸림, 만취 
	{
		NORMAL, DISTRACTION, DROWSINESS, DRUNKEN
	}
	
	
	/*
	 * KSS
	 * 운전자 차량의 객체정보들을 설정하거나 얻을 수 있는 메소드
	 *  */
	
	
	public void setState(STATE state){
		this.state = state;
	}
	
	public STATE getState(){
		return state;
	}
	
	public float getHeadTraker_pitch() {
		return headTraker_pitch;
	}

	public void setHeadTraker_pitch(float headTraker_pitch) {
		this.headTraker_pitch = headTraker_pitch;
	}

	public float getHeadTraker_yaw() {
		return headTraker_yaw;
	}

	public void setHeadTraker_yaw(float headTraker_yaw) {
		this.headTraker_yaw = headTraker_yaw;
	}

	public float getHeadTraker_roll() {
		return headTraker_roll;
	}

	public void setHeadTraker_roll(float headTraker_roll) {
		this.headTraker_roll = headTraker_roll;
	}

	public float getHeadTraker_x() {
		return headTraker_x;
	}

	public void setHeadTraker_x(float headTraker_x) {
		this.headTraker_x = headTraker_x;
	}

	public float getHeadTraker_y() {
		return headTraker_y;
	}

	public void setHeadTraker_y(float headTraker_y) {
		this.headTraker_y = headTraker_y;
	}

	public float getHeadTraker_z() {
		return headTraker_z;
	}

	public void setHeadTraker_z(float headTraker_z) {
		this.headTraker_z = headTraker_z;
	}

	
	public void SetPreSpeed(float vel)
	{
		this.preSpeed = vel;
		this.preDateForAcceleration = System.currentTimeMillis();
	}
	
	public void SetAcceleration()
	{
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.acceleration =  (speed - preSpeed) / ((System.currentTimeMillis() - preDateForAcceleration));
	}
	
	public float GetAcceleration()
	{
		return this.acceleration;
	}
	
	public float getSpeed() {
		return speed;
	}

	public void setSpeed(float speed) {
		this.speed = speed;
	}
	
	public void SetPreVelocity_x(float vel)
	{
		this.preVelocity_x = vel;
		this.preDateForAcc = System.currentTimeMillis();
	}
	
	public void SetPreVelocity_z(float vel)
	{
		this.preVelocity_z = vel;
	}
	
	public void SetAcceleration_X()
	{
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.acceleration_x = (velocity_x - preVelocity_x)/ ((System.currentTimeMillis() - preDateForAcc));
	}
	
	public float GetAcceleration_X()
	{
		return this.acceleration_x;
	}
	
	public float GetAcceleration_Z()
	{
		return this.acceleration_z;
	}
	
	public void SetAcceleration_Z()
	{
		this.acceleration_z = (velocity_z - preVelocity_z)/ ((System.currentTimeMillis() - preDateForAcc));
	}
	
	public void SetVelocity_X()
	{
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.velocity_x = (position.x - prePosition.x) /((System.currentTimeMillis()-preDateForVel) );
	}
	
	public float GetVelocity_X()
	{
		return this.velocity_x;
	}
	
	public void SetVelocity_Z()
	{
		this.velocity_z = (position.z - prePosition.z)/((System.currentTimeMillis()-preDateForVel));
	}
	
	public float GetVelocity_Z()
	{
		return this.velocity_z;
	}
		
	public void SetPrePosition(Vector3f pos)
	{
		this.prePosition = pos;
		preDateForVel = System.currentTimeMillis();
	}
	
	public Vector3f GetPrePosition()
	{
		return this.prePosition;
	}
	
	public float getDistanceFromCenterLine() {
		return distanceFromCenterLine;
	}

	public void setDistanceFromCenterLine(float distanceFromCenterLine) {
		this.distanceFromCenterLine = distanceFromCenterLine;
	}

	
	public int getCurrentLine() {
		return currentLine;
	}

	public void setCurrentLine(int currentLine) {
		this.currentLine = currentLine;
	}

	public float getLeftLaneDistance() {
		return leftLaneeDistance;
	}

	public void setLeftLaneDistance(float leftLaneeDistance) {
		this.leftLaneeDistance = leftLaneeDistance;
	}

	public float getRightLaneDistance() {
		return rightLaneDistance;
	}

	public void setRightLaneDistance(float rightLaneDistance) {
		this.rightLaneDistance = rightLaneDistance;
	}

	public float getOffsetFromLaneCenter() {
		return offsetFromLaneCenter;
	}

	public void setOffsetFromLaneCenter(float offsetFromLaneCenter) {
		this.offsetFromLaneCenter = offsetFromLaneCenter;
	}
	public float getRpm() {
		return rpm;
	}

	public void setRpm(float rpm) {
		this.rpm = rpm;
	}

	public String getFrontCarName() {
		return frontCarName;
	}

	public void setFrontCarName(String frontCarName) {
		this.frontCarName = frontCarName;
	}

	public float getFrontCarDistance() {
		return frontCarDistance;
	}

	public void setFrontCarDistance(float frontCarDistance) {
		this.frontCarDistance = frontCarDistance;
	}

	public float getAcceleration() {
		return acceleration;
	}

	public void setAcceleration(float acc) {
		this.acceleration = acc;
	}

	public Quaternion getRotation() {
		return rotation;
	}

	public void setRotation(Quaternion q) {
		this.rotation = q;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public Vector3f getPosition() {
		return position;
	}

	public void setPosition(Vector3f vector3f) {
		this.position = vector3f;
	}

	public float getBrake() {
		return brake;
	}

	public void setBrake(float brake) {
		this.brake = brake;
	}

	public float getSteerState() {
		return steerState;
	}

	public void setSteerState(float steerState) {
		this.steerState = steerState;
	}

	public float getSteeringAngle() {
		return steeringAngle;
	}

	public void setSteeringAngle(float steeringAngle) {
		this.steeringAngle = steeringAngle;
	}

	public TurnSignalState getSignal() {
		return signal;
	}

	public void setSignal(TurnSignalState signal) {
		this.signal = signal;
	}

}

//End --- KSS