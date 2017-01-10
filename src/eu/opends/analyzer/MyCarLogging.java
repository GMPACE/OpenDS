package eu.opends.analyzer;

import java.util.ArrayList;
import java.util.Date;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;


import eu.opends.car.LightTexturesContainer.TurnSignalState;


/*
 * Start --- KSS
 *  ������������ ������ ������ �ִ� ��ü Ŭ����
 *  */

public class MyCarLogging {

	/*
	 * KSS
	 * ��ü������ ������ �̷������ �ʰ� �̱��������� �����Ѵ�.
	 * �ϳ��� ��ü������ ���� ���� Ŭ�������� ���ٰ����ϰ� �Ѵ�.
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
	 * ���������� ��ü ������
	 * */
	
	private String name; // ����̹��̸�
	private Vector3f position; // ���� ��ġ
	private float brake; // �극��ũ �з�
	private float speed; // ���� �ӵ�
	private float steerState; // steer ����
	private float steeringAngle; // steer����
	private TurnSignalState signal; // �������õ�
	private Quaternion rotation; // ���� ����
	private float rpm; // rpm
	private float acceleration; // ���� ���ӵ�
	private String frontCarName; // ���� ���� ���̵�
	private float frontCarDistance; // ���� �������� ���Ÿ�
	private float distanceFromCenterLine;
	private int currentLine = -1; // ���� ����
	private float leftLaneeDistance; // ���� ���������� �Ÿ�
	private float rightLaneDistance; // ������ ���������� �Ÿ�
	private float offsetFromLaneCenter;
	
	private Vector3f prePosition = null;  //���� ��ġ
	private long preDateForVel;
	private long preDateForAcc;
	private long preDateForAcceleration;
	private float preSpeed;
	private float velocity_x;  //x�� �ӵ�
	private float velocity_z;  //z�� �ӵ�
	private float preVelocity_x;  //x�� ���� �ӵ�
	private float preVelocity_z;  //z�� ���� �ӵ�
	private float acceleration_x;  //x�� ���ӵ�
	private float acceleration_z;  //z�� ���ӵ�
	
	
	private float headTraker_pitch; // �ص�Ʈ��ŷ pitch 
	private float headTraker_yaw; // �ص�Ʈ��ŷ yaw
	private float headTraker_roll; // �ص�Ʈ��ŷ roll
	private float headTraker_x;	// �ص�Ʈ��ŷ x ��ǥ
	private float headTraker_y; // �ص�Ʈ��ŷ y ��ǥ
	private float headTraker_z; // �ص�Ʈ��ŷ z ��ǥ
	
	private STATE state = STATE.NORMAL;  // ���� ������ ����
	
	
	public enum STATE // ����, ����, ����, ���� 
	{
		NORMAL, DISTRACTION, DROWSINESS, DRUNKEN
	}
	
	
	/*
	 * KSS
	 * ������ ������ ��ü�������� �����ϰų� ���� �� �ִ� �޼ҵ�
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