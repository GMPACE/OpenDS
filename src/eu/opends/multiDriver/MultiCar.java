package eu.opends.multiDriver;

import com.jme3.math.Vector3f;
import antlr.collections.impl.Vector;


//Start --- KSS

/*
 * KSS
 * ��Ƽ ������ �������� ������ ���� �� ���� ������ ��ü Ŭ����
*/

public class MultiCar {
	
	
	/*
	 * KSS
	 * ��Ƽī ��ü ������
	 * */
	private String id; // ���̵�
	private float xPosition; // x�� ��ġ
	private float yPosition; // y�� ��ġ
	private float zPosition; // z��  ��ġ
	
	private float wRotation; // w�� ȸ��
	private float xRotation; // x�� ȸ��
	private float yRotation; // y�� ȸ��
	private float zRotation; // z�� ȸ��
	
	private float steeringWheel; // �� ��
	
	private float speed; // �ӷ�
	private float rpm; // rpm ��
	private float velocity_x; // �ӵ� x
	private float velocity_z; // �ӵ� y
	private float acceleration_x; // ���� x
	private float acceleration_z; // ���� z
	private float brake; // �극��ũ
	private float distanceFromCenterLine; // �߾ȼ����κ����� �Ÿ�
	private int currentLine; // ���� ����
	private float leftLaneeDistance; //���� ���������� �Ÿ�
	private float rightLaneDistance; // ������ ���������� �Ÿ�
	private float offsetFromLaneCenter; //�����߾����κ����� �ɼ�
	private STATE state = STATE.NORMAL;  // ���� ����
	
	

	
	public enum STATE // ����, ����, ����, ���� 
	{
		NORMAL, DISTRACTION, DROWSINESS, DRUNKEN
	}
	
	/*
	 * KSS
	 * ��Ƽī�� ��ü�������� �����ϰų� ���� �� �ִ� �޼ҵ�
	 *  */
	public void setState(STATE state){
		this.state = state;
	}
	
	public STATE getState(){
		return state;
	}
	
	public float getSpeed() {
		return speed;
	}
	public void setSpeed(float speed) {
		this.speed = speed;
	}
	public float getRpm() {
		return rpm;
	}
	public void setRpm(float rpm) {
		this.rpm = rpm;
	}
	public float getVelocity_x() {
		return velocity_x;
	}
	public void setVelocity_x(float velocity_x) {
		this.velocity_x = velocity_x;
	}
	public float getVelocity_z() {
		return velocity_z;
	}
	public void setVelocity_z(float velocity_z) {
		this.velocity_z = velocity_z;
	}
	public float getAcceleration_x() {
		return acceleration_x;
	}
	public void setAcceleration_x(float acceleration_x) {
		this.acceleration_x = acceleration_x;
	}
	public float getAcceleration_z() {
		return acceleration_z;
	}
	public void setAcceleration_z(float acceleration_z) {
		this.acceleration_z = acceleration_z;
	}
	public float getBrake() {
		return brake;
	}
	public void setBrake(float brake) {
		this.brake = brake;
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
	public float getLeftLaneeDistance() {
		return leftLaneeDistance;
	}
	public void setLeftLaneeDistance(float leftLaneeDistance) {
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
	
	public float getSteeringWheel() {
		return steeringWheel;
	}
	public void setSteeringWheel(float steeringWheel) {
		this.steeringWheel = steeringWheel;
	}
	
	private Vector3f position;
	

	
	public String getId() {
		return id;
	}
	public void setId(String id) {
		this.id = id;
	}
	public float getxPosition() {
		return xPosition;
	}
	public void setxPosition(float xPosition) {
		this.xPosition = xPosition;
	}
	public float getyPosition() {
		return yPosition;
	}
	public void setyPosition(float yPosition) {
		this.yPosition = yPosition;
	}
	public float getzPosition() {
		return zPosition;
	}
	public void setzPosition(float zPosition) {
		this.zPosition = zPosition;
	}
	public float getwRotation() {
		return wRotation;
	}
	public void setwRotation(float wRotation) {
		this.wRotation = wRotation;
	}
	public float getxRotation() {
		return xRotation;
	}
	public void setxRotation(float xRotation) {
		this.xRotation = xRotation;
	}
	public float getyRotation() {
		return yRotation;
	}
	public void setyRotation(float yRotation) {
		this.yRotation = yRotation;
	}
	public float getzRotation() {
		return zRotation;
	}
	public void setzRotation(float zRotation) {
		this.zRotation = zRotation;
	}
}

//End --- KSS
