package eu.opends.multiDriver;

import com.jme3.math.Vector3f;
import antlr.collections.impl.Vector;


//Start --- KSS

/*
 * KSS
 * 멀티 차량의 정보들을 가지며 설정 및 변경 가능한 객체 클래스
*/

public class MultiCar {
	
	
	/*
	 * KSS
	 * 멀티카 객체 정보들
	 * */
	private String id; // 아이디
	private float xPosition; // x축 위치
	private float yPosition; // y축 위치
	private float zPosition; // z축  위치
	
	private float wRotation; // w축 회전
	private float xRotation; // x축 회전
	private float yRotation; // y축 회전
	private float zRotation; // z축 회전
	
	private float steeringWheel; // 휠 값
	
	private float speed; // 속력
	private float rpm; // rpm 값
	private float velocity_x; // 속도 x
	private float velocity_z; // 속도 y
	private float acceleration_x; // 가속 x
	private float acceleration_z; // 가속 z
	private float brake; // 브레이크
	private float distanceFromCenterLine; // 중안선으로부터의 거리
	private int currentLine; // 현재 차선
	private float leftLaneeDistance; //왼쪽 차선까지의 거리
	private float rightLaneDistance; // 오른쪽 차선까지의 거리
	private float offsetFromLaneCenter; //차선중앙으로부터의 옵셋
	private STATE state = STATE.NORMAL;  // 현재 상태
	
	

	
	public enum STATE // 보통, 난폭, 졸림, 만취 
	{
		NORMAL, DISTRACTION, DROWSINESS, DRUNKEN
	}
	
	/*
	 * KSS
	 * 멀티카의 객체정보들을 설정하거나 얻을 수 있는 메소드
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
