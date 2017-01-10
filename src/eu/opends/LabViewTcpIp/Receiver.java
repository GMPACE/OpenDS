package eu.opends.LabViewTcpIp;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.Socket;
import eu.opends.input.SimulatorActionListener;
import eu.opends.main.Simulator;

public class Receiver implements Runnable {
	private Socket socket;
	private DataInputStream in;
	private Sender sender;
	private String receiveString = "1.0000000.0000000.0000000.7000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.0000000.000000";
	private String[] receiveData = new String[19];
	private Simulator sim;
	private int[] arr1 = { 0, 0, 0 };
	private int[] arr2 = { 0, 0, 0 };
	
	private int[] tempStorage = {0,0,0};
	private double tempBrake = 1;
	private double tempTrigger = 0;

	private final double MAX_ACCEL_VALUE = 3463.0;

	public static double rAccel = 0;
	public static double rBrake = 1;
	public static double rSteer = 0;
	public static double rgear = 0;

	public static double rUpperRampStatic = 0;
	public static double rUpperRamp = 0; // ACC + LKAS + LC
	public static double rRamp = 0; // ACC + LKAS
	public static double rRampAuto = 0; // ACC
	public static double rWiperMove = 0;
	public static double rWiperTrigger = 0;
	public static double rWiperOff = 0;
	public static double rWiperAuto = 0;
	public static double rWiperLow = 0;
	public static double rWiperHigh = 0;

	public static double rRightTurn = 0;
	public static double rLeftTurn = 0;

	public static double rBright = 0;
	public static double rLampTrigger = 0;

	Receiver(Socket socket, Simulator sim) {
		this.socket = socket;
		this.sim = sim;

		try {
			in = new DataInputStream(socket.getInputStream());
		} catch (IOException e) {

		}

		for (int i = 0; i < receiveData.length; i++) {
			receiveData[i] = "0.000000";
		}

	}

	public void run() {
		while (in != null) {
			try {
				receiveString = in.readUTF();
				//System.out.println(receiveString);
			} catch (IOException e) {

			}

			devideString();
			parsedDecemal();

			if (sim.getCar() != null) {
				if (sim.getCar().isEngineOn() == true) {
					setAccelator();
					setMode();
					setBrake();
					resume();
				}
			}

		}
	}

	/*
	 * SWC
	 * 랩뷰로 부터 받은 데이터 스트링을 필요한 형식에 맞게 짜른다.
	 */
	private void devideString() {
		int pivot = 0;
		int priorityPivot = 0;
		int count = 0;

		for (int i = 0; i < receiveString.length(); i++) {
			if (receiveString.charAt(pivot) == '.') {
				receiveData[count] = receiveString.substring(priorityPivot, pivot + 6);
				count++;
				priorityPivot = pivot + 7;
				pivot++;
			} else {
				pivot++;
			}
		}
	}

	/*
	 * SWC
	 * 데이터를 필요한 형으로 변환.
	 */
	private void parsedDecemal() {
		Receiver.rAccel = Double.parseDouble(receiveData[1]);
		Receiver.rBrake = Double.parseDouble(receiveData[2]);
		Receiver.rSteer = Double.parseDouble(receiveData[3]);
		Receiver.rgear = Double.parseDouble(receiveData[4]);

		Receiver.rUpperRampStatic = Double.parseDouble(receiveData[5]);
		Receiver.rUpperRamp = Double.parseDouble(receiveData[6]);
		Receiver.rRamp = Double.parseDouble(receiveData[7]);
		Receiver.rRampAuto = Double.parseDouble(receiveData[8]);

		Receiver.rWiperMove = Double.parseDouble(receiveData[9]);
		Receiver.rWiperTrigger = Double.parseDouble(receiveData[10]);
		Receiver.rWiperOff = Double.parseDouble(receiveData[11]);
		Receiver.rWiperAuto = Double.parseDouble(receiveData[12]);
		Receiver.rWiperLow = Double.parseDouble(receiveData[13]);
		Receiver.rWiperHigh = Double.parseDouble(receiveData[14]);

		Receiver.rRightTurn = Double.parseDouble(receiveData[15]);
		Receiver.rLeftTurn = Double.parseDouble(receiveData[16]);
		
		Receiver.rBright = Double.parseDouble(receiveData[17]);
		Receiver.rLampTrigger = Double.parseDouble(receiveData[18]);
		// System.out.println("Accel : " + Receiver.rAccel);

		arr2[0] = (int) rRampAuto;
		arr2[1] = (int) rRamp;
		arr2[2] = (int) rUpperRamp;
	}
	

	/*
	 * 넘어온 엑셀 값에 따라 오픈 디에스 차량의 속도를 증감.
	 */
	private void setAccelator() {
		//System.out.println("accel");
		if(Receiver.rBrake == 1){
			float accelIntencity = (float) ((Receiver.rAccel / MAX_ACCEL_VALUE) * (-1));
			if (Math.abs(accelIntencity) < 1 && Math.abs(accelIntencity) > 0) {
				sim.getCar().setAcceleratorPedalIntensity(accelIntencity);
			}
		}
	}
	
	/*
	 * SWC
	 * 넘어온 데이터 값에 따라 차량의 자율주행 모드를 설정
	 * 000 -> 수동
	 * 100 -> ACC
	 * 010 -> ACC + LKAS
	 * 001 -> ACC + LKAS +LC
	 */
	private void setMode() {
		if (arr1[0] != arr2[0] || arr1[1] != arr2[1] || arr1[2] != arr2[2]) {
			System.out.println("setmode");
			if (arr2[0] == 0 && arr2[1] == 0 && arr2[2] == 0) {
				System.out.println("000");
				if (TcpIpClient.instance.car == null) {
					TcpIpClient.instance.car = TcpIpClient.instance.sim.getCar();
				}
				SimulatorActionListener.accMode = false;
				SimulatorActionListener.laneKeepingMode = false;
				SimulatorActionListener.firstMode = true;
				TcpIpClient.instance.getActionListener().accModeWithCan();
			} else if (arr2[0] == 1 && arr2[1] == 0 && arr2[2] == 0) {
				System.out.println("100");
				SimulatorActionListener.accMode = true;
				SimulatorActionListener.secondMode = true;
				SimulatorActionListener.laneKeepingMode = false;
				TcpIpClient.instance.getActionListener().accModeWithCan();
			} else if (arr2[0] == 0 && arr2[1] == 1 && arr2[2] == 0) {
				System.out.println("010");
				SimulatorActionListener.accMode = true;
				SimulatorActionListener.laneKeepingMode = true;
				SimulatorActionListener.thirdMode = true;
				TcpIpClient.instance.getActionListener().accModeWithCan();
			} else if (arr2[0] == 0 && arr2[1] == 0 && arr2[2] == 1) {
				System.out.println("001");
				SimulatorActionListener.forthMode = true;
				TcpIpClient.instance.getActionListener().accModeWithCan();
			}
			arr1[0] = arr2[0];
			arr1[1] = arr2[1];
			arr1[2] = arr2[2];
		}
	}

	/*
	 * 넘어온 데이터 값에 따라 오픈디에스 차량의 속도를 감속한다.
	 * 1 -> 감속
	 * 0 -> 유지
	 * 브레이크 시 현재의 자율주행 모드를 임시 저장한다.
	 */
	private void setBrake() {
		if (Receiver.rBrake != tempBrake) {
			
			if (Receiver.rBrake == 0.0) {
				System.out.println("brake");
				SimulatorActionListener.speeStorage =sim.getCar().getCurrentSpeedKmh();
				
				sim.getCar().setBrakePedalIntensity(1f);
				sim.getThreeVehiclePlatoonTask().reportBrakeIntensity(1f);
				
				tempStorage[0] = arr1[0];
				tempStorage[1] = arr1[1];
				tempStorage[2] = arr1[2];
				
				
				SimulatorActionListener.accMode = false;
				SimulatorActionListener.laneKeepingMode = false;
				SimulatorActionListener.firstMode = true;
				TcpIpClient.instance.getActionListener().accModeWithCan();
			} else if (Receiver.rBrake == 1.0) {
				sim.getCar().setBrakePedalIntensity(0f);
				sim.getThreeVehiclePlatoonTask().reportBrakeIntensity(0f);
			}
			tempBrake = Receiver.rBrake;
		}
	}
	
	/*
	 * 브레이크 시 임시 저장된 자동제어 모드를 재실행한다.
	 */
	private void resume(){
		if(tempTrigger != Receiver.rLampTrigger){
			System.out.println("resume");
			if(Receiver.rLampTrigger == 1){
				if (tempStorage[0] == 0 && tempStorage[1] == 0 && tempStorage[2] == 0) {
					System.out.println("000");
				} else if (tempStorage[0] == 1 && tempStorage[1] == 0 && tempStorage[2] == 0) {
					System.out.println("100");
					SimulatorActionListener.accMode = true;
					SimulatorActionListener.secondMode = true;
					SimulatorActionListener.laneKeepingMode = false;
					TcpIpClient.instance.getActionListener().accModeWithCan();
				} else if (tempStorage[0] == 0 && tempStorage[1] == 1 && tempStorage[2] == 0) {
					System.out.println("010");
					SimulatorActionListener.accMode = true;
					SimulatorActionListener.laneKeepingMode = true;
					SimulatorActionListener.thirdMode = true;
					TcpIpClient.instance.getActionListener().accModeWithCan();
				} else if (tempStorage[0] == 0 && tempStorage[1] == 0 && tempStorage[2] == 1) {
					System.out.println("001");
					SimulatorActionListener.overTakeMode = true;
					SimulatorActionListener.forthMode = true;
					TcpIpClient.instance.getActionListener().accModeWithCan();
				}
			}
			tempTrigger = Receiver.rLampTrigger;
		}
	}
}