package eu.opends.LabViewTcpIp;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

import eu.opends.car.Car;

public class Sender implements Runnable {
	Socket socket;
	DataOutputStream out;
	String sendData = "data";
	int autosteer = 0;
	int autogear = 0;

	public Sender(Socket socket) {
		this.socket = socket;
		try {
			out = new DataOutputStream(socket.getOutputStream());

		} catch (Exception e) {
		}
	}

	public void run() {
		while (true) {	
				setSendData();
			try {
				out.writeUTF(sendData);
				Thread.sleep(100);
			} catch (IOException e) {

			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/*
	 * SWC
	 * ����� ������ �� ������ ��Ʈ���� �����Ѵ�.
	 */
	public void setSendData() {
		sendData = "data" + "autosteer" + Car.sAutoSteer
				+ "autogear" + Car.sAutoGear
				+ "speed" + Car.sSpeed
				+ "rpm" + Car.sRPM
				+ "gear" + 5
				+ "steer" + 0
				+ "odometer" + Car.sOdometer
				+ "eom";
	}
	
}
