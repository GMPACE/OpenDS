package eu.opends.LabViewTcpIp;

import java.io.IOException;
import java.net.Socket;
import java.rmi.ConnectException;
import eu.opends.car.SteeringCar;
import eu.opends.input.SimulatorActionListener;
import eu.opends.main.Simulator;

public class TcpIpClient {
	public static TcpIpClient instance;
	public Sender sender;
	public Receiver receiver;
	public Simulator sim;
	public SimulatorActionListener simulatorActionListener;
	public SteeringCar car;
	
	public static TcpIpClient getinstance()
	{
		if(instance == null){
			instance = new TcpIpClient();
		}
		return instance;
	}
	
	/*
	 * SWC
	 * 2���� ���� �����带 ����, ����� �����͸� �ְ�
	 * ��Ʈ��ũ Ŀ�ؼ��� �����Ѵ�.
	 */
	public void setSetting(Simulator sim){
		this.sim = sim;
		this.simulatorActionListener = new SimulatorActionListener(sim);
		this.car = sim.getCar();
		
		String serverIp;
		Socket serverSock;

		try {
			 //serverIp = "192.168.0.7";
			serverIp = "127.0.0.1";

			// ������ �����Ͽ� ������ ��û�Ѵ�.
			System.out.println("������ �������Դϴ�. ����IP : " + serverIp);
			serverSock = new Socket(serverIp, 6341);
			//serverSock = new Socket(serverIp, 7777);
			System.out.println("���� ���� ����");

			sender = new Sender(serverSock);
			receiver = new Receiver(serverSock, sim);

			Thread t1 = new Thread(sender);
			Thread t2 = new Thread(receiver);
						
			t1.start();
			t2.start();	
		} catch (ConnectException ce) {
			//ce.printStackTrace();
		} catch (IOException ie) {
			//ie.printStackTrace();
		} catch (Exception e) {
			//e.printStackTrace();
		}
	}
	
	public SimulatorActionListener getActionListener(){
		return this.simulatorActionListener;
	}
	
	public SteeringCar getCar() {
		return this.car;
	}
}
