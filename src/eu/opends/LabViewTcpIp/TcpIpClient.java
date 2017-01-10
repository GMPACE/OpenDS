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
	 * 2개의 별도 쓰레드를 생성, 랩뷰와 데이터를 주고
	 * 네트워크 커넥션을 생성한다.
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

			// 소켓을 생성하여 연결을 요청한다.
			System.out.println("서버에 연결중입니다. 서버IP : " + serverIp);
			serverSock = new Socket(serverIp, 6341);
			//serverSock = new Socket(serverIp, 7777);
			System.out.println("서버 연결 성공");

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
