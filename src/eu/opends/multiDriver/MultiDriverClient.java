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

package eu.opends.multiDriver;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.io.*;

import com.bulletphysics.collision.broadphase.Dbvt.Node;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import eu.opends.analyzer.MyCarLogging;
import eu.opends.car.Car;
import eu.opends.drivingTask.scenario.ScenarioLoader;
import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.environment.XMLParser;
import eu.opends.main.SimulationDefaults;
import eu.opends.main.Simulator;
import eu.opends.main.SimulatorClient;
import eu.opends.multiDriver.MultiCar.STATE;

/**
 * This class represents the connector to the CAN-Interface. Steering, gas, brake and 
 * control instructions from the real car will be forwarded to the simulator; heading,
 * geo coordinates and speed will be sent back to the CAN-Interface in order to display
 * the position and speed on a in-car display. Furthermore trigger collisions can be 
 * sent to the CAN-Interface.
 * 
 * @author Rafael Math
 */


public class MultiDriverClient extends Thread
{
	
	private Simulator sim;
	private ArrayList<Update> updateList = new ArrayList<Update>();
	private Car car;
	private int framerate;
	private boolean stoprequested;
	private boolean errorOccurred;
	private Calendar timeOfLastFire;
	

	/*
	 * Start --- KSS
	 * UDP통신을 할 소켓과 패킷 
	 * */
	private DatagramSocket socket; 
	private DatagramPacket outPacket;
	private DatagramPacket inPacket;
	
	//MulticarList
	private ArrayList<MultiCar> multicarList;
	//End --- KSS
	
	private   String id;
	private ArrayList<String> registeredVehiclesList;
	private InetAddress ip;
	private int port;
	
	/**
	 * Creates a new UDP connection with the multi driver server at the given IP and port
	 * 
	 * @param sim
	 * 			The simulator
	 * 
	 * @param driverName
	 * 			Name of the driver
	 */
	
	public MultiDriverClient(Simulator sim, String driverName)
    {
		
		super();
		
		this.sim = sim;
		this.car = sim.getCar();
		stoprequested = false;
		errorOccurred = false;
		timeOfLastFire = new GregorianCalendar();
		registeredVehiclesList = new ArrayList<String>();
		
		
		ScenarioLoader scenarioLoader = Simulator.getDrivingTask().getScenarioLoader();
		String carModelPath = scenarioLoader.getModelPath();
		
		SettingsLoader settingsLoader = Simulator.getDrivingTask().getSettingsLoader();
	
		 try {
			ip = InetAddress.getByName("127.0.0.1");
		} catch (UnknownHostException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		 
		port = settingsLoader.getSetting(Setting.MultiDriver_port, SimulationDefaults.MultiDriver_port);
		
		framerate = settingsLoader.getSetting(Setting.MultiDriver_updateRate, SimulationDefaults.MultiDriver_updateRate);
		
		try {

			// connect to Server
			
			socket = new DatagramSocket(); // KSS 클라이언트 소켓 생성
			
		 	// send car data (model path and driver name) to multi driver server and flush
			
			String registerString = "<register><modelPath>" + carModelPath + "</modelPath><driverName>"
										+ driverName + "</driverName></register>";
			
			//Multicar register
			multicarList = new ArrayList<MultiCar>(); // 멀티카를 담을 배열리스트

	
			
				
			// FIXME
		
			/*
			 * Start --- KSS
			 * 등록할 문자열을 바이트로 변환 후 패킷에 담아 서버로 메시지를 보낸다. 
			 * */
		 	byte[] msg = registerString.getBytes();
			outPacket = new DatagramPacket(registerString.getBytes(), msg.length,ip,port);
			socket.send(outPacket);
			//End --- KSS

		 	
		} catch (Exception e) {
			//e.printStackTrace();
			System.err.println("No UDP connection possible to multi driver server at " + ip + ":" + port);
			errorOccurred = true;
		}
    } 
	/**
	 * Listens for incoming MD instructions (as XML), such as position and orientation updates, 
	 * which will be forwarded to the XML-parser
	 */
	
	@Override	
	public void run() 
	{
		String shutDownMessage = "Connection to multi driver server closed";
		// when loop is left, connection will be closed
		// loop will be left when requested or error occurred
		while(!stoprequested && !errorOccurred)
		{
			try {
				// delete "NUL" at the end of each line
				String message = readMessage(socket).replace("\0", "");
				//System.out.println("read: "+message);
				// print XML instruction
			
				
				// parse and evaluate XML instruction
				// on "registered" --> call method setID();
				// on "update" --> perform changes
				// on "unregistered" --> call method requestStop()
				XMLParser parser = new XMLParser("<multiDriver>" + message + "</multiDriver>");
				parser.evalMultiDriverInstruction(sim, this);
			} catch (SocketException e) {
								
				// will be thrown if e.g. server was shut down
				shutDownMessage = "Multi driver server: connection closed by server";
				errorOccurred = true;
				
			} catch (StringIndexOutOfBoundsException e) {
				
				// will be thrown if e.g. "stop server" button has been clicked
				shutDownMessage = "Multi driver server: connection closed by server";
				errorOccurred = true;
				
			} catch (SocketTimeoutException e) {
				
			} catch (Exception e) {
				e.printStackTrace();
			}

		}		
		// close UDP connection to multi driver server if connected at all
		try {
			if ((socket != null) )
			{			
				// close socket connection
					
				System.out.println(shutDownMessage);
			}
		} catch (Exception ex) {
			System.err.println("Could not close connection to multi driver server");
		}
	}

	
	/**
	 * Sends car data to the multi driver server, such as position and rotation.
	 */
	public synchronized void sendCarData()
	{
		// break, if no connection established
		if(socket == null || id == null || errorOccurred)
			return;
		
		// generate time stamp
		Calendar currentTime = new GregorianCalendar();		
		
		// if enough time has passed by since last fire, the event will be forwarded
		if(forwardEvent(currentTime))
		{
			/*
			 * Start --- KSS 
			 * 차량운전자의 데이터를 xml형식으로 서버로 보낸다.
			 * */
		
			Vector3f pos = car.getPosition();
			Quaternion rot = car.getRotation();
			float heading = car.getHeadingDegree();
			Quaternion wheelRot = car.getCarControl().getWheel(0).getWheelSpatial().getLocalRotation();
			float array[] = new float[3];
			wheelRot.toAngles(array);
			float wheelSteering = array[1];
			float wheelPositon = array[0];

		 	// send car data (ID, position and rotation) to multi driver server and flush
			String positionString = "<update id=\"" + id + "\">" +
										"<position x=\"" + pos.getX() + "\" y=\"" + pos.getY() + "\" z=\"" + pos.getZ() + "\" />" + 
										"<rotation w=\"" + rot.getW() + "\" x=\"" + rot.getX() + "\" y=\"" + rot.getY() + "\" z=\"" + rot.getZ() + "\"/>" +
										"<heading>" + heading + "</heading>" +  
										"<wheel steering=\"" + wheelSteering + "\" position=\"" + wheelPositon + "\"/>" +
										"<speed speed=\"" + MyCarLogging.getInstance().getSpeed()  + "\"/>" +
										"<rpm rpm=\"" + MyCarLogging.getInstance().getRpm()  + "\"/>" +
										"<velocity x=\"" + MyCarLogging.getInstance().GetVelocity_X() + "\" z=\"" + MyCarLogging.getInstance().GetVelocity_Z() + "\"/>" +
										"<acceleration x=\"" + MyCarLogging.getInstance().GetAcceleration_X() + "\" z=\"" + MyCarLogging.getInstance().GetAcceleration_Z() + "\"/>" +
										"<brake brake=\"" + MyCarLogging.getInstance().getBrake()  + "\"/>" +
										"<distanceFromCenterLine distanceFromCenterLine=\"" + MyCarLogging.getInstance().getDistanceFromCenterLine()  + "\"/>" +
										"<currentLine currentLine=\"" + MyCarLogging.getInstance().getCurrentLine()  + "\"/>" +
										"<LaneeDistance left=\"" + MyCarLogging.getInstance().getLeftLaneDistance() + "\" right=\"" + MyCarLogging.getInstance().getRightLaneDistance() + "\"/>" +
										"<offsetFromLaneCenter offsetFromLaneCenter=\"" + MyCarLogging.getInstance().getOffsetFromLaneCenter()  + "\"/>" +
										"<state state=\"" + MyCarLogging.getInstance().getState()  + "\"/>" +
									"</update>";
	
		 	byte[] msg = positionString.getBytes();
			outPacket = new DatagramPacket(msg, msg.length,ip,port);
			try {
				socket.send(outPacket);
				
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			//End --- KSS
		}
	}
	
	
	public synchronized void setID(String id) 
	{
		this.id = id;		
		
		System.out.println("Connected to multi driver server as '" + id + "'");
	}
	
	
	/**
	 * Requests the connection to close after the current loop
	 * 
	 * @param id
	 * 			Vehicle ID
	 */
	public synchronized void requestStop(String id) 
	{
		if(id.equals(this.id))
			stoprequested = true;
	}
	
	
	public synchronized void close() 
	{
		// break, if no connection established
		if(socket == null || id == null || errorOccurred)	
		{
			stoprequested = true;
			return;
		}

	 	// send unregister string to multi driver server and flush
		String outputString = "<unregister>" + id + "</unregister>";
		

	 	
	 	byte[] msg = outputString.getBytes();
		outPacket = new DatagramPacket(msg, msg.length,ip,port);
		try {
			socket.send(outPacket);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
    
	
	public ArrayList<String> getRegisteredVehicles() 
	{
		return registeredVehiclesList;
	}
	
	
	public synchronized void addRegisteredVehicle(String vehicleID)
	{
		registeredVehiclesList.add(vehicleID);
	}
		
	public synchronized void removeRegisteredVehicle(String vehicleID)
	{
		registeredVehiclesList.remove(vehicleID);
	}
	
	
	/**
	 * Reads an incoming message from the socket connection.
	 * 
	 * @param socket2
	 * 			Socket connection
	 * 
	 * @return
	 * 			Message string of up to 10,000 characters
	 * 
	 * @throws IOException
	 */
	
	private String readMessage(DatagramSocket socket2) throws IOException 
	{
	
		/*
		 * Start --- KSS
		 * 서버로부터 메시지를 받는 메소드
		 * */
		byte[] buffer = new byte[10000];

		inPacket = new DatagramPacket(buffer, buffer.length);
		socket.receive(inPacket);
		
		return new String(inPacket.getData(),0 ,inPacket.getLength());
		//End --- KSS
	}
	
	
	/**
	 * This method checks whether the incoming camera information should 
	 * be sent to the server at the current time complying with the given 
	 * frame rate
	 * 
	 * @param now
	 * 			The current time stamp
	 * 
	 * @return true if enough time has passed by since last fire, false otherwise
	 */
    private boolean forwardEvent(Calendar now)
    {
        // fire an event every x milliseconds
    	int fireInterval = 1000 / framerate;

        // subtract time of last event from current time to get time elapsed since last fire
        long elapsedMillisecs = timeDiff(now,timeOfLastFire);
        
        if (elapsedMillisecs >= fireInterval)
        {
            // update time of last fire
            timeOfLastFire.add(Calendar.MILLISECOND, fireInterval);

            // fire
            return true;
        }
        else
            // do not fire
            return false;
    }
    
    
	/**
	 * This method computes the difference between two given time stamps
	 * 
	 * @param timestamp1
	 * 			First time stamp value to compare
	 * 
	 * @param timestamp2
	 * 			Second time stamp value to compare
	 * 
	 * @return difference between the given time stamps in milliseconds (long)
	 */
    private static long timeDiff(Calendar timestamp1, Calendar timestamp2)
    {
        return Math.abs(timestamp1.getTimeInMillis() - timestamp2.getTimeInMillis());
    }
    String vehicleID = null;
    Node carNode = null;

	public void setCarNode(Node node) {
		carNode = node;
	}
	
	public void setWatchCar(String vehicleID){
		this.vehicleID = vehicleID;
	}
	
	
	public void addVehicle2(String vehicleID, String modelPath, String driverName) {
		
		updateList.add(new WatchAddUpdate(sim, vehicleID, modelPath, driverName));
	}

	public void changeVehicle2(String vehicleID, String positionString,
			String rotationString, String headingString, String wheelString) {

				
		updateList.add(new WatchChangeUpdate( sim, vehicleID, positionString,
				rotationString, headingString, wheelString));
	}
	
	/*
	 * Start --- KSS
	 * 멀티차량이 접속할 때마다 호출되는 메소드
	 * 어댑터에 멀티차량을 아이디와 함께 등록시켜 추가한다.
	 * */
	public void addVehicle(String vehicleID, String modelPath, String driverName)
	{
		MultiCar car = new MultiCar(); //멀티카 객체 생성
		car.setId(vehicleID); // 서버에서 받은 아이디로 설정
		MultiAdapter.getInstance().add(car); // 어댑터에 추가
		
		updateList.add(new AddUpdate(sim, vehicleID, modelPath, driverName));

	}
	//End --- KSS
	
	/*
	 * Start --- KSS
	 * 멀티차량이 움직여 정보가 바뀔때마다 호출되는 메소드
	 * 서버로부터 멀티차량의 정보를 얻어와 파싱한 뒤 어댑터에 변경사항을 저장한다.
	 * */
	public void changeVehicle(String vehicleID, String positionString,String rotationString,String headingString,String wheelString,String speedString,
			String rpmString, String velocityString, String accelerationString,
			String brakeString, String distanceFromCenterLineString, String currentLineString, String LaneeDistanceString, String offsetFromLaneCenterString,
			String stateString)
	{
		
		String[] pos = positionString.split(";");
		String[] rot = rotationString.split(";");
		String[] wheel = wheelString.split(";");
		String[] velocity = velocityString.split(";");
		String[] acceleration = accelerationString.split(";");
		String[] LaneeDistance = LaneeDistanceString.split(";");
		
	
		multicarList = MultiAdapter.getInstance().getMulticarList(); // KSS 어댑터로부터 등록된 멀티카리스트를 받아온다.
		if(multicarList != null){
			for(MultiCar car : multicarList){ // KSS for문을 이용해 하나씩 검사하며 아이디와 비교해 같으면 변경사항을 설정
				if(car.getId().equals(vehicleID)){
					if(pos!=null){
						car.setxPosition(Float.parseFloat(pos[0]));
						car.setyPosition(Float.parseFloat(pos[1]));
						car.setzPosition(Float.parseFloat(pos[2]));	
					}
					if(rot != null){
						car.setwRotation(Float.parseFloat(rot[0]));	
						car.setxRotation(Float.parseFloat(rot[1]));
						car.setyRotation(Float.parseFloat(rot[2]));
						car.setzRotation(Float.parseFloat(rot[3]));	
					}
					
					if(wheel != null){
						car.setSteeringWheel(Float.parseFloat(wheel[0]));
					}
					
					if(speedString != null){
						car.setSpeed(Float.parseFloat(speedString));
					}
					if(rpmString != null){
						car.setRpm(Float.parseFloat(rpmString));
					}
					
					if(velocity != null){
						car.setVelocity_x(Float.parseFloat(velocity[0]));
						car.setVelocity_z(Float.parseFloat(velocity[1]));
					}
					
					if(acceleration != null){
						car.setAcceleration_x(Float.parseFloat(acceleration[0]));
						car.setAcceleration_z(Float.parseFloat(acceleration[1]));
					}
						
					if(brakeString != null){
						car.setBrake(Float.parseFloat(brakeString));
					}
					
					if(distanceFromCenterLineString != null){
						car.setDistanceFromCenterLine(Float.parseFloat(distanceFromCenterLineString));
					}
					
					if(currentLineString != null){
						car.setCurrentLine(Integer.parseInt(currentLineString));
					}
					
					if(LaneeDistance != null){
						car.setLeftLaneeDistance(Float.parseFloat(LaneeDistance[0]));
						car.setRightLaneDistance(Float.parseFloat(LaneeDistance[1]));
					}
					
					if(offsetFromLaneCenterString != null){
						car.setOffsetFromLaneCenter(Float.parseFloat(offsetFromLaneCenterString));
					}
					if(stateString != null){
						if(stateString.equals("NORMAL")){
							car.setState(STATE.NORMAL);
						}
						else if(stateString.equals("DRUNKEN")){
							car.setState(STATE.DRUNKEN);
						}
						else if(stateString.equals("DROWSINESS")){
							car.setState(STATE.DROWSINESS);
						}
						else if(stateString.equals("DISTRACTION")){
							car.setState(STATE.DISTRACTION);
						}
					}
							
				}
			}
		}
			
		updateList.add(new ChangeUpdate(sim, vehicleID, positionString, rotationString, headingString, wheelString));
	}
	
	//End -- KSS
	
	
	public void removeVehicle(String vehicleID)
	{
		updateList.add(new RemoveUpdate(sim, vehicleID));
	}
	
	public void update() 
	{
		
		updateSceneGraph();
		if(SimulatorClient.flag!=2){
	
			sendCarData();		
		}
	}
	
	public String getWatchCar(){
		return this.vehicleID;
	}
	
	public void updateSceneGraph()
	{		
		while(updateList.size() > 0)	
		{
			Update update = updateList.get(0);
			update.performUpdate();
			updateList.remove(0);
		}
	}
}
