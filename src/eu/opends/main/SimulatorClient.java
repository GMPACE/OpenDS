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


package eu.opends.main;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import com.jme3.app.StatsAppState;
import com.jme3.app.state.VideoRecorderAppState;
import com.jme3.input.Joystick;
import com.jme3.math.Vector3f;
import com.jme3.niftygui.NiftyJmeDisplay;
import com.sun.javafx.application.PlatformImpl;

import de.lessvoid.nifty.Nifty;
import eu.opends.analyzer.DrivingTaskLogger;
import eu.opends.analyzer.DataWriter;
import eu.opends.audio.AudioCenter;
import eu.opends.basics.InternalMapProcessing;
import eu.opends.basics.SimulationBasics;
import eu.opends.camera.SimulatorCam;
import eu.opends.cameraFlight.CameraFlight;
import eu.opends.cameraFlight.NotEnoughWaypointsException;
import eu.opends.canbus.CANClient;
import eu.opends.car.ResetPosition;
import eu.opends.car.SteeringCar;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.effects.EffectCenter;
import eu.opends.environment.TrafficLightCenter;
import eu.opends.eyetracker.EyetrackerCenter;
import eu.opends.hmi.HMICenter;
import eu.opends.input.ForceFeedbackJoystickController;
import eu.opends.input.KeyBindingCenter;
import eu.opends.knowledgeBase.KnowledgeBase;
import eu.opends.multiDriver.MultiDriverClient;
import eu.opends.niftyGui.DrivingTaskSelectionGUIController;
import eu.opends.oculusRift.OculusRift;
import eu.opends.reactionCenter.ReactionCenter;
import eu.opends.settingsController.SettingsControllerServer;
import eu.opends.taskDescription.contreTask.SteeringTask;
import eu.opends.taskDescription.tvpTask.MotorwayTask;
import eu.opends.taskDescription.tvpTask.ThreeVehiclePlatoonTask;
import eu.opends.tools.CollisionListener;
import eu.opends.tools.ObjectManipulationCenter;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.SpeedControlCenter;
import eu.opends.tools.Util;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.trigger.TriggerCenter;
import eu.opends.visualization.LightningClient;
import eu.opends.visualization.MoviePlayer;

/**
 * 
 * @author Rafael Math
 */

/*
 * Start --- KSS 
 * 멀티스크린을 위해 시뮬레이터와 같은 원리로 동작하는 클래스를 작성하여 구분한다.
 * */

public class SimulatorClient extends SimulationBasics
{
	private final static Logger logger = Logger.getLogger(Simulator.class);
	public static int flag=1;
    public static void main(String[] args) 
    {    
    	flag=2; // KSS 기존 시뮬레이터와 구분하기 위한 필드값 1 == 시뮬레이터 2 == 시뮬레이터클라이언트
    	try
    	{
    		// copy native files of force feedback joystick driver
    		boolean isWindows = System.getProperty("os.name").toLowerCase().indexOf("win") >= 0;
    		if(isWindows)
    		{
    			boolean is64Bit = System.getProperty("sun.arch.data.model").equalsIgnoreCase("64");
    			if(is64Bit)
    			{
    				copyFile("lib/ffjoystick/native/win64/ffjoystick.dll", "ffjoystick.dll");
    				copyFile("lib/ffjoystick/native/win64/SDL.dll", "SDL.dll");
    			}
    			
    			else
    			{
    				copyFile("lib/ffjoystick/native/win32/ffjoystick.dll", "ffjoystick.dll");
    				copyFile("lib/ffjoystick/native/win32/SDL.dll", "SDL.dll");
    			}
    		}
    	
    		
    		// load logger configuration file
    		PropertyConfigurator.configure("assets/JasperReports/log4j/log4j.properties");
    		
    	
    		// only show severe jme3-logs
    		java.util.logging.Logger.getLogger("").setLevel(java.util.logging.Level.SEVERE);
    		
    		//PlatformImpl.startup(() -> {});
    		
	    	Simulator sim = new Simulator();
    		
	    	StartPropertiesReader startPropertiesReader = new StartPropertiesReader();

			sim.setSettings(startPropertiesReader.getSettings(sim));

			// show/hide settings screen
			sim.setShowSettings(startPropertiesReader.showSettingsScreen());
			
			if(!startPropertiesReader.getDrivingTaskPath().isEmpty() &&
					DrivingTask.isValidDrivingTask(new File(startPropertiesReader.getDrivingTaskPath())))
    		{
    			SimulationDefaults.drivingTaskFileName = startPropertiesReader.getDrivingTaskPath();
    			sim.drivingTaskGiven = true;
    		}
			
			if(!startPropertiesReader.getDriverName().isEmpty())
				SimulationDefaults.driverName = startPropertiesReader.getDriverName();
			
			
	    	if(args.length >= 1)
	    	{
	    		if(DrivingTask.isValidDrivingTask(new File(args[0])))
	    		{
	    			SimulationDefaults.drivingTaskFileName = args[0];
	    			sim.drivingTaskGiven = true;
	    		}
	    	}
	
	    	if(args.length >= 2)
	    	{
	    		SimulationDefaults.driverName = args[1];
	    	}
			
	    	sim.setPauseOnLostFocus(false);
	    
			sim.start();
    	}
    	catch(Exception e1)
    	{
    		logger.fatal("Could not run main method:", e1);
    	}
    	
    }
    private static void copyFile(String sourceString, String targetString) 
	{
		try {
			
			Path source = Paths.get(sourceString);
			Path target = Paths.get(targetString);
			
			if(Files.exists(source, LinkOption.NOFOLLOW_LINKS))
				Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
			else
				System.err.println("ERROR: '" + sourceString + "' does not exist.");
		
		} catch (IOException e) {

			e.printStackTrace();
		}
	}
 
}

//End -- KSS