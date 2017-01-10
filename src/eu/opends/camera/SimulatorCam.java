


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

package eu.opends.camera;

import java.text.DecimalFormat;
import java.util.ArrayList;

import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.font.BitmapFont.Align;
import com.jme3.material.*;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.control.CameraControl;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.*;

import eu.opends.analyzer.MyCarLogging;
import eu.opends.camera.CameraFactory.CameraMode;
import eu.opends.car.Car;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.main.Simulator;
import eu.opends.multiDriver.MultiAdapter;
import eu.opends.multiDriver.MultiCar;
import eu.opends.multiDriver.MultiCar.STATE;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.Util;
import eu.opends.tools.Hud;

import com.jme3.app.SimpleApplication;
import com.jme3.light.DirectionalLight;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.texture.Texture;
import com.jme3.ui.Picture;
import com.jme3.util.TangentBinormalGenerator;
/**
 * 
 * @author Rafael Math
 */

public class SimulatorCam extends CameraFactory 
{	
	private Car car;
	private Node carNode;
	private Geometry geoCone;
	private Geometry trafficSphere;
	private Simulator sim;
	public float horizontalAngle;
	public float verticalAngle;
	private int numOfMulticar; //multi car의 댓수_SHS
	private ArrayList<MultiCar> listOfCarPos;  //multi 자동차 위치_SHS
	private ArrayList<Geometry> listOfShape = new ArrayList<Geometry>();  //multi 자동차 모양_SHS
	private ArrayList<Geometry> listOfTexture = new ArrayList<Geometry>();  //multi Texture_SHS
	
	//Top View내 깜박이는데 필요한 변수들
	private boolean blinking = true;
	private float secondPassed = 0f;
	private float blinkOnTime = 0.5f;  //켜있는 시간
	private float blinkOffTime = 0.5f;  //꺼지는 시간
	private Picture degreeOfRisk;  //위험도를 나타내는 그림
	private static Node riskIndicator = new Node("riskIndicator");  
	
	private MyCarLogging myCarLogging;  //내 차 정보

	public SimulatorCam(Simulator sim, Car car) 
	{	    
		this.car = car;
		carNode = car.getCarNode();
		this.sim = sim;
		initCamera(sim, carNode);

		String cameraModeString = settingsLoader.getSetting(Setting.General_cameraMode, "ego").toUpperCase();
		if(cameraModeString == null || cameraModeString.isEmpty())
			cameraModeString = "EGO";
		CameraMode cameraMode = CameraMode.valueOf(cameraModeString);
		setCamMode(cameraMode);
		initMapMarker();

		SetListOfCarPos();
		SetNumOfMultiCar();
		SetMultiCarInitMapMarker();
	}


	private void initMapMarker()
	{
		Cylinder cone = new Cylinder(4, 4, 2f, 0.1f, 6f, true, false);
		cone.setLineWidth(4f);
		geoCone = new Geometry("TopViewMarker", cone);  //삼각형 모양으로 도형 생성_SHS

		Material coneMaterial = new Material(sim.getAssetManager(),"Common/MatDefs/Misc/Unshaded.j3md");
		coneMaterial.setColor("Color", ColorRGBA.White);  //삼각형 도형 흰색 설정_SHS

		geoCone.setMaterial(coneMaterial);
		geoCone.setCullHint(CullHint.Always);  //랜더링을 안하도록 설정_SHS

		sim.getRootNode().attachChild(geoCone);		//랜더링 하기 위해 simulator의 root node에 연결
	}

	public void setCamMode(CameraMode mode)
	{
		switch (mode)
		{
		case EGO:
			System.out.println("EGO");
			camMode = CameraMode.EGO;
			sim.getRootNode().detachChild(mainCameraNode);
			carNode.attachChild(mainCameraNode);
			chaseCam.setEnabled(false);
			setCarVisible(false);// FIXME --> false  // used for oculus rift internal car environment
			((CameraControl) frontCameraNode.getChild("CamNode1").getControl(0)).setEnabled(true);
			frontCameraNode.setLocalTranslation(car.getCarModel().getEgoCamPos());
			frontCameraNode.setLocalRotation(new Quaternion().fromAngles(0, 0, 0));
			break;

		case CHASE:
			System.out.println("CHASE");
			camMode = CameraMode.CHASE;
			sim.getRootNode().detachChild(mainCameraNode);
			carNode.attachChild(mainCameraNode);
			chaseCam.setEnabled(true);
			chaseCam.setDragToRotate(false);
			setCarVisible(true);
			((CameraControl) frontCameraNode.getChild("CamNode1").getControl(0)).setEnabled(false);
			break;

		case TOP:
			System.out.println("TOP");
			camMode = CameraMode.TOP;
			sim.getRootNode().detachChild(mainCameraNode);
			carNode.attachChild(mainCameraNode);
			chaseCam.setEnabled(false);
			setCarVisible(true);
			((CameraControl) frontCameraNode.getChild("CamNode1").getControl(0)).setEnabled(true);

			Vector3f camPos3 = new Vector3f(car.getCarModel().getStaticBackCamPos());
			frontCameraNode.setLocalTranslation(camPos3.x+3f,camPos3.y-2f,camPos3.z);
			frontCameraNode.setLocalRotation(new Quaternion().fromAngles(0, 330, 0));
			break;

		case OUTSIDE:
			System.out.println("OUTSIDE");
			camMode = CameraMode.OUTSIDE;
			sim.getRootNode().detachChild(mainCameraNode);
			carNode.attachChild(mainCameraNode);
			chaseCam.setEnabled(false);
			setCarVisible(true);
			((CameraControl) frontCameraNode.getChild("CamNode1").getControl(0)).setEnabled(true);

			Vector3f camPos2 = new Vector3f(car.getCarModel().getStaticBackCamPos());
			frontCameraNode.setLocalTranslation(camPos2.x-3f,camPos2.y-2f,camPos2.z);
			frontCameraNode.setLocalRotation(new Quaternion().fromAngles(0, 330, 0));
			break;

		case STATIC_BACK:
			System.out.println("STATIC_BACK");
			camMode = CameraMode.STATIC_BACK;
			sim.getRootNode().detachChild(mainCameraNode);
			carNode.attachChild(mainCameraNode);
			chaseCam.setEnabled(false);
			setCarVisible(true);
			((CameraControl) frontCameraNode.getChild("CamNode1").getControl(0)).setEnabled(true);

			Vector3f camPos = new Vector3f(car.getCarModel().getStaticBackCamPos());
			frontCameraNode.setLocalTranslation(camPos.x-0.1f,camPos.y-2f,camPos.z-6f);
			frontCameraNode.setLocalRotation(new Quaternion().fromAngles(0, 330, 0));
			break;

		case OFF:
			camMode = CameraMode.OFF;
			chaseCam.setEnabled(false);
			setCarVisible(false);
			break;
		}
	}

	public void changeCamera() 
	{
		// STATIC_BACK --> EGO (--> CHASE, only if 1 screen) --> TOP --> OUTSIDE --> STATIC_BACK --> ...
		switch (camMode)
		{
		case STATIC_BACK: setCamMode(CameraMode.EGO); break;
		case EGO: setCamMode(CameraMode.TOP); break;
		case CHASE: setCamMode(CameraMode.TOP); break;
		case TOP: setCamMode(CameraMode.OUTSIDE); break;
		case OUTSIDE: setCamMode(CameraMode.STATIC_BACK); break;
		default: break;
		}
	}


	public void updateCamera()
	{
		/*
		 * Start -- KSS 
		 * 카메라 모드 설정
		 * */
		if(camMode == CameraMode.EGO)
		{	

			if(mirrorMode == MirrorMode.RIGHT)
			{
				backViewPort.setEnabled(false);
				leftBackViewPort.setEnabled(false);
				rightBackViewPort.setEnabled(true);
				backMirrorFrame.setCullHint(CullHint.Always);
				leftMirrorFrame.setCullHint(CullHint.Always);
				rightMirrorFrame.setCullHint(CullHint.Dynamic);
			}
			else if(mirrorMode == MirrorMode.BACK_ONLY)
			{
				backViewPort.setEnabled(true);
				leftBackViewPort.setEnabled(false);
				rightBackViewPort.setEnabled(false);
				backMirrorFrame.setCullHint(CullHint.Dynamic);
				leftMirrorFrame.setCullHint(CullHint.Always);
				rightMirrorFrame.setCullHint(CullHint.Always);
			}
			else if(mirrorMode == MirrorMode.LEFT)
			{
				backViewPort.setEnabled(false);
				leftBackViewPort.setEnabled(true);
				rightBackViewPort.setEnabled(false);
				backMirrorFrame.setCullHint(CullHint.Always);
				leftMirrorFrame.setCullHint(CullHint.Dynamic);
				rightMirrorFrame.setCullHint(CullHint.Always);
			}
			else
			{
				backViewPort.setEnabled(false);
				leftBackViewPort.setEnabled(false);
				rightBackViewPort.setEnabled(false);
				backMirrorFrame.setCullHint(CullHint.Always);
				leftMirrorFrame.setCullHint(CullHint.Always);
				rightMirrorFrame.setCullHint(CullHint.Always);
			}			
		}
		//End --- KSS
		else
		{
			backViewPort.setEnabled(false);
			leftBackViewPort.setEnabled(false);
			rightBackViewPort.setEnabled(false);

			backMirrorFrame.setCullHint(CullHint.Always);
			leftMirrorFrame.setCullHint(CullHint.Always);
			rightMirrorFrame.setCullHint(CullHint.Always);
		}

		if(camMode == CameraMode.TOP)
		{
			// camera detached from car node --> update position and rotation separately
			Vector3f targetPosition = carNode.localToWorld(new Vector3f(0, 0, 0), null);
			Vector3f camPos = new Vector3f(targetPosition.x, targetPosition.y + 5, targetPosition.z);
			frontCameraNode.setLocalTranslation(camPos);

			float upDirection = 0;
			if(isCarPointingUp)
			{
				float[] angles = new float[3];
				carNode.getLocalRotation().toAngles(angles);
				upDirection = angles[1];
			}
			frontCameraNode.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, upDirection, 0));
		}

		if(camMode == CameraMode.OUTSIDE)
		{
			// camera detached from car node --> update position and rotation separately
			frontCameraNode.setLocalTranslation(outsideCamPos);

			Vector3f carPos = carNode.getWorldTranslation();

			Vector3f direction = carPos.subtract(outsideCamPos);
			direction.normalizeLocal();
			direction.negateLocal();

			Vector3f up = new Vector3f(0, 1, 0);

			Vector3f left = up.cross(direction);
			left.normalizeLocal();

			if (left.equals(Vector3f.ZERO)) {
				if (direction.x != 0) {
					left.set(direction.y, -direction.x, 0f);
				} else {
					left.set(0f, direction.z, -direction.y);
				}
			}
			up.set(direction).crossLocal(left).normalizeLocal();
			frontCameraNode.setLocalRotation(new Quaternion().fromAxes(left, up, direction));

		}

		// additional top view window ("map")		
		if(topViewEnabled)  //Topview 모드 일경우
		{			
			topViewPort.setEnabled(true);
			topViewFrame.setCullHint(CullHint.Dynamic);
			geoCone.setCullHint(CullHint.Dynamic);
			//TODO

			MultiCarUpdate();

			// camera detached from car node --> update position and rotation separately
			float upDirection = 0;
			float addLeft = 0;
			float addRight = 0;
			if(isCarPointingUp)
			{
				float[] angles = new float[3];
				carNode.getLocalRotation().toAngles(angles);
				upDirection = angles[1] + FastMath.PI;

				// allow to place car in lower part of map (instead of center)
				addLeft = topViewcarOffset * FastMath.sin(upDirection);
				addRight = topViewcarOffset * FastMath.cos(upDirection);
			}
			Quaternion camRot = new Quaternion().fromAngles(FastMath.HALF_PI, upDirection, 0);
			topViewCamNode.setLocalRotation(camRot);
			topViewCamNode.detachChildNamed("TopViewMarker");
			topViewCamNode.attachChild(riskIndicator);

			Vector3f targetPosition = carNode.localToWorld(new Vector3f(0, 0, 0), null);
			float left = targetPosition.x + addLeft;
			float up = targetPosition.y + topViewVerticalDistance;
			float ahead = targetPosition.z + addRight;
			Vector3f camPos = new Vector3f(left, up, ahead);
			topViewCamNode.setLocalTranslation(camPos);

			// set cone position
			geoCone.setLocalTranslation(targetPosition.x, targetPosition.y + 3, targetPosition.z);  //내 차량의 3D 좌표를 이용하여 y축값 +3 하여 위치 옮김
			geoCone.setLocalRotation(carNode.getLocalRotation());  //내 차량의 방향과 일치하게 회전 시킴

			//위험정도를 나타내는 그림 -> Top view화면의 오른쪽 하단에 있는 표
			degreeOfRisk = new Picture("degreeOfRisk");
			degreeOfRisk.setImage(sim.getAssetManager(), "Textures/Gauges/HUD/riskDegree.png", true);
			degreeOfRisk.setWidth(300);
			degreeOfRisk.setHeight(450);
			degreeOfRisk.setLocalTranslation(0, 0, 0);
			degreeOfRisk.setCullHint(CullHint.Inherit);
			riskIndicator.attachChild(degreeOfRisk);
			Node guiNode = sim.getGuiNode();
			guiNode.attachChild(riskIndicator);

		}
		else
		{
			topViewPort.setEnabled(false);
			topViewFrame.setCullHint(CullHint.Always);
			geoCone.setCullHint(CullHint.Always);
		}
	}

	public void setCarVisible(boolean setVisible) 
	{
		if(setVisible)
		{
			// show everything except sub-geometries of node interior
			Node interior = Util.findNode(carNode, "interior");
			for(Geometry g : Util.getAllGeometries(carNode))
				if(g.hasAncestor(interior))
					g.setCullHint(CullHint.Always);  //interior
				else
					g.setCullHint(CullHint.Dynamic); //rest (or interior == null)

		}
		else
		{
			// cull everything except sub-geometries of node interior
			Node interior = Util.findNode(carNode, "interior");
			for(Geometry g : Util.getAllGeometries(carNode))
				if(g.hasAncestor(interior))
					g.setCullHint(CullHint.Dynamic); //interior
				else
					g.setCullHint(CullHint.Always);  //rest (or interior == null)

		}

		//PanelCenter.showHood(!setVisible);
		if(Hud.flag>=1)  //Hud 켜있는 경우_SHS
			Hud.showHood(!setVisible);
	}

	public void SetListOfCarPos(ArrayList<MultiCar> list)  //전달인자로 multicar의 정보를 받아와 각각 차량에 대해 도형을 생성함_SHS
	{
		this.listOfCarPos = list;
		//Multi Driver
		for(int i=0;i<listOfCarPos.size();i++)
		{
			Sphere trafficShape = new Sphere(10,10,3);  //도형은 원으로 생성_SHS
			trafficShape.setLineWidth(4f);
			listOfShape.add(new Geometry("TrafficMarker", trafficShape));

			Material material = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
			material.setColor("Color",  ColorRGBA.Red);
			listOfShape.get(i).setMaterial(material);
			listOfShape.get(i).setCullHint(CullHint.Always);
			sim.getRootNode().attachChild(listOfShape.get(i));
		}
	}

	public void SetListOfCarPos()  //multiadapter에 있는 multicar list의 주소값 저장 _SHS
	{
		listOfCarPos = MultiAdapter.getInstance().getMulticarList();
	}

	public void SetNumOfMultiCar()  //multicar의 차량 댓수 설정
	{
		if(listOfCarPos == null)
		{
			numOfMulticar = 0;
			return;
		}

		numOfMulticar =  MultiAdapter.getInstance().getMulticarList().size();
	}


	public void SetMultiCarInitMapMarker()  //초기에 multicar의 정보를 나타내기 위한 도형을 초기화 시키는 함수_SHS
	{
		for(int i=0;i<numOfMulticar;i++)
		{
			Sphere sphere = new Sphere(10,10,3);
			sphere.setLineWidth(4f);
			trafficSphere = new Geometry("TrafficMarker" + i , sphere);

			Material sphereMaterial = new Material(sim.getAssetManager(),"Common/MatDefs/Misc/Unshaded.j3md");
			sphereMaterial.setColor("Color", new ColorRGBA(1,0,0,0));
			
			//sphereMaterial.setColor("Color", new ColorRGBA(1,0,0,0.25f));
			//sphereMaterial.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);  //Use AlphaBlending_SHS
			//sphereMaterial.getAdditionalRenderState().setDepthWrite(false);  //Use depth buffer_SHS
			//sphereMaterial.getAdditionalRenderState().setAlphaTest(true);
			
			trafficSphere.setMaterial(sphereMaterial);
			trafficSphere.setCullHint(CullHint.Always);

			sim.getRootNode().attachChild(trafficSphere);  //simulator에서 랜더링 하는 node에 attach_SHS
			listOfShape.add(trafficSphere);
			
			//Texture
			Box box = new Box( 2.5f,2.5f,2.5f);  //texture mapping 을 하기 위한 도형 box(cube)생성_SHS
			Geometry geometry = new Geometry("TrafficTexture" + i, box);
			geometry.setLocalTranslation(new Vector3f(-3f,1.1f,0f));  //초기 위치 설정_SHS
			
			Material cubeMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
			Texture cubeTexture = sim.getAssetManager().loadTexture("Textures/Navigation/crossing_straight.png");
			
			cubeMaterial.setTexture("ColorMap", cubeTexture);
			geometry.setMaterial(cubeMaterial);
			geometry.setCullHint(CullHint.Always);
			
			sim.getRootNode().attachChild(geometry);
			listOfTexture.add(geometry);
		}
	}


	public void MultiCarUpdate()  //multicar list의 정보를 받아와 topview에 frame 단위로 각각의 도형을 위치 및 특징을 바꾸는 함수_SHS
	{		
		if(listOfCarPos == null)
			return;

		if(numOfMulticar != listOfCarPos.size())  // multicar  댓수 변화 있으면
		{
			SetNumOfMultiCar();
			SetMultiCarInitMapMarker();
		}

		secondPassed += 0.1;
		float maxTime = blinking ? blinkOnTime : blinkOffTime;
		
		if(secondPassed > maxTime)  //깜박임
		{
			secondPassed = 0;
			for(int i=0;i<numOfMulticar-1;i++)
			{
				if(blinking)
				{
					sim.getRootNode().getChild("TrafficMarker" + i).setCullHint(CullHint.Inherit);  //켜짐_SHS
					blinking = false;
				}
				else
				{
					sim.getRootNode().getChild("TrafficMarker" + i).setCullHint(CullHint.Always); //꺼짐_SHS
					blinking = true;
				}

				
				//위치 설정
				float trafficPosx =listOfCarPos.get(i).getxPosition();
				float trafficPosy =listOfCarPos.get(i).getyPosition();
				float trafficPosz =listOfCarPos.get(i).getzPosition();
				listOfShape.get(i).setLocalTranslation(trafficPosx, trafficPosy, trafficPosz);
			}
		}
		//깜박깜박 -> always, inherit 반복으로_SHS
		

		//texture_SHS
		for(int i=0; i<numOfMulticar -1;i++)
		{
			if(listOfCarPos.get(i).getState() == STATE.DROWSINESS)  //Drowsiness _SHS
			{
				Material cubeMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
				Texture cubeTexture = sim.getAssetManager().loadTexture("Textures/State/drowse.png");
				cubeMaterial.setTexture("ColorMap", cubeTexture);
				
				sim.getRootNode().getChild("TrafficTexture" + i).setMaterial(cubeMaterial);
				sim.getRootNode().getChild("TrafficTexture" + i).setCullHint(CullHint.Inherit);
			}

			else if(listOfCarPos.get(i).getState() == STATE.DISTRACTION)  //distraction _SHS
			{
				Material cubeMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
				Texture cubeTexture = sim.getAssetManager().loadTexture("Textures/State/cellphone.png");
				cubeMaterial.setTexture("ColorMap", cubeTexture);

				sim.getRootNode().getChild("TrafficTexture" + i).setMaterial(cubeMaterial);
				sim.getRootNode().getChild("TrafficTexture" + i).setCullHint(CullHint.Inherit);
			}
			
			else if(listOfCarPos.get(i).getState() == STATE.DRUNKEN)  // Druken_SHS
			{
				Material cubeMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
				Texture cubeTexture = sim.getAssetManager().loadTexture("Textures/State/drunken.png");
				cubeMaterial.setTexture("ColorMap", cubeTexture);

				sim.getRootNode().getChild("TrafficTexture" + i).setMaterial(cubeMaterial);
				sim.getRootNode().getChild("TrafficTexture" + i).setCullHint(CullHint.Inherit);
			}
			
			else
			{
				sim.getRootNode().getChild("TrafficTexture" + i).setCullHint(CullHint.Always);
			}
			
			
			//texture의 위치 이동_SHS
			float trafficPosx =listOfCarPos.get(i).getxPosition();
			float trafficPosy =listOfCarPos.get(i).getyPosition();
			float trafficPosz =listOfCarPos.get(i).getzPosition();
			listOfTexture.get(i).setLocalTranslation(trafficPosx, trafficPosy+3, trafficPosz);
		}
	}
	
}
