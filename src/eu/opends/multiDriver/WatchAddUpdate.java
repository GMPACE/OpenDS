package eu.opends.multiDriver;

import com.jme3.bullet.control.VehicleControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.shape.Sphere;

import eu.opends.car.CarModelLoader;
import eu.opends.car.SteeringCar;
import eu.opends.main.Simulator;


public class WatchAddUpdate implements Update {
	private Simulator sim;
	private String vehicleID;
	private String modelPath;

	private String driverName;

	public WatchAddUpdate(Simulator sim2, String vehicleID,
			String modelPath, String driverName) {
		this.sim = sim2;
		this.vehicleID = vehicleID;
		this.modelPath = modelPath;
		this.driverName = driverName;
	}

	public void performUpdate() {
		// System.err.println("addVehicle() --> vehicleID: " + vehicleID +
		// ", modelPath: " + modelPath + " , driverName: " + driverName);

		try {

			// sim.setCar()
			// load new car with the CarModelLoader
			/*
			 * CarModelLoader carModel = new CarModelLoader(sim, modelPath, 0);
			 * VehicleControl carControl = carModel.getCarControl(); Node
			 * carNode = carModel.getCarNode(); carNode.setName(vehicleID);
			 * 
			 * // add bounding sphere to a multi-driver car which can be hit by
			 * the // eye gaze ray Sphere sphere = new Sphere(20, 20, 4);
			 * Geometry boundingSphere = new Geometry(vehicleID +
			 * "_boundingSphere", sphere); Material boundingSphereMaterial = new
			 * Material( sim.getAssetManager(),
			 * "Common/MatDefs/Misc/Unshaded.j3md");
			 * boundingSphereMaterial.setColor("Color", ColorRGBA.Green);
			 * boundingSphere.setMaterial(boundingSphereMaterial);
			 * boundingSphere.setCullHint(CullHint.Always);
			 * carNode.attachChild(boundingSphere);
			 * 
			 * sim.getPhysicsSpace().add(carControl);
			 * sim.getSceneNode().attachChild(carNode);
			 * sim.getMultiDriverClient().addRegisteredVehicle(vehicleID);
			 */
			if ("watcher".equals(driverName) == false) {
				sim.getMultiDriverClient().setWatchCar(vehicleID);
				SteeringCar car = sim.getCar();

				// CarModelLoader carModel = new CarModelLoader(sim, modelPath,
				// 0);
				// VehicleControl carControl = carModel.getCarControl();
				// Node carNode = carModel.getCarNode();
				// carNode.setName(vehicleID);

				// add bounding sphere to a multi-driver car which can be hit by
				// the eye gaze ray
				Sphere sphere = new Sphere(0, 0, 0);
				Geometry boundingSphere = new Geometry(vehicleID
						+ "_boundingSphere", sphere);
				Material boundingSphereMaterial = new Material(
						sim.getAssetManager(),
						"Common/MatDefs/Misc/Unshaded.j3md");
				boundingSphereMaterial.setColor("Color", ColorRGBA.Green);
				boundingSphere.setMaterial(boundingSphereMaterial);
				boundingSphere.setCullHint(CullHint.Always);
				car.getCarNode().attachChild(boundingSphere);

				// sim.getPhysicsSpace().add(carControl);
				// sim.getSceneNode().attachChild(carNode);
				sim.getMultiDriverClient().addRegisteredVehicle(vehicleID);
			}

		} catch (Exception e) {
			System.err.println("Could not create vehicle '" + vehicleID + "'!");
		}

	}
}