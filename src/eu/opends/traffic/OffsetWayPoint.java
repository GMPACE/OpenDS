package eu.opends.traffic;

import com.jme3.math.Vector3f;

import eu.opends.tools.Point;

public class OffsetWayPoint {
	public Point point1;
	public Point point2;
	float offset;
	
	public Vector3f wayPoint;
	
	float slope;
	float powSlopePulsOne;
	float offsetMultipowSlope;
	
	public OffsetWayPoint(){
		wayPoint = new Vector3f();
	}
	
	public OffsetWayPoint(Point point1, Point point2, float offset){
		this.point1 = point1;
		this.point2 = point2;
		
		this.offset = offset;
		
		if(point1.x == point2.x){
			point1.x += 0.2f;
		}
		if(point1.z == point2.z){
			point1.z += 0.2f;
		}
		
		slope = (point2.z - point1.z) / (point2.x - point1.x);
		powSlopePulsOne = (float)(Math.pow(slope, 2) + 1);
		offsetMultipowSlope = (float) (offset * (Math.sqrt(1 / powSlopePulsOne)));
		
		wayPoint = new Vector3f();
		
		if(slope >= 0){
			if(point2.z - point1.z >= 0){
				wayPoint.set(point2.x - offsetMultipowSlope * slope, 0.0f, point2.z + offsetMultipowSlope);
			}else{
				wayPoint.set(point2.x + offsetMultipowSlope * slope, 0.0f, point2.z - offsetMultipowSlope);
			}
		}else{
			if(point2.z - point1.z >= 0){
				wayPoint.set(point2.x + offsetMultipowSlope * slope, 0.0f, point2.z - offsetMultipowSlope);
			}else{
				wayPoint.set(point2.x - offsetMultipowSlope * slope, 0.0f, point2.z + offsetMultipowSlope);
			}
		}
	}
	
	public void setOffsetPoint(float x, float z){
		wayPoint.set(x, 0.0f, z);
	}
	public Vector3f getPoint(){
		return wayPoint;
	}
	
}
