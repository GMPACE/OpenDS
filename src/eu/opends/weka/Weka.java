
package eu.opends.weka;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import com.jme3.math.Vector3f;

import eu.opends.analyzer.MyCarLogging;
import eu.opends.analyzer.MyCarLogging.STATE;
import eu.opends.environment.XMLParser;
import eu.opends.multiDriver.MultiAdapter;
import eu.opends.multiDriver.MultiCar;
import weka.classifiers.trees.J48;
import weka.classifiers.trees.RandomForest;
import weka.core.Attribute;
import weka.core.FastVector;
import weka.core.Instance;
import weka.core.Instances;
import weka.classifiers.Classifier;
import weka.classifiers.Evaluation;


//---Start KSS

/*
 * KSS
 * ���赥���͸� �ҷ��ͼ� RandomForest 
 * Ʈ���� �̿��Ͽ� �н��� �� ������������ 
 * ���¸� Ȯ���Ѵ�.
 * */


public class Weka extends Thread{
	
	BufferedReader normalReader;

	Instances normalSet;

	
	RandomForest normalTree;

		
	Evaluation normalEval;

	double[][]state_matrix;
	
	
	String folderName;
	
	
	/*
	 * KSS
	 * <������>
	 * Weka ��ü�� �����ϸ� �ٷ� �����带 �����Ͽ� �����Ѵ�.
	 * */
	
	public Weka(String outputFolder){
		
		folderName = outputFolder;
		//start();
		
	}
	

	/*
	 * KSS
	 * <evaluate �޼ҵ�>
	 * EvaluateState��� ���ο� �����带 �����Ͽ� ������ ���¸� �ľ��Ѵ�.
	 * */
	
	public void evaluate(){ //������ ���� �ľ� �޼ҵ�
		
		Runnable runnable = new EvaluateState();
		Thread thread = new Thread(runnable);
		thread.start(); //���ο� ������ �����ؼ� ����
		
	}
	

	/*
	 * KSS
	 * <Weka ������>
	 * �н���ų �� ������ �ҷ��ͼ� �ν��Ͻ����� �����ϰ� RandomForest Ʈ���� �̿��Ͽ� �н���Ų��.
	 * */
	
	@Override
	public void run() 
	{
		
		
		try {
			// ���°� �ʱ�ȭ
			state_matrix = new double[4][4];
			for(int i =0; i<4; i++){
				for(int j =0; j<4; j++){
					state_matrix[i][j]=0f;
				}
			}
			
			// Ʈ���̴� �ν��ͽ��� �ҷ�����
			
			normalReader = new BufferedReader(new FileReader("model4.arff")); //Ʈ���̴� �� �� ���� �ҷ�����

			
			normalSet = new Instances(normalReader); // �ҷ��� ������ �̿��Ͽ� Ʈ���̴��� �ν��Ͻ��� ����

		
			normalReader.close(); // ���ϸ��� �ݱ�
			
			
			normalSet.setClassIndex(20); //20��° ���ڸ� Ŭ����ȭ

			//�ɼ�
			String[] options = new String[1];
			options[0] = "- U";
			

			
			// Ʈ�� ����
			normalTree = new RandomForest();

			
			//�з�
			normalTree.buildClassifier(normalSet);

								
			
			//����
			normalEval = new Evaluation(normalSet);

			
			
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		evaluate(); //������ ���� �ľ� �޼ҵ� ����
		
	}
	
	/*
	 * KSS
	 * <EvaluateState ������>
	 * 1�ʸ��� �������� �����͸� �̿��Ͽ� �׽�Ʈ�� �ν��Ͻ����� �����Ͽ� �������� ���¸� �ľ��Ѵ�.
	*/
	
	 class EvaluateState implements Runnable {

		
		@Override
		public void run() {
			while(true)
			{
			// TODO Auto-generated method stub

			
			try {
				
				// ������ �Ӽ� ����
				Attribute CarPosition_x = new Attribute("CarPosition_x");
				Attribute CarPosition_y = new Attribute("CarPosition_y");
				Attribute CarPosition_z = new Attribute("CarPosition_z");
				Attribute CarRotation_x = new Attribute("CarRotation_x");
				Attribute CarRotation_y = new Attribute("CarRotation_y");
				Attribute CarRotation_z = new Attribute("CarRotation_z");
				Attribute CarRotation_w = new Attribute("CarRotation_w");
				Attribute CurrentSpeed = new Attribute("CurrentSpeed");
				Attribute Engine_RPM = new Attribute("Engine_RPM");
				Attribute Velocity_x = new Attribute("Velocity_x");
				Attribute Velocity_z = new Attribute("Velocity_z");
				Attribute Acceleration_x = new Attribute("Acceleration_x");
				Attribute Acceleration_z = new Attribute("Acceleration_z");
				Attribute SteeringAngle = new Attribute("SteeringAngle");
				Attribute Brake = new Attribute("Brake");
				Attribute Distance_from_center = new Attribute("Distance_from_center");
				Attribute Lane_Number = new Attribute("Lane_Number");
				Attribute Distance_to_left_lane_line = new Attribute("Distance_to_left_lane_line");
				Attribute Distance_to_right_lane_line = new Attribute("Distance_to_right_lane_line");
				Attribute Offset_from_lane_center = new Attribute("Offset_from_lane_center");
				
				//����� �Ӽ�����, �����ϰ��� �ϴ� Ŭ���� �Ӽ� ����
				FastVector fvClassVal = new FastVector(4);
				fvClassVal.addElement("NORMAL");
				fvClassVal.addElement("DISTRACTION");
				fvClassVal.addElement("DROWSINESS");
				fvClassVal.addElement("DRUNKEN");
				Attribute State = new Attribute("State",fvClassVal);
				
				//�Ӽ� ���� ����
				FastVector fvWekaAttributes = new FastVector(21);
				fvWekaAttributes.addElement(CarPosition_x);
				fvWekaAttributes.addElement(CarPosition_y);
				fvWekaAttributes.addElement(CarPosition_z);
				fvWekaAttributes.addElement(CarRotation_x);
				fvWekaAttributes.addElement(CarRotation_y);
				fvWekaAttributes.addElement(CarRotation_z);
				fvWekaAttributes.addElement(CarRotation_w);
				fvWekaAttributes.addElement(CurrentSpeed);
				fvWekaAttributes.addElement(Engine_RPM);
				fvWekaAttributes.addElement(Velocity_x);
				fvWekaAttributes.addElement(Velocity_z);
				fvWekaAttributes.addElement(Acceleration_x);
				fvWekaAttributes.addElement(Acceleration_z);
				fvWekaAttributes.addElement(SteeringAngle);
				fvWekaAttributes.addElement(Brake);
				fvWekaAttributes.addElement(Distance_from_center);
				fvWekaAttributes.addElement(Lane_Number);
				fvWekaAttributes.addElement(Distance_to_left_lane_line);
				fvWekaAttributes.addElement(Distance_to_right_lane_line);
				fvWekaAttributes.addElement(Offset_from_lane_center);
				fvWekaAttributes.addElement(State);
				
				
				//�� �н����� �����.
				Instances testSet = new Instances("real", fvWekaAttributes,1);
				
				//Ŭ���� �ε����� �����Ѵ�.
				testSet.setClassIndex(20);
				
				                                   
				// �׽�Ʈ  �ν��Ͻ� ����
				//������ ���� �Ӽ� �� 
				Instance test = new Instance(21);
				test.setValue((Attribute)fvWekaAttributes.elementAt(0), MyCarLogging.getInstance().getPosition().getX());
				test.setValue((Attribute)fvWekaAttributes.elementAt(1), MyCarLogging.getInstance().getPosition().getY());
				test.setValue((Attribute)fvWekaAttributes.elementAt(2), MyCarLogging.getInstance().getPosition().getZ());
				test.setValue((Attribute)fvWekaAttributes.elementAt(3), MyCarLogging.getInstance().getRotation().getX());
				test.setValue((Attribute)fvWekaAttributes.elementAt(4), MyCarLogging.getInstance().getRotation().getY());
				test.setValue((Attribute)fvWekaAttributes.elementAt(5), MyCarLogging.getInstance().getRotation().getZ());
				test.setValue((Attribute)fvWekaAttributes.elementAt(6), MyCarLogging.getInstance().getRotation().getW());
				test.setValue((Attribute)fvWekaAttributes.elementAt(7), MyCarLogging.getInstance().getSpeed());
				test.setValue((Attribute)fvWekaAttributes.elementAt(8), MyCarLogging.getInstance().getRpm());
				test.setValue((Attribute)fvWekaAttributes.elementAt(9), MyCarLogging.getInstance().GetVelocity_X());
				test.setValue((Attribute)fvWekaAttributes.elementAt(10),MyCarLogging.getInstance().GetVelocity_Z());
				test.setValue((Attribute)fvWekaAttributes.elementAt(11),MyCarLogging.getInstance().GetAcceleration_X());
				test.setValue((Attribute)fvWekaAttributes.elementAt(12),MyCarLogging.getInstance().GetAcceleration_Z());
				test.setValue((Attribute)fvWekaAttributes.elementAt(13),MyCarLogging.getInstance().getSteeringAngle());
				test.setValue((Attribute)fvWekaAttributes.elementAt(14),MyCarLogging.getInstance().getBrake());
				test.setValue((Attribute)fvWekaAttributes.elementAt(15),MyCarLogging.getInstance().getDistanceFromCenterLine());
				test.setValue((Attribute)fvWekaAttributes.elementAt(16),MyCarLogging.getInstance().getCurrentLine());
				test.setValue((Attribute)fvWekaAttributes.elementAt(17),MyCarLogging.getInstance().getLeftLaneDistance());
				test.setValue((Attribute)fvWekaAttributes.elementAt(18),MyCarLogging.getInstance().getRightLaneDistance());
				test.setValue((Attribute)fvWekaAttributes.elementAt(19),MyCarLogging.getInstance().getOffsetFromLaneCenter());
				test.setValue((Attribute)fvWekaAttributes.elementAt(20),MyCarLogging.getInstance().getState()+"");
								
				//Instance �߰�
				testSet.add(test);
				
				//���ο� �ν��Ͻ����� �̿��Ͽ� ����
				normalEval.evaluateModel(normalTree, testSet);
							
			
				//���� ȥ�� ��Ŀ��� ������  �ִ� ���� ���� �����ϸ� �׻��·� ���� 
				for(int i= 0; i <4; i++){
					if(state_matrix[i][0] != normalEval.confusionMatrix()[i][0]){
						MyCarLogging.getInstance().setState(STATE.NORMAL);
					}	
					if(state_matrix[i][1] != normalEval.confusionMatrix()[i][1]){
						MyCarLogging.getInstance().setState(STATE.DISTRACTION);
					}
					if(state_matrix[i][2] != normalEval.confusionMatrix()[i][2]){
						MyCarLogging.getInstance().setState(STATE.DROWSINESS);
					}
					if(state_matrix[i][3] != normalEval.confusionMatrix()[i][3]){
						MyCarLogging.getInstance().setState(STATE.DRUNKEN);
					}	
				}

				state_matrix = normalEval.confusionMatrix();
				
			
				Thread.sleep(1000); //1�� �ֱ�� �ݺ�����
				
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		
			}
		}
		
	}
}

//---End KSS
