package eu.opends.multiDriver;

import java.util.ArrayList;

import antlr.collections.List;

//Start --- KSS

/*
 * KSS
 * ��Ƽī���� �������ִ¾���� Ŭ����
 * */
public class MultiAdapter {
	
	/*
	 * KSS
	 * ��ü������ ������ �̷������ �ʰ� �̱��������� �����Ѵ�.
	 * �ϳ��� ��ü������ ���� ���� Ŭ�������� ���ٰ����ϰ� �Ѵ�.
	 * */
	
	private static MultiAdapter instance;
	
	public static MultiAdapter getInstance(){
		if(instance == null){
			instance = new MultiAdapter();
			items = new ArrayList<>();
		}
		return instance;
	}
		
	private static ArrayList<MultiCar> items; //��Ƽī���� ���� �迭
	
	
	public void add(MultiCar car){ // ��Ƽī �߰�
		
		items.add(car);
	}
	
	public void clear(){ // ���� �����.
		items.clear();
	}
	
	public void addAll(ArrayList<MultiCar> item){ // ��Ƽī �迭�� �߰�
		items.addAll(item);
	}
		
	
	public ArrayList<MultiCar> getMulticarList(){ //��Ƽī���� ���� �迭�� ��´�.
		return items;
	}
	
}
//End --- KSS
