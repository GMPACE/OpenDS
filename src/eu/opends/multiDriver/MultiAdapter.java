package eu.opends.multiDriver;

import java.util.ArrayList;

import antlr.collections.List;

//Start --- KSS

/*
 * KSS
 * 멀티카들을 관리해주는어댑터 클래스
 * */
public class MultiAdapter {
	
	/*
	 * KSS
	 * 객체생성이 여러번 이루어지지 않게 싱글톤패턴을 적용한다.
	 * 하나의 객체생성을 통해 여러 클래스에서 접근가능하게 한다.
	 * */
	
	private static MultiAdapter instance;
	
	public static MultiAdapter getInstance(){
		if(instance == null){
			instance = new MultiAdapter();
			items = new ArrayList<>();
		}
		return instance;
	}
		
	private static ArrayList<MultiCar> items; //멀티카들을 담을 배열
	
	
	public void add(MultiCar car){ // 멀티카 추가
		
		items.add(car);
	}
	
	public void clear(){ // 전부 지운다.
		items.clear();
	}
	
	public void addAll(ArrayList<MultiCar> item){ // 멀티카 배열로 추가
		items.addAll(item);
	}
		
	
	public ArrayList<MultiCar> getMulticarList(){ //멀티카들을 담을 배열을 얻는다.
		return items;
	}
	
}
//End --- KSS
