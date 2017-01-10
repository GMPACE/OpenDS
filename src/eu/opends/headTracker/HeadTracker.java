package eu.opends.headTracker;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;

import eu.opends.analyzer.MyCarLogging;


//Start ---KSS


/*
 * KSS
 * HeadTracker의 데이터를 서버에서 받아와 파싱하여 저장한다.
 * */

public class HeadTracker extends Thread{

	Socket socket = null;
	BufferedInputStream bis = null;
	
	
	/*
	 * KSS
	 * <생성자>
	 * HeadTracker의 데이터를 받을 서버에 접속하기 위해 소켓과 버퍼스트림을 생성한다.
	 * */
	
	public HeadTracker(){

    	         
	       try {
	       	socket = new Socket("127.0.0.1",23000); //서버 접속할 소켓생성
	       	bis = new BufferedInputStream(socket.getInputStream()); // 데이터 받을 버퍼스트림 생성        	        	         

	       	start(); //쓰레드로 받기
	       
	           
	       }catch (ConnectException e){
	    	   
	    	   System.err.println("No HeadTraker connection possible to server");
	       } catch (UnknownHostException e) {
	           System.out.println("Unkonw exception " + e.getMessage());
	           	
	       } catch (IOException e) {
	           System.out.println("IOException caught " + e.getMessage());
	    
	       } 
    }
	

	/*
	 * KSS
	 * <쓰레드>
	 * 매프레임 운전자의 Head 정보를 받아서 파싱하여 값을 저장한다. 
	 * */

	@SuppressWarnings("finally")
	@Override
	public void run(){
		
		while(true)
		{
		try{
		
	       	int s = 0,i=0;
	       	String data="";
	       	String []a = null;
	       	
	       	while ( (s = bis.read()) != -1) {	     // 데이터 받아서 파싱하기   		
				
	       			char chr = (char)s;
	       
					if(chr == ';'){
						
						if(i==6){
							i=0;
						}
						
						/*
						 * KSS
						 * pitch,yaw,roll,x,y,z 축의 값을 받아와서 저장
						 * */
						
						if(i==0){
							MyCarLogging.getInstance().setHeadTraker_pitch(Float.parseFloat(data));
						}
						else if(i==1){
							MyCarLogging.getInstance().setHeadTraker_yaw(Float.parseFloat(data));
						}
						else if(i==2){
							MyCarLogging.getInstance().setHeadTraker_roll(Float.parseFloat(data));
						}
						else if(i==3){
							MyCarLogging.getInstance().setHeadTraker_x(Float.parseFloat(data));
						}
						else if(i==4){
							MyCarLogging.getInstance().setHeadTraker_y(Float.parseFloat(data));
						}
						else if(i==5){
							MyCarLogging.getInstance().setHeadTraker_z(Float.parseFloat(data));
						}
						
						data=""; 
						i++;
					}
					else if(chr ==' '){
						
					}
					else{
						data =data+ new Character(chr).toString();
				    	//System.out.println(data);
					}
					
				
	       		
			}
	       	

	       	
			} catch (UnknownHostException e) {
            	System.out.println("Unkonw exception " + e.getMessage());

			} catch (IOException e) {
            	System.out.println("IOException caught " + e.getMessage());
            	e.printStackTrace();
        	} catch (Exception e){
        		break;
			}finally {
        		break;
			}
		}
		
       	try {
			bis.close();
		  	socket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
       
	}
	
}
//End ---KSS