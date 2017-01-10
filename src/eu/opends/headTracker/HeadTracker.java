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
 * HeadTracker�� �����͸� �������� �޾ƿ� �Ľ��Ͽ� �����Ѵ�.
 * */

public class HeadTracker extends Thread{

	Socket socket = null;
	BufferedInputStream bis = null;
	
	
	/*
	 * KSS
	 * <������>
	 * HeadTracker�� �����͸� ���� ������ �����ϱ� ���� ���ϰ� ���۽�Ʈ���� �����Ѵ�.
	 * */
	
	public HeadTracker(){

    	         
	       try {
	       	socket = new Socket("127.0.0.1",23000); //���� ������ ���ϻ���
	       	bis = new BufferedInputStream(socket.getInputStream()); // ������ ���� ���۽�Ʈ�� ����        	        	         

	       	start(); //������� �ޱ�
	       
	           
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
	 * <������>
	 * �������� �������� Head ������ �޾Ƽ� �Ľ��Ͽ� ���� �����Ѵ�. 
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
	       	
	       	while ( (s = bis.read()) != -1) {	     // ������ �޾Ƽ� �Ľ��ϱ�   		
				
	       			char chr = (char)s;
	       
					if(chr == ';'){
						
						if(i==6){
							i=0;
						}
						
						/*
						 * KSS
						 * pitch,yaw,roll,x,y,z ���� ���� �޾ƿͼ� ����
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