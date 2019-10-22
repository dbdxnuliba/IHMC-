package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPClient implements Runnable
{
   //params
   final String serverIPAddressString = "localhost";
   final int serverPort = 6669;    
   final int sleepBetweenSendings = 1500;
   
   int counter = 0;

   @Override
   public void run()
   {
      try
      {
         InetAddress serverIPAddress = InetAddress.getByName(serverIPAddressString);
         DatagramSocket clientSocket = new DatagramSocket(serverPort+1);
         byte[] sendData;
         DatagramPacket sendPacket;
         
         while(true) 
         {
            //send
            ++counter;
            String request = String.valueOf(counter);    
            sendData = request.getBytes();
            sendPacket = new DatagramPacket(sendData, sendData.length, serverIPAddress, serverPort);            
            System.out.println("Client sending: " + String.valueOf(counter));            
            clientSocket.send(sendPacket);
                        
            //recieve
            byte[] receiveData = new byte[1024]; 
            DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);             
            clientSocket.receive(receivePacket); 
            String response = new String(receivePacket.getData()).trim();            
            System.out.println("Client recieving: " + String.valueOf(response));   

            Thread.sleep(sleepBetweenSendings);
         }  
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
