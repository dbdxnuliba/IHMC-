package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPServer implements Runnable
{   
   //params
   final int port = 6669;   
   
   int counter = 0;
   
   @Override
   public void run()
   {       
      try
      {    
         //variables     
         DatagramSocket serverSocket = new DatagramSocket(port);
         byte[] receiveData = new byte[1024];
         DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);         
         InetAddress clientIPAddress;
         int clientPort;
         String request;  
                
         byte[] sendData;
         DatagramPacket sendPacket;
         String response;  
         
         while(true){
            //recieve
            serverSocket.receive(receivePacket);
            clientIPAddress = receivePacket.getAddress();
            clientPort = receivePacket.getPort();
            request = new String(receivePacket.getData()).trim();
            System.out.println("Server recieving: " + String.valueOf(request));
            
            //send      
            ++counter;
            response = String.valueOf(counter);
            sendData = response.getBytes();
            sendPacket = new DatagramPacket(sendData, sendData.length, clientIPAddress, clientPort);
            System.out.println("Server sending: " + String.valueOf(counter));
            serverSocket.send(sendPacket); 
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args) {
      /*
      //UDPServer and UDPClient testing
      Thread server = new Thread(new UDPServer());
      server.start();
      
      Thread client = new Thread(new UDPClient());
      client.start();      
      */
      
      //UDPDataSender testing
      UDPDataSender sender = new UDPDataSender();  
      int counter = 0;
      while(true) {
         sender.sendDistance(++counter);
         try {
            Thread.sleep(1500);            
         }
         catch (Exception e) {
            e.printStackTrace();
         }
      }      
   }
}