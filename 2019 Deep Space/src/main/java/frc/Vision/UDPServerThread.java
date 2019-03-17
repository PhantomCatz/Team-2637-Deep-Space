package frc.Vision;

import java.io.IOException;
import java.lang.reflect.Type;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.concurrent.ConcurrentHashMap;

import frc.Vision.SensorObject;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

public class UDPServerThread extends Thread
{
    DatagramSocket socket = null;

    private static final int PORT_NUMBER = 5005;

    private String threadName = "UDPServer";

    public UDPServerThread() 
    {
        this.setName(threadName);
        System.out.println("Starting UDP Server");
    }

    public void run()
    {
        try
        {
            socket = new DatagramSocket(PORT_NUMBER);

            byte[] incomingData;

            while (true)
            {
                incomingData = new byte[256];
                ConcurrentHashMap<String, SensorObject> chm;
                DatagramPacket incomingPacket = new DatagramPacket(incomingData, incomingData.length);

                socket.receive(incomingPacket);

                byte[] data = incomingPacket.getData();

                try
                {
                    //constructing a string using a byte array will cause the constructed
                    //string to have the exact length of the byte array which means a lot
                    //of trailing whitespace.
                    String json = new String(data).trim();

                    Gson gson = new GsonBuilder().create();

                    Type map = new TypeToken<ConcurrentHashMap<String, SensorObject>>(){}.getType();

                    chm = gson.fromJson(json, map);
                    //SensorObjContainer.overwriteMap(chm);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }

                InetAddress IPAddress = incomingPacket.getAddress();
                int port = incomingPacket.getPort();

                String reply = "ack";
                byte[] replyBytes = reply.getBytes();

                DatagramPacket replyPacket = new DatagramPacket(replyBytes, 
                                                                replyBytes.length, 
                                                                IPAddress, 
                                                                port);
                
                socket.send(replyPacket);
                Thread.sleep(10);
            }
        }
        catch (SocketException e)
        {
            e.printStackTrace();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }
}