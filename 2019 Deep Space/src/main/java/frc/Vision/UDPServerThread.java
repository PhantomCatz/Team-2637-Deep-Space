package frc.Vision;

import java.io.IOException;
import java.lang.reflect.Type;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.concurrent.ConcurrentHashMap;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

//public class UDPServer implements Runnable
public class UDPServerThread extends Thread {
    DatagramSocket socket = null;

    private static final int PORT_NUMBER = 5005;

    // private Thread t;

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

                //VisionObjContainer.update(null);
                
                ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>> chm;
                //ConcurrentHashMap<String, VisionObject> chm;

                //System.out.println("Attempting to recieve a packet");

                DatagramPacket incomingPacket = new DatagramPacket(incomingData, incomingData.length);

                socket.receive(incomingPacket);

                byte[] data = incomingPacket.getData();

                try
                {
                    //constructing a string using a byte array will cause the constructed
                    //string to have the exact length of the byte array which means a lot
                    //of trailing whitespace.
                    String json = new String(data).trim();

                    //System.out.println(json);
                    //System.out.println("length: " + json.length());

                    Gson gson = new GsonBuilder().create();

                    Type map = new TypeToken<ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>>>(){}.getType();
                    //Type map = new TypeToken<ConcurrentHashMap<String, VisionObject>>(){}.getType();

                    chm = gson.fromJson(json, map);
                    VisionObjContainer.overwriteMap(chm);

                    //VisionObjContainer.update(gson.fromJson(json, VisionObject.class));
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

    /*
    public void start()
    {
        System.out.println("Starting UDP Server");

        if (t == null)
        {
            t = new Thread(this, threadName);
            t.start();
        }
    }
    */
}