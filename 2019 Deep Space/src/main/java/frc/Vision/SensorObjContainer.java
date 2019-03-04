package frc.Vision;

import java.util.concurrent.ConcurrentHashMap;
import java.util.Enumeration;
//import frc.Vision.*;

public class VisionObjContainer
{
    private static final Object lock = new Object();
    
    private static volatile ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>> m_chm = new ConcurrentHashMap<>();

    public static void overwriteMap(ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>> chm)
    {
        synchronized(lock)
        {
            m_chm = chm;
            //System.out.println("map overwritten");
        }
    }

    public static Enumeration<VisionObject> getElements()
    {
        synchronized(lock)
        {
            try 
            {
                return m_chm.get("auto").elements();
            }
            catch (NullPointerException e)
            {
                return null;
            }
        }
    }

    public static VisionObject get(String key)
    {
        synchronized(lock)
        {
            try
            {
                //System.out.println("VisionObject.get check");
                return m_chm.get("auto").get(key);
                //return m_chm.get(key);
            }
            catch (NullPointerException e)
            {
                //System.out.println("VisionObject.get null check");
                
                return null;
            }
        }
    }

    public static void update(String key, VisionObject value)
    {
        synchronized(lock)
        {
            try
            {
                m_chm.get("auto").replace(key, value);
                //m_chm.replace(key, value);
            }
            catch (NullPointerException e)
            {
                return;
            }
        }
    }

    //*/
}