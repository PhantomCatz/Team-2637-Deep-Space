package frc.Vision;

import java.util.concurrent.ConcurrentHashMap;
import java.util.Enumeration;

public class VisionObjContainer
{
    private static final Object lock = new Object();
    
    /*
    private static volatile VisionObject m_vobj = null;

    public static void update(VisionObject vobj)
    {
        synchronized(lock)
        {
            m_vobj = vobj;
        }
    }

    public static VisionObject get()
    {
        synchronized(lock)
        {
            return m_vobj;
        }
    }
    
    /*/
    
    //private static volatile ConcurrentHashMap<String, VisionObject> m_chm = new ConcurrentHashMap<>();
    private static volatile ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>> m_chm = new ConcurrentHashMap<>();

    //public static void overwriteMap(ConcurrentHashMap<String, VisionObject> chm)
    public static void overwriteMap(ConcurrentHashMap<String, ConcurrentHashMap<String, VisionObject>> chm)
    {
        synchronized(lock)
        {
            m_chm = chm;
        }
    }

    public static Enumeration<VisionObject> getElements()
    {
        synchronized(lock)
        {
            try 
            {
                //System.out.println(m_chm.get("auto").mappingCount());

                return m_chm.get("auto").elements();
                //return m_chm.elements();
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
                return m_chm.get("auto").get(key);
                //return m_chm.get(key);
            }
            catch (NullPointerException e)
            {
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