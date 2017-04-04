import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import lejos.hardware.Battery;
import lejos.hardware.Power;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;


public class StatusTracker implements Runnable
{
    protected final LineFollowerChassis chassis;
    protected final LineFollower follower;
    protected final String name;
    protected final Socket sock;
    protected final DataOutputStream dos;
    protected final DataInputStream dis;
    protected final PoseProvider odom;
    protected final PoseProvider imu;
    protected Lidar.LidarScan scan;
    protected final Power battery;
    protected final SampleProvider infraRed;
    protected final MapTracker tracker;

    public StatusTracker(LineFollowerChassis chassis, LineFollower follower, Pose p, String name, String host, Power batt, SampleProvider ir, MapTracker tracker) throws UnknownHostException, IOException
    {
        this.chassis = chassis;
        this.name = name;
        odom = chassis.getPoseProvider();
        imu = chassis.getIMUPoseProvider();
        battery = batt;
        infraRed = ir;
        this.follower = follower;
        this.tracker = tracker;
        setPose(p);
        sock = new Socket(host, 2446);
        dos = new DataOutputStream(sock.getOutputStream());
        dis = new DataInputStream(sock.getInputStream());
        dos.writeUTF(name);
        Thread thread = new Thread(this);
        thread.setDaemon(true);
        thread.start();
    }
    
    public void setScan(Lidar.LidarScan s)
    {
        scan = s;
    }
    
    public void setPose(Pose p)
    {
        odom.setPose(p);
        imu.setPose(p);
    }
    
    
    @Override
    public void run()
    {
        float heading=0.0f;
        float[] sample = new float[1];
        Pose p = new Pose();
        while(true)
        {
            if (follower.isRunning())
                try
                {
                    infraRed.fetchSample(sample, 0);
                    dos.writeFloat(sample[0]);
                    if (scan != null)
                    {
                        dos.writeBoolean(true);
                        scan.pose.dumpObject(dos);
                        dos.writeLong(scan.timestamp);
                        for(int i=0; i < scan.ranges.length; i++)
                            dos.writeInt(scan.ranges[i]*10);
                        scan = null;
                    }
                    else
                        dos.writeBoolean(false);
                    odom.getPose().dumpObject(dos);
                    imu.getPose().dumpObject(dos);
                    tracker.getSlamPose().dumpObject(dos);
                    dos.writeFloat(follower.getSpeed());
                    dos.writeFloat(battery.getVoltage());
                    float speed = dis.readFloat();
                    if (dis.readBoolean())
                    {
                        p.loadObject(dis);
                        tracker.setPoseDelta(p);
                    }
                    int state = dis.readInt();
                    switch(state)
                    {
                    case 0:
                        tracker.setTarget(MapTracker.PlanState.NONE, null, 0.0f, 0.0f);
                        break;
                    case 1:
                        p.loadObject(dis);
                        tracker.setTarget(MapTracker.PlanState.DIRECT, p, speed, 0.0f);
                        break;
                    case 2:
                        p.loadObject(dis);
                        heading = dis.readFloat();
                        tracker.setTarget(MapTracker.PlanState.PLAN, p, speed, heading);
                        break;
                    default:
                        System.out.println("Bad plan state");
                        break;
                    }
                    if (speed == -2)
                    {
                        /*
                        follower.setSpeed(0);
                        while(follower.getSpeed() > 0)
                            Delay.msDelay(100);
                        */
                        follower.setState(LineFollower.FollowState.EXIT);
                        return;
                    }
                    if (speed == -1)
                        follower.setState(LineFollower.FollowState.PAUSED); 
                    else
                    {
                        follower.setState(LineFollower.FollowState.RUNNING);
                        //follower.setSpeed(speed);
                    }
                } catch (IOException e)
                {
                    return;
                }
            Delay.msDelay(100);
        }
        
    }
    
    public void close()
    {
        try
        {
            dis.close();
            dos.close();
            sock.close();
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
