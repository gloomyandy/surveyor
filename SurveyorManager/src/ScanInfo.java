import lejos.robotics.navigation.Pose;
import edu.wlu.cs.levy.breezyslam.components.Velocities;


public class ScanInfo
{
    final static int POSE_GYRO = 0;
    final static int POSE_SLAM = 1;
    final static int POSE_ODO = 2;
    final static int POSE_CNT = POSE_SLAM+2;
    Velocities velocity;
    int[] rawScan;
    double[] processedScan;
    long timestamp;
    Pose[] poses = new Pose[POSE_CNT];
    //Pose[] scanPoses;
    byte[] map;
    float infraRed;
    double distance;
    
    public ScanInfo(Pose start, long sts, Pose end, long ets, int[] scan, float ir)
    {
        if (start == null || end == null)
        {
            velocity = new Velocities(0.0, 0.0, 0.0);
            if (end != null)
                poses[POSE_GYRO] = end;
            else
                poses[POSE_GYRO] = new Pose();
        }
        else
        {
            poses[POSE_GYRO] = end;
            double dx = end.getX() - start.getX();
            double dy = end.getY() - start.getY();
            double dt = normalize(end.getHeading() - start.getHeading());
            velocity = new Velocities(Math.sqrt(dx*dx + dy*dy), dt, (ets - sts)/1000000.0);
            timestamp = ets;
            
        }
        poses[POSE_ODO] = new Pose();
        rawScan = new int[scan.length];
        System.arraycopy(scan, 0, rawScan, 0, scan.length);        
    }
    
    public void update(double [] points, byte[] map, Pose pose, int distance)
    {
        processedScan = points;
        this.map = map;
        poses[POSE_SLAM] = pose;
        //this.distance = (double)distance/points.length;
        this.distance = (double)distance*100/65500;
    }
    
    protected float normalize(float h)
    {
        while (h < -180) h += 360;
        while (h > 180) h -= 360;
        return h;        
    }
    
    protected Pose subPose(Pose p1, Pose p2)
    {
        Pose ret = new Pose();
        ret.setLocation(p1.getX() - p2.getX(), p1.getY() - p2.getY());
        ret.setHeading(normalize(p1.getHeading() - p2.getHeading()));
        return ret;
        
    }
    
    protected Pose addPose(Pose p1, Pose p2)
    {
        Pose ret = new Pose();
        ret.setLocation(p1.getX() + p2.getX(), p1.getY() + p2.getY());
        ret.setHeading(normalize(p1.getHeading() + p2.getHeading()));
        return ret;        
    }
    
    public Pose getPoseDelta()
    {
        return subPose(poses[POSE_SLAM], poses[POSE_GYRO]);
    }
}
