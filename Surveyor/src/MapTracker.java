import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;


public class MapTracker implements LineTracker
{
    public static enum PlanState{NONE, DIRECT, PLAN};
    final static float[] speedMult = {1.0f, 0.75f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PoseProvider imuPose;
    Pose deltaPose = new Pose();
    public Pose targetPose = new Pose();
    float heading = 0.0f;
    public float targetSpeed = 0.0f;
    public PlanState planState;
    
    public MapTracker(LineFollowerChassis chassis)
    {
        imuPose = chassis.getIMUPoseProvider();
    }

    @Override
    public int sampleSize()
    {
        // TODO Auto-generated method stub
        return 2;
    }

    @Override
    public synchronized void fetchSample(float[] sample, int offset)
    {
        Pose slamPose = addPose(imuPose.getPose(), deltaPose);
        //System.out.println("IMU heading " + imuPose.getPose().getHeading());
        //float error = normalize(heading - imuPose.getPose().getHeading());
        //System.out.println("target " + targetPose + " imu " + imuPose.getPose() + " slam " + slamPose);
        float error = 0;
        switch(planState)
        {
        case NONE:
            break;
        case DIRECT:
            error = normalize(slamPose.angleTo(targetPose.getLocation()) - slamPose.getHeading());
            break;
        case PLAN:
            error = normalize(heading - slamPose.getHeading());
            break;
        }
        if (planState == PlanState.NONE)
        {
            sample[offset] = 0.0f;
            sample[offset+1] = 0.0f;            
        }
        else
        {
            sample[offset] = error/100;
            if (slamPose.distanceTo(targetPose.getLocation()) < 100)
            {
                sample[offset+1] = 0.0f;
                planState = PlanState.NONE;
            }
            else
                sample[offset+1] = targetSpeed*speedMult[(int)(Math.abs(error)/22.5f)];            
        }
        System.out.printf("Error %f speed %f\n", sample[0], sample[1]);
    }

    @Override
    public boolean startCalibration()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void stopCalibration()
    {
        // TODO Auto-generated method stub

    }

    @Override
    public void calibrate()
    {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean marker()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void close()
    {
        // TODO Auto-generated method stub

    }

    @Override
    public void setEdge(int edge)
    {
        // TODO Auto-generated method stub

    }

    public void setHeading(float newHeading)
    {
        heading = newHeading;
    }
    
    protected float normalize(float h)
    {
        while (h < -180) h += 360;
        while (h > 180) h -= 360;
        return h;        
    }
    
    protected Pose poseDelta(Pose op, Pose np)
    {
        Pose ret = new Pose();
        ret.setLocation(np.getX() - op.getX(), np.getY() - op.getY());
        ret.setHeading(normalize(np.getHeading() - op.getHeading()));
        return ret;
        
    }
    
    protected Pose addPose(Pose op, Pose np)
    {
        Pose ret = new Pose();
        ret.setLocation(np.getX() + op.getX(), np.getY() + op.getY());
        ret.setHeading(normalize(np.getHeading() + op.getHeading()));
        return ret;
        
    }
    
    public synchronized void setTarget(PlanState state, Pose p, float speed, float heading)
    {
        if (state == PlanState.NONE)
        {
            if (planState != PlanState.NONE)
            {
                targetSpeed = 0;
                targetPose = new Pose();
                planState = PlanState.NONE;
            }
            return;
        }
            
        if (p.getX() != targetPose.getX() || p.getY() != targetPose.getY() || p.getHeading() != targetPose.getHeading())
        {
            Pose pnew = new Pose(p.getX(), p.getY(), p.getHeading());
            targetPose = pnew;
            planState = state;
        }
        targetSpeed = speed;
        this.heading = heading;
    }
    
    public synchronized void setPoseDelta(Pose p)
    {
        Pose pnew = new Pose(p.getX(), p.getY(), p.getHeading());
        deltaPose = pnew;
        System.out.println("new delta " + pnew);
    }
    
    public synchronized Pose getSlamPose()
    {
        return addPose(imuPose.getPose(), deltaPose);    
    }
    
}
