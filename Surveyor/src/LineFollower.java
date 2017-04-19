import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.navigation.LineFollowingMoveController;
import lejos.utility.Delay;


public class LineFollower implements Runnable
{
    public enum FollowState {STOPPED, PAUSED, RUNNING, EXIT}
    
    protected Chassis chassis;
    protected MapTracker tracker;
    protected float P, I, D;
    protected FollowState state = FollowState.STOPPED;
    protected Thread thread;
    //protected static final float MAX_STEER = 125.0f;
    protected static final float MAX_STEER = 10.0f;
    protected static final long LOOP_TIME = 100;
    protected float curSpeed = 0.0f;
    protected float targetSpeed = curSpeed;
    protected float acceleration = 50.0f;
    protected float outputVal;
    
    public LineFollower(Chassis chassis, MapTracker sensor, float P, float I, float D)
    {
        this.chassis = chassis;
        this.tracker = sensor;
        this.P = P;
        this.I = I;
        this.D = D;
        thread = new Thread(this);
        thread.setDaemon(true);
        thread.start();
    }

    private void sample()
    {
        float [] sample = new float[tracker.sampleSize()];
        while (chassis.isMoving())
        {
            tracker.fetchSample(sample, 0);            
        }
    }
    
    
    public void calibrate()
    {
        chassis.setSpeed(10, 10);
        Audio audio = BrickFinder.getLocal().getAudio();
        audio.systemSound(Audio.ASCENDING);
        if (tracker.startCalibration())
        {
            chassis.arc(0, 30);
            sample();
            chassis.arc(0, -60);
            sample();
            chassis.arc(0, 30);
            sample();
        }
        tracker.stopCalibration();
        audio.systemSound(Audio.DESCENDING);       
    }
    
    public void setState(FollowState newState)
    {
        state = newState;
    }

    /*
    public void setSpeed(float newSpeed)
    {
        targetSpeed = newSpeed;
    }
    */
    public void setHeading(float newHeading)
    {
        tracker.setHeading(newHeading);
    }
    

    public float getSpeed()
    {
        return curSpeed;
    }
    
    public boolean isRunning()
    {
        return state == FollowState.RUNNING || state == FollowState.PAUSED;
    }
    
    public boolean isStopped()
    {
        return state == FollowState.STOPPED;    
    }
    
    @Override
    public void run()
    {
        float prevError = 0.0f;
        float totalError = 0.0f;
        float [] sample = new float[tracker.sampleSize()];
        for(;;)
        {
            long start = System.currentTimeMillis();
            switch (state)
            {
            case PAUSED:
                chassis.setVelocity(0, 0);
                curSpeed = 0;
                break;
                
            case RUNNING:
                tracker.fetchSample(sample, 0);
                float error = sample[0];
                targetSpeed = sample[1];
                // Accumulate errors for I term
                //totalError = totalError * 0.95f;
                
                //if (error*totalError <= 0)
                    //totalError = totalError*0.80f;
                
                //totalError += error + (error - prevError);
                totalError += error;
                //if (totalError*I > MAX_STEER/2)
                    //totalError = MAX_STEER/2/I;
                //else if (totalError*I < -MAX_STEER/2)
                    //totalError = -MAX_STEER/2/I;
                // calculate PID value
                //System.out.print("E " + error + " P " + P*error + " I " + I*totalError + " D " + D*(error - prevError));
                float output = P*error + I*totalError + D*(error - prevError);
                //System.out.println("E " + error + " P " + P*error + " I " + I*totalError + " D " + D*(error - prevError) + " O " + output);
                prevError = error;
                // limit it
                if (output > MAX_STEER)
                    output = MAX_STEER;
                else if (output < -MAX_STEER)
                    output = -MAX_STEER;
                // adjust speed if needed
                if (curSpeed != targetSpeed)
                {
                    float accel = acceleration*LOOP_TIME/1000;
                    if (curSpeed < targetSpeed)
                    {
                        curSpeed += accel;
                        if (curSpeed > targetSpeed)
                            curSpeed = targetSpeed;
                    }
                    else
                    {
                        curSpeed -= accel;
                        if (curSpeed < targetSpeed)
                            curSpeed = targetSpeed;
                    }
                }
                // steer as needed
                outputVal = output;
                /*
                if (curSpeed == 0.0f)
                {
                    output = 0;
                    totalError = 0;
                }
                */
                //System.out.println("curSpeed " + curSpeed + " control " + output);
                chassis.setVelocity(curSpeed, output);
                //System.out.println("" + ((LineFollowerChassis)chassis).getDistance() + ", " + error);
                //System.out.println(" S " + output );
                break;
            case EXIT:
                System.out.println("Chassis exit");
                chassis.setVelocity(0,  0);
                System.out.println("velocity set");
                chassis.waitComplete();
                System.out.println("Chassis complete");
                state = FollowState.STOPPED;
                return;
            default:
                break;
            }
            long delay = LOOP_TIME - (System.currentTimeMillis() - start);
            //if (delay <= 0) System.out.println("Out of time");
            Delay.msDelay(delay);
        }
    }
    
    public float getOutput()
    {
        return outputVal;
    }
    
    
}
