import lejos.robotics.SampleProvider;
import lejos.robotics.filter.LinearCalibrationFilter;
import lejos.robotics.filter.MedianFilter;
import lejos.utility.Delay;


public class LightSensorLineTracker implements LineTracker, Runnable
{
    protected static final int MARKER_WIDTH = 20; 
    protected static final float MARKER_VAL = 1.25f;
    protected LinearCalibrationFilter source;
    protected static final int CALIBRATE = 100;
    protected float prevSample = 0.0f;
    protected boolean markerFound = false;
    protected float curSample = 0.0f;
    protected LineFollowerChassis chassis;
    protected boolean active = false;
    
    public LightSensorLineTracker(SampleProvider sensor, LineFollowerChassis chassis)
    {
        // we need to calibrate the output to be in the range -1.0 to 1.0
        source = new LinearCalibrationFilter(sensor);
        source.setCalibrationType(LinearCalibrationFilter.OFFSET_AND_SCALE_CALIBRATION);
        source.setScaleCalibration(1);
        this.chassis = chassis;
        Thread sampler = new Thread(this);
        active = true;
        sampler.setDaemon(true);
        sampler.start();
    }

    public boolean startCalibration()
    {
        //calibrate();
        source.startCalibration();
        // say we need to scan the line
        return true;
    }
    
    public void stopCalibration()
    {
        source.stopCalibration();
        System.out.println("offset " + source.getOffsetCorrection()[0] + " scale " + source.getScaleCorrection()[0]);
    }
    
    public void calibrate()
    {
        float[] sample = new float[source.sampleSize()];
        for(int i = 0; i < 100; i++)
        {
            source.fetchSample(sample, 0);
            Delay.msDelay(5);
        }
    }
    
    @Override
    public int sampleSize()
    {
        // TODO Auto-generated method stub
        return source.sampleSize();
    }

    public void fetchSample(float[] sample, int offset)
    {
        sample[offset] = curSample;
        return;
        /*
        // TODO Auto-generated method stub
        source.fetchSample(samples, 0);
        Delay.msDelay(3);
        source.fetchSample(samples, 1);
        System.out.println(" " + samples[0] + " " + samples[1]);
        if (samples[1] > 1.25)
        {
            samples[1] = 0;
            if (samples[0] > 1.25)
            {
                markerFound = true;
                samples[0]= 0;
            }
        }
        else if (samples[0] > 1.25)
            samples[1] = 0;
        sample[offset] = (samples[0] + samples[1])/2;
        */
    }
    
    public boolean marker()
    {
        boolean ret = markerFound;
        markerFound = false;
        return ret;
    }

    @Override
    public void run()
    {
        float [] sample = new float[source.sampleSize()];
        double markerStart = 0;
        // TODO Auto-generated method stub
        while (active)
        {
            Delay.msDelay(3);
            source.fetchSample(sample,  0);
            float curVal = sample[0];
            // could this be the start of a marker?
            if (curVal > MARKER_VAL)
            {
                // replace val with straight on
                curSample = 0.0f;
                // Are we ignoring markers?
                if (chassis.getDistance() < markerStart + MARKER_WIDTH)
                    continue;
                // assume we are at the start of a marker, keep sampling for a distance to make sure
                double pos = chassis.getDistance();
                while (active && curVal > MARKER_VAL && chassis.getDistance() < pos + MARKER_WIDTH/4)
                {
                    Delay.msDelay(3);
                    source.fetchSample(sample,  0);
                    curVal = sample[0];
                }
                if (!active) break;
                if (curVal > MARKER_VAL)
                {
                    // we have detected a marker for the minimum width, def. get a marker
                    markerFound = true;
                    // skip rest of marker
                    markerStart = pos;
                }
                else
                {
                    // false alarm
                    curSample = curVal;
                }
            }
            else
                curSample = curVal;
        }
        
    }
    
    public void close()
    {
        active = false;
    }
    
}
