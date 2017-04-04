import lejos.robotics.SampleProvider;


public interface LineTracker extends SampleProvider
{
    public static final int EDGE_LEFT = -1;
    public static final int EDGE_RIGHT = 1;
    
    public boolean startCalibration();
    public void stopCalibration();
    public void calibrate();
    public boolean marker();
    public void close();
    public void setEdge(int edge);
}
