import java.awt.Color;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import edu.wlu.cs.levy.breezyslam.algorithms.DeterministicSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.RMHCSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.SinglePositionSLAM;
import edu.wlu.cs.levy.breezyslam.components.Laser;
import edu.wlu.cs.levy.breezyslam.components.Position;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;


public class RobotInfo
{
    public static final int SHOW_PATH_PLANNING=ScanInfo.POSE_CNT;;
    public static final int SHOW_SCANS=SHOW_PATH_PLANNING+1;
    public static final int SHOW_SCAN_HISTORY=SHOW_SCANS+1;
    public static final int SHOW_CNT = SHOW_SCAN_HISTORY+1;
    public boolean[] displayOptions = {false, true, false, false, true, false};
    
    public static enum RunState{INIT, PAUSE, PLAY, STEP, BACK, FORWARD, START, END, REWIND, SETPOS, RUN, RUNPAUSE, EXIT};
    protected RunState state = RunState.INIT;
    protected Track.TrackInfoView view;
    public String name;
    public Pose slamPose = new Pose();
    public Pose gyroPose = new Pose();
    public ScanInfo currentScan = null;
    public List<ScanInfo> scanHistory = new ArrayList<ScanInfo>();
    protected int rawScans = 0;
    public int processedScans = 0;
    public int currentScanNo = 0;
    public float actualSpeed;
    public float targetSpeed;
    public float battery;
    public float infraRed;
    public double distance;
    public Pose targetPose = new Pose(2560.0f, 5120.0f, 0.0f);
    final static Color[] trackColors = {Color.GREEN, Color.RED, Color.MAGENTA, Color.BLUE};
    final static String[] trackNames = {"odo", "slm", "od2", "plan"};
    
    
    protected Socket sock;
    protected DataInputStream dis;
    protected DataOutputStream dos;
    protected Thread inputThread;
    protected Thread playbackThread;
    protected Thread slamThread;
    protected boolean active;
    protected static Track track;
    protected TrackDisplay map;
    protected boolean updateReq = false;
    protected int reportedScans = -1;
    
    protected static final int LIDAR_OFFSET = 55;
    protected static final int ROBOT_DIAMETER = 250;
    protected Laser laser = new Laser(360, 1.0/2.0, 360.0, 10000, 0, LIDAR_OFFSET);
    public static int    MAP_SIZE_PIXELS = 1024;
    public static double MAP_SIZE_METERS = 10.24;
    private static int    SCAN_SIZE       = 360;
    public static double SCAN_QUALITY_THRESHOLD = 50000.0;
    protected SinglePositionSLAM slam = new RMHCSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, 1);
    //private SinglePositionSLAM slam = new DeterministicSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);
    protected int pendingPos = 0;
    
    public static enum PlanState{NONE, DIRECT, PLAN, EXPLORE, HOME};
    public PlanState planState = PlanState.NONE;
    public GradientPlanner plan;
    protected float heading = 0f;
    protected Pose homePose = new Pose(2560, 5120, 0);



    public RobotInfo()
    {
        view = track.getView(this);
        view.bind(this);
        map = view.getMap();
        actualSpeed = 0.0f;
        targetSpeed = 0.0f;
        state = RunState.INIT;
        playbackThread = new Thread(new Runnable(){
            @Override
            public void run()
            {
                runPlayback();
            }});
        slamThread = new Thread(new Runnable(){
            @Override
            public void run()
            {
                runSlam();
            }});
        active = true;
        playbackThread.setDaemon(true);
        playbackThread.start();
        slamThread.setDaemon(true);
        slamThread.start();
    }

    
    public RobotInfo(Socket s)
    {
        this();
        sock = s;
        
        try
        {
            dis = new DataInputStream(s.getInputStream());
            dos = new DataOutputStream(s.getOutputStream());
            name = dis.readUTF();
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
        inputThread = new Thread(new Runnable(){
            @Override
            public void run()
            {
                runInput();
            }});
        inputThread.setDaemon(true);
        inputThread.start();
    }
    
    public RobotInfo(List<ScanInfo> history)
    {
        this();
        synchronized(scanHistory)
        {
            for(ScanInfo info : history)
                scanHistory.add(info);
            rawScans = scanHistory.size();
            scanHistory.notifyAll();
        }
    }
    
    public void close()
    {
        if (active)
        {
            active = false;
            view.release();
        }
    }
    
    public void setSpeed(float s)
    {
        targetSpeed = s;
    }
    
    protected Pose loadPose(DataInputStream s) throws IOException
    {
        Pose p = new Pose();
        p.loadObject(s);
        return p;
    }
    
    protected float getHeadingToWayPoint(GradientPlanner p, Pose start)
    {
        Pose prev=start;
        Pose next=null;
        for(int i = 0; i < 10; i++)
        {
            next = p.nextPose(prev);
            if (next == null)
                return start.angleTo(prev.getLocation());
            prev = next;
        }
        return start.angleTo(next.getLocation());
    }

    protected void runInput()
    {
        Pose prevPose = null;
        long prevTimestamp = 0;
        long baseTimestamp = 0;
        int lastPoseUpdate = 0;
        float newHeading=0.0f;
        System.out.println("Input thread running");
        while (active)
        {
            try
            {
                ScanInfo scan = null;
                infraRed = dis.readFloat();
                Boolean haveScan = dis.readBoolean();
                if (haveScan)
                {
                    System.out.println("got scan");
                    Pose sp = loadPose(dis);
                    long timestamp = dis.readLong()*1000;
                    int[] ranges = new int[SCAN_SIZE];
                    for(int i = 0; i < ranges.length; i++)
                        ranges[i] = dis.readInt() - 95;
                    if (baseTimestamp == 0)
                        baseTimestamp = timestamp;
                    timestamp = timestamp - baseTimestamp;
                    ScanInfo si = new ScanInfo(prevPose, prevTimestamp, sp, timestamp, ranges, infraRed);
                    synchronized(scanHistory)
                    {
                        scanHistory.add(si);
                        rawScans = scanHistory.size();
                        scanHistory.notifyAll();
                    }
                    prevTimestamp = timestamp;
                    prevPose = sp;
                    scan = si;
                }
                Pose odoPose = loadPose(dis);
                gyroPose = loadPose(dis);
                slamPose = loadPose(dis);
                if (scan != null)
                    scan.poses[ScanInfo.POSE_ODO] = odoPose;
                actualSpeed = dis.readFloat();
                battery = dis.readFloat();
                switch(state)
                {
                case RUN:
                    dos.writeFloat(targetSpeed);
                    break;
                case EXIT:
                    dos.writeFloat(-2.0f);
                    break;
                default:
                    dos.writeFloat(-1.0f);
                    break;
                }
                //dos.writeFloat(heading);
                scan = null;
                synchronized(scanHistory)
                {
                    if (processedScans > lastPoseUpdate && processedScans > 10)
                    {
                        scan = scanHistory.get(processedScans-1);
                        lastPoseUpdate = processedScans;
                    }
                }
                if (scan != null && scan.distance < SCAN_QUALITY_THRESHOLD)
                {
                    dos.writeBoolean(true);
                    scan.getPoseDelta().dumpObject(dos);
                }
                else
                    dos.writeBoolean(false);
                synchronized(this)
                {
                    switch(planState)
                    {
                    case NONE:
                        dos.writeInt(0);
                        //dos.writeFloat(heading);
                        break;
                    case DIRECT:
                        dos.writeInt(1);
                        targetPose.dumpObject(dos);
                        break;
                    case PLAN:
                    case EXPLORE:
                        dos.writeInt(2);
                        targetPose.dumpObject(dos);
                        newHeading = getHeadingToWayPoint(plan, slamPose);
                        dos.writeFloat(newHeading);
                        break;
                    }
                }
                if (newHeading != heading)
                    System.out.println("heading " + newHeading);
                heading = newHeading;
                view.update();
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
                view.release();
                this.close();
                
            }
        }
        
    }
    
    
    public void setTargetPose(Pose p, PlanState state)
    {
        GradientPlanner newPlan = null;
        Pose newPose = null;
        switch(state)
        {
        case NONE:
            newPose = null;
            newPlan = null;
            break;
        case DIRECT:
            newPose = p;
            newPlan = null;
            break;
        case HOME:
            p = homePose;
            state = PlanState.PLAN;
        case PLAN:
            newPose = p;
            newPlan = new GradientPlanner(currentScan.map, MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, (int)(MAP_SIZE_METERS*1000), (int)(MAP_SIZE_METERS*1000) );
            if (!newPlan.findPath(slamPose, p))
            {
                System.out.println("No path");
                newPlan = null;
                newPose = null;
                state = PlanState.NONE;
            }
            break;
        case EXPLORE:
            newPose = p;
            newPlan = new GradientPlanner(currentScan.map, MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, (int)(MAP_SIZE_METERS*1000), (int)(MAP_SIZE_METERS*1000) );
            List<Pose> path;
            if (!newPlan.explore(slamPose) || (path = newPlan.getPath(slamPose)).size() == 0)
            {
                System.out.println("No path");
                newPlan = null;
                newPose = null;
                state = PlanState.NONE;
            }
            else
            {
                newPose = path.get(path.size()-1);
            }
            break;
        }
        synchronized(this)
        {
            planState = state;
            plan = newPlan;
            targetPose = newPose;            
        }
        refreshDisplay();
    }
    
static long totalTime = 0;
    protected void runSlam()
    {
        // TODO Auto-generated method stub
        double totalDist=0;
        double maxDist = 0;
        while (active)
        {
            // wait for a scan
            synchronized(scanHistory)
            {
                while (processedScans >= rawScans)
                    try
                    {
                        scanHistory.wait();
                    } catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
            }
            long t = System.currentTimeMillis();
            //if (processedScans > 375) slam.map_quality = 1;
            ScanInfo info = scanHistory.get(processedScans);
            slam.update(info.rawScan, info.velocity);
            Position p = slam.getpos();
            Pose p2 = new Pose((float)p.x_mm, (float)p.y_mm, (float)p.theta_degrees);
            double[] processedScan = slam.scan_for_distance.getPoints();
            byte[] mapbytes = new byte [MAP_SIZE_PIXELS*MAP_SIZE_PIXELS];
            slam.getmap(mapbytes);
            synchronized(scanHistory)
            {
                info.update(processedScan, mapbytes, p2, slam.getDistance());
                processedScans++;
            }
            totalDist += info.distance;
            if (info.distance > maxDist)
                maxDist = info.distance;
            t = System.currentTimeMillis() - t;
            totalTime += t;
            System.out.println("frame " + (processedScans - 1) + " slam time " + t + " total " + totalTime);
            long allocatedMemory      = (Runtime.getRuntime().totalMemory()-Runtime.getRuntime().freeMemory());
            long presumableFreeMemory = Runtime.getRuntime().maxMemory() - allocatedMemory;
            //System.out.println("Processed " + processedScans + " memory " + presumableFreeMemory/(1024*1024));
            if (processedScans >= rawScans)
                System.out.printf("max dist %f avg dist %f\n", maxDist, totalDist/processedScans);
        }
        
    }

    protected void updateTracks(int scanNo)
    {
        map.clearTracks();
        for(int i = 0; i < ScanInfo.POSE_CNT; i++)
        {
            if (displayOptions[i]) 
                map.drawTrack(scanHistory, scanNo, i, trackColors[i], trackNames[i]);
        }
    }
    
    protected void updateScans(int scanNo)
    {
        for(int i = 0; i < ScanInfo.POSE_CNT; i++)
        {
            if (displayOptions[i])
            {
                if (displayOptions[SHOW_SCAN_HISTORY])
                {
                    for(int j = 0; j < scanNo; j++)
                        map.drawScan(scanHistory, laser.getOffsetMm(), j, i,  trackColors[i]);
                }
                else if (displayOptions[SHOW_SCANS])
                    map.drawScan(scanHistory, laser.getOffsetMm(), scanNo, i,  trackColors[i]);
                    
            }
        }
    }
    
    protected void updateMap(int scanNo)
    {
        map.drawMap(scanHistory, scanNo);
    }

    protected void updatePaths(int scanNo)
    {
        if (displayOptions[SHOW_PATH_PLANNING])
            if (plan == null)
                map.drawPaths(this, targetPose, null, null, null, trackColors[SHOW_PATH_PLANNING]);  
            else
                map.drawPaths(this, targetPose, plan.getPath(scanHistory.get(scanNo).poses[ScanInfo.POSE_SLAM]), plan.getCostMap(), plan.getFrontiers(), trackColors[SHOW_PATH_PLANNING]);        
        else
            map.clearPaths();
    }
    
    protected void updateAll()
    {
        //System.out.println("Paint");
        updateTracks(currentScanNo);
        updateScans(currentScanNo);
        updatePaths(currentScanNo);
        updateMap(currentScanNo);
        map.update();
        view.update();
        
    }
    
    protected void gotoScan(int scanNo)
    {
        if (scanNo < 0) scanNo = 0;
        if (scanNo >= processedScans) scanNo = processedScans - 1;
        currentScan = scanHistory.get(scanNo);
        slamPose = currentScan.poses[ScanInfo.POSE_SLAM];
        gyroPose = currentScan.poses[ScanInfo.POSE_GYRO];
        distance = currentScan.distance;
        currentScanNo = scanNo;
        refreshDisplay();
    }
    
    protected void step(int step)
    {
        if (planState == PlanState.PLAN || planState == PlanState.EXPLORE)
            setTargetPose(targetPose, planState);
        gotoScan(currentScanNo + step);
    }

    private void updateDisplay()
    {
        Boolean needUpdate = false;
        synchronized (this)
        {
            if (updateReq)
            {
                updateReq = false;
                reportedScans = processedScans;
                needUpdate = true;
            }
        }
        if (needUpdate)
                updateAll();
        map.syncDisplay();
    }
    
    private void waitForChange(long timeout)
    {
        synchronized(this)
        {
            map.syncDisplay();
            updateReq = updateReq || (reportedScans != processedScans);
            if (!updateReq)
                try
                {
                    this.wait(timeout);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            updateReq = updateReq || (reportedScans != processedScans);
            map.syncDisplay();
        }

    }
    
    protected void runPlayback()
    {
        // TODO Auto-generated method stub
        long start=0;
        int setPosCnt = 0;
        reportedScans = 0;

        while (active)
        { 
            switch(state)
            {
            case PLAY:
                if (currentScanNo + 1 >= processedScans)
                {
                    //Delay.msDelay(100);
                    break;
                }
                ScanInfo info = scanHistory.get(currentScanNo);
                long now = System.currentTimeMillis() - start;
                long delay = (info.timestamp - now*1000);
                if (delay > 0) break;
                step(1);
                break;
            case RUNPAUSE:
            case RUN:
                if (currentScanNo + 1 >= processedScans)
                {
                    //Delay.msDelay(100);
                    break;
                }
                step(1);
                break;

            case START:
                currentScanNo = -1;
                // fall through
            case STEP:
                step(1);
                setState(RunState.PAUSE);
                break;
            case END:
                currentScanNo = processedScans;
                // fall through
            case BACK:
                step(-1);
                setState(RunState.PAUSE);
                break;
            case FORWARD:
                step(5);
                if (currentScanNo >= processedScans -1)
                    setState(RunState.PAUSE);
                break;
            case REWIND:
                step(-5);
                if (currentScanNo <= 0)
                    setState(RunState.PAUSE);
                break;
            case SETPOS:
                if (pendingPos != currentScanNo)
                {
                    currentScanNo = pendingPos;
                    step(0);
                    setPosCnt = 0;
                }
                else if (setPosCnt++ > 0)
                {
                    step(0);
                    setState(RunState.PAUSE);
                }
                //System.out.println("pp " + pendingPos + " cs " + currentScanNo);
                break;
            case EXIT:
                // restart
                //slam.reset();
                //System.out.println("use detrministic slam");
                //slam = new DeterministicSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);
                break;
            case PAUSE:
                if (currentScanNo >= 0 && currentScanNo < scanHistory.size())
                    start = System.currentTimeMillis() - scanHistory.get(currentScanNo).timestamp/1000 ;
                else
                    start = System.currentTimeMillis();
                break;
                
            default:
                 break;
            }
            updateDisplay();
            waitForChange(100);
        }
        
    }

    public static void loadHistory(Track track, String filename)
    {
        //final int[] correction = {-1, 0, -3, -2, -4, -1, 0, 0, -2, -1, -1, -1, 2, 0, 0, 0, -5, -2, 3, 7, 8, 4, 0, 0, 4, 6, 6, 5, 3, 4, 4, 8, 10, 10, 6, 7, 8, 8, 7, 11, 10, 11, 10, 9, 13, 13, 8, 8, 7, 7, 8, 6, 6, 7, 8, 5, 7, 9, 10, 11, 14, 12, 11, 11, 11, 14, 13, 13, 13, 14, 14, 13, 15, 15, 12, 11, 12, 12, 12, 12, 11, 11, 9, 12, 10, 8, 8, 8, 9, 10, 10, 7, 5, 4, 4, 4, 3, 7, 1, 0, 2, 1, 4, 3, 3, 5, 4, 4, 7, 3, 4, 2, 4, 5, 6, 3, 6, 4, 3, 5, 5, 1, 2, 4, 5, 3, 2, 1, 0, 0, 0, 2, -2, -3, -3, -4, -4, -6, -7, -6, -6, -5, -5, -6, -5, -7, -4, -4, -6, -5, -8, -7, -7, -7, -6, -6, -7, -8, -6, -7, -6, -9, -6, -4, -2, -4, -5, -6, -3, -3, -1, -1, -3, -3, -6, -6, -5, -1, -3, -7, -7, -5, -5, -4, -2, -6, -6, -4, -5, -3, -5, -7, -3, -4, -3, -5, -6, -5, -5, -2, -4, -5, -6, -6, -5, -3, -3, -4, -5, -6, -3, 0, 0, 0, -1, -2, -3, -1, 0, -1, 0, 1, 0, -1, 0, -2, 0, 2, 0, 0, -2, -3, -5, -2, -1, 0, -2, -4, -4, -5, -5, -6, -5, -6, -5, -4, -4, -5, -7, -7, -1, 2, 1, -3, -6, -5, -4, 1, 4, 2, -3, -3, -3, 0, 4, 7, 0, -4, -1, 2, 7, 9, 7, 0, 0, 2, 8, 10, 8, 6, 1, 1, 2, 11, 11, 11, 7, 6, 4, 7, 7, 10, 9, 7, 5, 6, 7, 8, 8, 6, 3, 2, 2, 2, 2, 4, 1, 0, 0, 0, 1, 0, 1, 2, 1, -2, -2, -3, -1, -1, -3, -4, -4, -3, -3, -5, -5, -5, -8, -6, -8, -6, -7, -7, -6, -7, -6, -6, -5, -3, -4, -4, -4, -2, -2, -3, -6, -4, -4, -5, -3, -2, 1, 3, 1, 2, 0, -1, -2, -2};
        List<ScanInfo> scans = new ArrayList<ScanInfo>();
        BufferedReader input = null;
        Pose prevPose = null;
        long baseTimestamp = 0;
        long prevTimestamp=0;
        try
        {
            FileReader fstream = new FileReader(filename);
            input = new BufferedReader(fstream);
            int lineno = 0;
            while (true)
            {
                //System.out.println("Line " + lineno++);
                String line = input.readLine();
                if (line == null)
                {
                    break;
                }
                String [] toks = line.split(" +");
                long timestamp = Long.parseLong(toks[0]);
                if (baseTimestamp == 0)
                    baseTimestamp = timestamp;
                timestamp = timestamp - baseTimestamp;
                //Pose pose = new Pose((float)Double.parseDouble(toks[4])+576+2560, (float)Double.parseDouble(toks[5])+5120, (float)Double.parseDouble(toks[6]));
                Pose pose = new Pose((float)Double.parseDouble(toks[4]), (float)Double.parseDouble(toks[5]), (float)Double.parseDouble(toks[6]));
                Pose pose2 = new Pose((float)Double.parseDouble(toks[7]), (float)Double.parseDouble(toks[8]), (float)Double.parseDouble(toks[9]));

                int [] scan = new int [360];
                Pose[] poses = new Pose[360];
                for (int k=0; k<scan.length; ++k)
                {
                    scan[k] = Integer.parseInt(toks[k+24]);
                    if (scan[k] != 0)
                        //scan[k] = scan[k] - 95 - correction[k];
                        scan[k] = scan[k] - 95;
                    if (scan[k] < 0 || scan[k] > 20480)
                       scan[k] = 0;
                }
                /*
                for (int k=0; k<scan.length; ++k)
                {
                    poses[k] = new Pose(Float.parseFloat(toks[24+360 + k*3]), Float.parseFloat(toks[24+360 + k*3 + 1]), Float.parseFloat(toks[24+360 + k*3 + 2]));
                }
                */
                ScanInfo info = new ScanInfo(prevPose, prevTimestamp, pose, timestamp, scan, 0.0f);
                info.poses[ScanInfo.POSE_ODO] = pose2;
                //info.scanPoses = poses;
                scans.add(info);
                prevPose = pose;
                prevTimestamp = timestamp;
            }
            input.close();
        }
        catch (IOException e)
        {
            System.err.println("Error: " + e.getMessage());
        }
        RobotInfo.track = track;
        RobotInfo r = new RobotInfo(scans);
        Delay.msDelay(500);
        /*
        int []scanData = new int[360];
        int []cnts = new int[360];
        int []min = new int[360];
        int [] max = new int[360];
        for(int i = 0; i < 360; i++)
        {
            min[i] = Integer.MAX_VALUE;
            max[i] = Integer.MIN_VALUE;
        }
        for(int i = 0; i < scans.size(); i++)
        {
            ScanInfo si = scans.get(i);
            for(int j = 0; j < 360; j++)
            {
                if (si.rawScan[j] > 200 && si.rawScan[j] < 500)
                {
                    scanData[j] += si.rawScan[j];
                    cnts[j]++;
                    if (si.rawScan[j] > max[j])
                        max[j] = si.rawScan[j];
                    if (si.rawScan[j] < min[j])
                        min[j] = si.rawScan[j];
                }
            }
        }
        int avg = 0;
        int cnt = 0;
        for(int i = 0; i < 360; i++)
        {
            System.out.printf("%5d %6.1f %5d %5d %5d %5d\n", i, (double)scanData[i]/cnts[i], cnts[i], min[i], max[i], max[i] - min[i]);
            avg += scanData[i];
            cnt += cnts[i];
        }
        int base = (int)(((double)avg/cnt)+0.5);
        double a = (double)avg/cnt;
        System.out.println("avg " + (double)avg/cnt + " base " + base);
        for(int i = 0; i < 360; i++)
            System.out.printf(" %d,", (int)((double)scanData[i]/cnts[i] - a + 0.5));
        */
        r.setState(RunState.PAUSE);
    }
    
    public void refreshDisplay()
    {
        //System.out.println("Refresh");
        synchronized(this)
        {
            updateReq = true;
            notifyAll();
        }
    }
    
    public void setState(RunState s)
    {
        synchronized(this)
        {
            state = s;
            System.out.println("State " + state);
            refreshDisplay();
        }
    }
    
    public RunState getState()
    {
        return state;
    }
    
    public void setPlaybackPos(int pos)
    {
        if (state == RunState.RUN || state == RunState.RUNPAUSE) return;
        if (pos == currentScanNo) return;
        pendingPos = pos;
        System.out.println("pending " + pos + " current " + currentScanNo);
        setState(RunState.SETPOS);
    }
    
    public void setDisplayOption(int option, boolean state)
    {
        displayOptions[option] = state;
        refreshDisplay();
    }
    
    public boolean getDisplayOption(int option)
    {
        return displayOptions[option];
    }

    public static void waitForRobots(Track track)
    {
        System.out.println("Waiting for robots");
        RobotInfo.track = track;

        try
        {
            ServerSocket sock = new ServerSocket(2446);
            while (true)
            {
                Socket newClient = sock.accept();
                System.out.println("Got new client");
                new RobotInfo(newClient);
            }
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
}
