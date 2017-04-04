import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.I2CPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.I2CSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;


public class Lidar
{
    final byte ADDRESS = (byte)(0x62 << 1);
    final byte REG_DATA = (byte)0x8f;
    final byte REG_CMD = 0;
    final byte REG_SAMPLE = 0x2;
    final byte CMD_MEASURE = 0x4;
    final int COG1 = 24;
    final int COG2 = 56;
    protected boolean running;
    protected RegulatedMotor motor, left, right;
    protected PoseProvider robotPose;
    protected PoseProvider robotPose2;
    protected PrintWriter log;
    protected I2CSensor laser;
    protected LidarScan completedScan;
    protected int error = 0;
    protected int retry = 0;
    protected int missed = 0;
    protected int scanCnt = 0;
    protected StatusTracker status;
    
    static public class LidarScan
    {
        int[] ranges = new int[360];
        Pose[] poses = new Pose[360];
        long timestamp;
        Pose pose;
        Pose pose2;
        int leftTacho, rightTacho;
    }
    
    public Lidar(Port motorPort, Port sensorPort, RegulatedMotor left, RegulatedMotor right, PoseProvider robotPose, PoseProvider pose2, String logName, StatusTracker status) throws IOException
    {
        System.out.println("Starting Lidar");
        log = new PrintWriter(new FileWriter(logName));
        motor = new EV3LargeRegulatedMotor(motorPort);
        laser = new I2CSensor(sensorPort, ADDRESS, I2CPort.TYPE_HIGHSPEED);
        this.left = left;
        this.right = right;
        this.robotPose = robotPose;
        this.robotPose2 = pose2;
        this.status = status;
        running = true;
        Thread scanThread = new Thread(
                new Runnable() {
                    public void run() {
                        runCaptureScans();
                    }
                }
            );
        Thread processThread = new Thread(
                new Runnable() {
                    public void run() {
                        runProcessScans();
                    }
                }
            );
        scanThread.setDaemon(true);
        scanThread.setPriority(Thread.MAX_PRIORITY);
        processThread.setDaemon(true);
        processThread.start();
        scanThread.start();
    }


    protected void scanComplete(LidarScan scan)
    {
        synchronized(this)
        {
            if (completedScan != null)
                System.err.println("Lidar data overrun");
            completedScan = scan;
            scan.timestamp = System.currentTimeMillis();
            scan.pose = robotPose.getPose();
            scan.leftTacho = left.getTachoCount();
            scan.rightTacho = right.getTachoCount();
            scan.pose2 = robotPose2.getPose();
            notifyAll();
        }        
    }
    
    protected LidarScan waitForScan()
    {
        synchronized(this)
        {
            while (completedScan == null)
                try
                {
                    wait();
                } catch (InterruptedException e)
                {
                    break;
                }
            LidarScan ret = completedScan;
            completedScan = null;
            return ret;
        }
    }
    
    protected void runCaptureScans()
    {
        byte[] data = new byte[16];

        System.out.println("Lidar capture thread running");
        motor.setAcceleration(2000);
        motor.setSpeed(180*COG2/COG1);
        motor.forward();
        laser.setRetryCount(500);
        laser.sendData(REG_SAMPLE, (byte)0x10);
        int pos = -1;
        LidarScan scan = new LidarScan();

        laser.sendData(REG_CMD, CMD_MEASURE);
        while(true)
        {
            int newPos = (motor.getTachoCount()*COG1/COG2) % 360;
            if (pos != newPos)
            {
                laser.getData(REG_DATA, data, 2);
                //laser.sendData(REG_CMD, CMD_MEASURE);
                laser.sendData(REG_CMD, (byte)0x3);
                int range = (((int)data[0] << 8) | ((int)data[1] & 0xff));
                if (range < 0 || range > 5000)
                {
                    retry++;
                    range = 0;
                    laser.getData(REG_DATA, data, 2);
                    int range2 = (((int)data[0] << 8) | ((int)data[1] & 0xff));
                    if (range2 > 0 && range2 < 5000)
                        range = range2;
                    else
                        error++;
                }
                if ((pos + 1) % 360 != newPos) missed++;
                if (newPos == 359 || (newPos < pos && pos != 359))
                {
                    if (newPos == 359)
                    {
                        scan.ranges[newPos] = range;
                        //scan.poses[newPos] = robotPose.getPose();
                    }
                    //else
                        //System.err.println("Missed end of scan pos " + pos + " newPos " + newPos);
                    if (!running) break;
                    scanComplete(scan);
                    scan = new LidarScan();
                    if (newPos != 359)
                    {
                        scan.ranges[newPos] = range;
                        //scan.poses[newPos] = robotPose.getPose();
                    }
                }
                else
                {
                    scan.ranges[newPos] = range;
                    //scan.poses[newPos] = robotPose.getPose();
                }
                pos = newPos;
            }
            else
                Delay.msDelay(1);
        }
        scanComplete(scan);
        motor.stop();
        pos = motor.getTachoCount()*COG1/COG2;
        motor.rotateTo((((pos+359)/360)*360*COG2/COG1));
        //pos = motor.getTachoCount();
        //motor.rotateTo((((pos+359)/360)*360));
    }
    
    protected void runProcessScans()
    {
        System.out.println("Lidar process thread running");
        while (running)
        {
            LidarScan scan = waitForScan();
            if (scan == null)
            {
                System.err.println("Missing lidar scan");
                break;
            }
            log.print(scan.timestamp*1000 + " " + scanCnt++ + " " + scan.leftTacho + " " + scan.rightTacho + " " + 
                    scan.pose.getX() + " " + scan.pose.getY() + " " + scan.pose.getHeading() + " " +
                    scan.pose2.getX() + " " + scan.pose2.getY() + " " + scan.pose2.getHeading() + 
                        " 0 0 0 0 0 0 0 0 0 0 0 0 0 0");
            int zero = 0;
            for(int i = 0; i < scan.ranges.length; i++)
            {
                log.print(" " + scan.ranges[i]*10);
                if (scan.ranges[i] == 0) zero++;
            }
            /*
            for(int i = 0; i < scan.ranges.length; i++)
            {
               if (scan.poses[i] != null)
                    log.print(" " + scan.poses[i].getX() + " " + scan.poses[i].getY() + " " + scan.poses[i].getHeading());
                else
                    log.print(" 0.0 0.0 0.0");
            }
            */
            log.println();
            //System.out.println("scan " + scanCnt + " missed " + missed + " retry " + retry + " error " + error + " zero " + zero);
            status.setScan(scan);
        }
    }

    /*
    @Override
    public void run()
    {
        byte[] cmd = new byte[16];
        byte[] data = new byte[16];

        System.out.println("Lidar thread running");
        //Sound.beepSequenceUp();
        motor.setSpeed(75*COG2/COG1);
        motor.backward();
        laser.setRetryCount(500);
        //int pos = (-reg.getTachoCount()*COG1/COG2) % 360;
        int pos = -1;
        int nextPos = 0;
        int [] ranges = new int[360];
        int error = 0;
        int retry = 0;

        log.print(System.currentTimeMillis()*1000 + " 0 " + left.getTachoCount() + " " + right.getTachoCount() + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0");
        laser.sendData(REG_CMD, CMD_MEASURE);
        while(true)
        {
            int newPos = (-motor.getTachoCount()*COG1/COG2) % 360;
            if (pos != newPos)
            {
                laser.getData(REG_DATA, data, 2);
                laser.sendData(REG_CMD, CMD_MEASURE);
                int range = (((int)data[0] << 8) | ((int)data[1] & 0xff));
                if (range > 0 && range < 5000)
                {
                    ranges[newPos] = range;
                    for(int j=nextPos; j < newPos; j++)
                        log.print(" 0");
                    log.print(" " + range*10);
                    nextPos = newPos+1;
                }
                else
                {
                    //System.out.println("range = " + range + " pos " + newPos);
                    retry++;
                    laser.getData(REG_DATA, data, 2);
                    int range2 = (((int)data[0] << 8) | ((int)data[1] & 0xff));
                    if (range2 > 0 && range2 < 5000)
                        ranges[newPos] = range2;
                    else
                        error++;
                    System.err.println("range = " + range + "range2 = " + range2 +" pos " + newPos);
                    
                }

                if (newPos == 359)
                {
                    int missed = 0;
                    for(int i = 0; i < ranges.length; i++)
                        if (ranges[i] == 0)
                        {
                            //ranges[i] = ranges[(i-1) % 360];
                            missed++;
                        }
                    System.err.println("Missed " + missed + " error " + error + " retry " + retry);
                    for(int i = 0; i < ranges.length; i++)
                    {
                        //System.out.print(" " +ranges[i]*10);
                        ranges[i] = 0;
                    }
                    log.println();
                    if (!running) break;
                    log.print(System.currentTimeMillis()*1000 + " 0 " + left.getTachoCount() + " " + right.getTachoCount() + " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0");
                    nextPos = 0;
                }
                pos = newPos;
                //s.sendData(REG_CMD, CMD_MEASURE);
            }
            else
                Delay.msDelay(1);
        }
        pos = -motor.getTachoCount()*COG1/COG2;
        motor.rotateTo(-(((pos+359)/360)*360*COG2/COG1));
        //reg.stop();
    }
    */
    
    public void close()
    {
        running = false;
        motor.waitComplete();
        Delay.msDelay(5000);
        System.out.println("taco " + motor.getTachoCount() + " mod " + (motor.getTachoCount() % 360));
        laser.close();
        motor.close();
        log.close();
    }

}
