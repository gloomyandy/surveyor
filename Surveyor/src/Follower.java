import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.UnknownHostException;

import lejos.hardware.Audio;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;


public class Follower
{
    static class Delta implements SampleProvider
    {
        SampleProvider mode1, mode2;
        public Delta(SampleProvider mode1, SampleProvider mode2)
        {
            this.mode1 = mode1;
            this.mode2 = mode2;
            
        }

        @Override
        public int sampleSize()
        {
            // TODO Auto-generated method stub
            return 1;
        }

        @Override
        public void fetchSample(float[] sample, int offset)
        {
            // TODO Auto-generated method stub
            float m1;
            mode1.fetchSample(sample, offset);
            m1 = sample[offset];
            mode2.fetchSample(sample, offset);
            sample[offset] -= m1; 
        }
        
    }

    static void initIMU(BNO055 imu)
    {
        if(!imu.begin(BNO055.opmode_t.OPERATION_MODE_NDOF_FMC_OFF)) {
            /* There was a problem detecting the BNO055 ... check your connections */
            System.out.println("Ooops, no BNO055 detected ... Check your wiring!");
            System.exit(0);
        }

        Delay.msDelay(1000);

        /* Display the current temperature */
        System.out.println("Current Temperature: " + imu.getTemp() + " C");

        imu.setExtCrystalUse(true);
        Delay.msDelay(2000);
        double[] pos = new double[3]; // [x,y,z] position data
        BNO055.CalData cal;
        cal = imu.getCalibration();
        boolean calLoaded = false;
        try
        {
            FileInputStream is = new FileInputStream("caldata.dat");
            byte[] calData = new byte[22]; 
            is.read(calData);
            imu.setCalibrationData(calData);
            calLoaded = true;
        } catch (Exception e1)
        {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
        if (!calLoaded)
        {
            System.out.println("Fail to load calibration data");
            System.exit(0);
        }
        imu.setMode(BNO055.opmode_t.OPERATION_MODE_NDOF);
        //imu.setMode(BNO055.opmode_t.OPERATION_MODE_IMUPLUS);
        // Wait for data to stabilise
        /*
        for(int i = 0; i < 10; i++)
        {
            pos = imu.getVector(BNO055.vector_type_t.VECTOR_EULER);
            System.out.print("X: " + pos[0] + " Y: " + pos[1] + " Z: " + pos[2]);
            cal = imu.getCalibration();
            System.out.println(" CALIBRATION: Sys=" + cal.sys + " Gyro=" + cal.gyro + " Accel=" + cal.accel + " Mag=" + cal.mag);
            Delay.msDelay(1000);
        }
        */
        System.out.println("Change mode");
        imu.setMode(BNO055.opmode_t.OPERATION_MODE_CONFIG);
        System.out.println("Config mode");
        Delay.msDelay(1000);
        imu.setMode(BNO055.opmode_t.OPERATION_MODE_IMUPLUS);
        // Wait for data to stabilise
        
        for(int i = 0; i < 3; i++)
        {
            pos = imu.getVector(BNO055.vector_type_t.VECTOR_EULER);
            System.out.print("X: " + pos[0] + " Y: " + pos[1] + " Z: " + pos[2]);
            cal = imu.getCalibration();
            System.out.println(" CALIBRATION: Sys=" + cal.sys + " Gyro=" + cal.gyro + " Accel=" + cal.accel + " Mag=" + cal.mag);
            Delay.msDelay(1000);
        }
        
        
    }
    
    static void saveIMUCalibration(BNO055 imu)
    {
        byte[] calData = imu.getCalibrationData();
        try {
        FileOutputStream fs = new FileOutputStream("caldata.dat");
        fs.write(calData);
        fs.close();
        } catch (Exception e)
        {
            System.out.println("Exception " + e);
        }
    }
    
    
    public static void main(String[] args)
    {
        Brick brick = BrickFinder.getLocal();
        RegulatedMotor left = new EV3LargeRegulatedMotor(brick.getPort("A"));
        RegulatedMotor right = new EV3LargeRegulatedMotor(brick.getPort("D"));
        //LineFollowingMoveController pilot = new NewPilot(56.0, 100.0, left, right);
        //LineFollowerChassis chassis = new LineFollowerChassis(new Wheel[] { DifferentialChassis.modelWheel(left, 55.0).offset(52.0), 
          //      DifferentialChassis.modelWheel(right, 55.0).offset(-52.0)});
        LineFollowerChassis chassis;
        MapTracker tracker;
        LineFollower follower;
        EV3IRSensor ir = new EV3IRSensor(brick.getPort("S3"));
        BNO055 imu = new BNO055(brick.getPort("S1"));
        initIMU(imu);
        chassis = new LineFollowerChassis(new Wheel[] { WheeledChassis.modelWheel(left, 56.0).offset(49.0), 
              WheeledChassis.modelWheel(right, 56.0).offset(-49.0)}, imu);
        tracker = new MapTracker(chassis);
        //follower = new LineFollower(chassis, tracker, 35, 5, 15);
        follower = new LineFollower(chassis, tracker, 50f, 0f, 0f);


        
        Pose pose = new Pose();
        pose.setHeading(0.0f);
        pose.setLocation(2560, 5120);
        StatusTracker status = null;
        try
        {
            status = new StatusTracker(chassis,  follower, pose, BrickFinder.getLocal().getName(), "192.168.0.9", brick.getPower(), ir.getDistanceMode(), tracker);
        } catch (UnknownHostException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        follower.calibrate();
        status.setPose(pose);                
        Lidar lidar = null;
        try
        {
            lidar = new Lidar(brick.getPort("C"), brick.getPort("S2"), left, right, chassis.getIMUPoseProvider(), chassis.getPoseProvider(), "log.dat", status);
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        //initIMU(imu);
        //follower.calibrate();
        //status.setPose(pose);                
        Delay.msDelay(1000);
        follower.setState(LineFollower.FollowState.PAUSED);
        Audio audio = BrickFinder.getLocal().getAudio();
        //pose.setLocation(0, 0);
        //follower.setSpeed(100);
        //tracker.setHeading(180);
        while(follower.isRunning())
        {
            Delay.msDelay(50);
            /*
            if (loopcnt++ % 20 == 0)
            {
                BNO055.CalData cal = imu.getCalibration();
                System.out.println("CALIBRATION: Sys=" + cal.sys + " Gyro=" + cal.gyro + " Accel=" + cal.accel + " Mag=" + cal.mag);

            }
            */
        }
        saveIMUCalibration(imu);
        System.out.println("waiting for chassis to stop");
        while(!follower.isStopped() )
            Delay.msDelay(1);
        tracker.close();
        System.out.println("Tracker");
        lidar.close();
        System.out.println("Lidar");
        left.close();
        System.out.println("Left");
        right.close();
        System.out.println("Right");
        status.close();
        System.out.println("socket");
    }

}
