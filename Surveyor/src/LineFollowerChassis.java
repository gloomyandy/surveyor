import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;
import lejos.utility.Matrix;
import lejos.hardware.motor.BaseRegulatedMotor;


public class LineFollowerChassis extends WheeledChassis
{
    protected double baseDistance = 0;
    Wheel[] wheels;
    RegulatedMotor[] motors;
    Matrix reverse2;
    Matrix forward2;
    BNO055 imu;
    
    public LineFollowerChassis(Wheel[] wheels)
    {
        super(wheels, TYPE_DIFFERENTIAL);
        forward2 = forward.copy();
        reverse2 = forward2.inverse();
        if (reverse2 == null)
            System.out.println("Reverse2 null");
        else
            System.out.println("Reverse2 created");
        this.wheels = wheels;
        motors = new RegulatedMotor[wheels.length];
        for(int i = 0; i < wheels.length; i++)
        {
            motors[i] = wheels[i].getMotor();
        }
        // TODO Auto-generated constructor stub
    }
    
    public LineFollowerChassis(Wheel[] wheels, BNO055 imu)
    {
        super(wheels, TYPE_DIFFERENTIAL);
        this.imu = imu;
        this.wheels = wheels;
        forward2 = forward.copy();
        reverse2 = forward2.inverse();
        if (reverse2 == null)
            System.out.println("Reverse2 null");
        else
            System.out.println("Reverse2 created");
        motors = new RegulatedMotor[wheels.length];
        for(int i = 0; i < wheels.length; i++)
        {
            motors[i] = wheels[i].getMotor();
        }
        // TODO Auto-generated constructor stub
    }
    

    protected synchronized Matrix getPosition() {
        Matrix x = new Matrix(motors.length+dummyWheels, 1);
        master.startSynchronization();
        for (int i = 0; i < motors.length; i++) {
            x.set(i, 0, ((BaseRegulatedMotor)motor[i]).getPosition());
        }
        if (dummyWheels==1) x.set(motors.length, 0, 0);
        master.endSynchronization();
        return x;
      }

    private class IMUOdometer implements PoseProvider {
        Matrix lastTacho;
        double xPose, yPose, aPose, aBase;
        int    time = 64;

        private IMUOdometer() {
          lastTacho = getAttribute(TACHOCOUNT);
          PoseTracker tracker = new PoseTracker();
          tracker.setDaemon(true);
          tracker.start();
        }

        private double getYaw()
        {
            return 360 - imu.getVector(BNO055.vector_type_t.VECTOR_EULER)[0];
        }
        
        @Override
        public synchronized Pose getPose() {
          return new Pose((float) xPose, (float) yPose, (float) aPose);
        }

        @Override
        public synchronized void setPose(Pose pose) {
          xPose = pose.getX();
          yPose = pose.getY();
          aPose = pose.getHeading();
          aBase = getYaw() + aPose;
        }

        private int cnt = 0;
        private synchronized void updatePoseAvg() {
            double newPose = getYaw() - aBase;
            //Matrix currentTacho = getAttribute(TACHOCOUNT);
            Matrix currentTacho = getPosition();
            Matrix delta = currentTacho.minus(lastTacho);
            
            while (newPose < -180)
                newPose += 360;
              while (newPose > 180)
                newPose -= 360;
            double diff =  ( ( (newPose + 180) - (aPose + 180) + 180 + 360 ) % 360 ) - 180;
            double avPose = (360 + (newPose + 180) + ( diff / 2 ) ) % 360 - 180;
            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            //double sin = Math.sin(Math.toRadians(aPose));
            //double cos = Math.cos(Math.toRadians(aPose));
            double sin = Math.sin(Math.toRadians(avPose));
            double cos = Math.cos(Math.toRadians(avPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);
            
            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            //aPose += delta.get(2, 0);
            
            //aPose = getYaw() - aBase;
            aPose = newPose;
            /*
            while (aPose < -180)
              aPose += 360;
            while (aPose > 180)
              aPose -= 360;
            */
            // adjust loop speed (between 4 and 64 msec);
            long prevTime = time;
            if (max > 20) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            time = 64;
            //if (cnt++ % 100 == 0)
              //  System.out.println("max is " + max);
            if (prevTime != time)
                System.out.println("Set time to " + time);
            lastTacho = currentTacho;
          }
        private synchronized void updatePose() {
            Matrix currentTacho = getAttribute(TACHOCOUNT);
            double newPose = getYaw() - aBase;
            Matrix delta = currentTacho.minus(lastTacho);
            
            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            double sin = Math.sin(Math.toRadians(aPose));
            double cos = Math.cos(Math.toRadians(aPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);
            
            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            //aPose += delta.get(2, 0);
            
            //aPose = getYaw() - aBase;
            aPose = newPose;
            
            while (aPose < -180)
              aPose += 360;
            while (aPose > 180)
              aPose -= 360;
            
            // adjust loop speed (between 4 and 64 msec);
            if (max > 10) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            lastTacho = currentTacho;
          }
        private synchronized void updatePosePos() {
            Matrix currentTacho = getAttribute(TACHOCOUNT);
            //double newPose = getYaw() - aBase;
            Matrix delta = currentTacho.minus(lastTacho);
            
            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            double sin = Math.sin(Math.toRadians(aPose));
            double cos = Math.cos(Math.toRadians(aPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);
            
            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            aPose += delta.get(2, 0);
            
            //aPose = getYaw() - aBase;
            //aPose = newPose;
            
            while (aPose < -180)
              aPose += 360;
            while (aPose > 180)
              aPose -= 360;
            
            // adjust loop speed (between 4 and 64 msec);
            if (max > 10) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            time = 64;
            lastTacho = currentTacho;
          }

        private class PoseTracker extends Thread {
          public void run() {
            while (true) {
                //updatePosePos();
                updatePoseAvg();
              Delay.msDelay(time);
            }
          }
        }
      }


    private IMUOdometer IMUodometer = null;
   
    public PoseProvider getIMUPoseProvider() {
        if (IMUodometer == null) IMUodometer = new IMUOdometer();
        return  IMUodometer;
      }

    public void setWheelParams(double diameter, double offset)
    {
        for (int row = 0; row < wheels.length; row++) {
            Matrix factors = wheels[row].getFactors();
            factors.set(0, 2, -(2.0 * (row > 0 ? -offset : offset) / diameter));
            forward2.setMatrix(row, row, 0, 2, factors);
          }
        reverse2 = forward2.inverse();
    }
    
    
    private Odometer odometer = null;

    @Override
    public PoseProvider getPoseProvider() {
      if (odometer == null) odometer = new Odometer();
      return  odometer;
    }
    
    /** The odometer keeps track of the robot pose based on odometry using the encoders of the regulated motors of the wheels.
     * @author Aswin Bouwmeester
     *
     */
    private class Odometer implements PoseProvider {
      Matrix lastTacho;
      double xPose, yPose, aPose;

      int    time = 64;

      private Odometer() {
        lastTacho = getAttribute(TACHOCOUNT);
        PoseTracker tracker = new PoseTracker();
        tracker.setDaemon(true);
        tracker.start();
      }

      @Override
      public Pose getPose() {
        return new Pose((float) xPose, (float) yPose, (float) aPose);
      }

      @Override
      public synchronized void setPose(Pose pose) {
        xPose = pose.getX();
        yPose = pose.getY();
        aPose = pose.getHeading();
      }

      private synchronized void updatePose() {
        Matrix currentTacho = getAttribute(TACHOCOUNT);
        Matrix delta = currentTacho.minus(lastTacho);

        int max = (int) getMax(delta);
        delta = reverse2.times(delta);
        double newPose = aPose + delta.get(2, 0);
        while (newPose < -180)
            newPose += 360;
          while (newPose > 180)
            newPose -= 360;
        double diff =  ( ( (newPose + 180) - (aPose + 180) + 180 + 360 ) % 360 ) - 180;
        double avPose = (360 + (newPose + 180) + ( diff / 2 ) ) % 360 - 180;
        //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
        //double sin = Math.sin(Math.toRadians(aPose));
        //double cos = Math.cos(Math.toRadians(aPose));
        double sin = Math.sin(Math.toRadians(avPose));
        double cos = Math.cos(Math.toRadians(avPose));

        /*
        delta = reverse2.times(delta);
        double sin = Math.sin(Math.toRadians(aPose));
        double cos = Math.cos(Math.toRadians(aPose));
        */
        double x = delta.get(0, 0);
        double y = delta.get(1, 0);
        
        xPose += cos * x - sin * y;
        yPose += sin * x + cos * y;
        aPose = newPose;
        /*
        aPose += delta.get(2, 0);
        while (aPose < -180)
          aPose += 360;
        while (aPose > 180)
          aPose -= 360;
        */
        // adjust loop speed (between 4 and 64 msec);
        if (max > 10) time=time / 2;
        if (max < 10) time=time * 2;
        time = Math.max(Math.min(time, 64), 4);
        lastTacho = currentTacho;
      }

      private class PoseTracker extends Thread {
        public void run() {
          while (true) {
            updatePose();
            Delay.msDelay(time);
          }
        }
      }
    }
}
