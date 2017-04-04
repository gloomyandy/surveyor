import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import lejos.utility.Delay;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
//import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
//import org.opencv.videoio.VideoCapture;
//import org.opencv.videoio.Videoio;

class ImageServer implements Runnable {
    Mat[] images = new Mat[0];
    
    public void setImage(int index, Mat img)
    {
        if (images.length <= index)
        {
            Mat[] newImages = new Mat[index+1];
            System.arraycopy(images, 0, newImages, 0, images.length);
            images = newImages;
        }
        images[index] = img;
    }
    public ImageServer()
    {
        Thread server = new Thread(this);
        server.setDaemon(true);
        server.start();
       
    }
    public void run() {
        ServerSocket ss = null;
        Socket sock = null;
        try {
            ss = new ServerSocket(8080);
            sock = ss.accept();
            String boundary = "Thats it folks!";
            writeHeader(sock.getOutputStream(), boundary);
            
            int cnt = 0;
            while (true) {
                for(Mat mat : images)
                {
                    if (mat != null)
                        writeJpg(sock.getOutputStream(), mat, boundary);
                    //Delay.msDelay(1000);
                }
                    
            }
        } 
        catch (IOException e)
        {
            System.out.println("Got exception " + e);
        }
        finally
        {
            try {
                if (sock != null) sock.close();
                if (ss != null) ss.close();
            } catch (IOException e)
            {
                System.out.println("Got exception during close " + e);
            }
        }
        
    }
     
    private static void writeHeader(OutputStream stream, String boundary) throws IOException {
        stream.write(("HTTP/1.0 200 OK\r\n" +
                "Connection: close\r\n" +
                "Max-Age: 0\r\n" +
                "Expires: 0\r\n" +
                "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" +
                "Pragma: no-cache\r\n" + 
                "Content-Type: multipart/x-mixed-replace; " +
                "boundary=" + boundary + "\r\n" +
                "\r\n" +
                "--" + boundary + "\r\n").getBytes());
    }
     
    private static void writeJpg(OutputStream stream, Mat img, String boundary) throws IOException {
        //ByteArrayOutputStream baos = new ByteArrayOutputStream();
        //ImageIO.write(img, "jpg", baos);
        MatOfByte buf = new MatOfByte();
        Highgui.imencode(".jpg", img, buf);
        //Imgcodecs.imencode(".jpg", img, buf);
        byte[] imageBytes = buf.toArray();
        stream.write(("Content-type: image/jpeg\r\n" +
                "Content-Length: " + imageBytes.length + "\r\n" +
                "\r\n").getBytes());
        stream.write(imageBytes);
        stream.write(("\r\n--" + boundary + "\r\n").getBytes());
    }

}

public class OpenCVLineTracker implements Runnable, LineTracker
{
    VideoCapture vid;
    ImageServer debug;
    int threshold;
    double area;
    float sample = 0.0f;
    boolean active = false;
    Mat mono = new Mat();
    Mat blur = new Mat();
    Mat thresh = new Mat();
    Mat roiImage;
    Mat roiImage2;
    Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    Mat erodeImg = new Mat();
    Mat dilateImg = new Mat();
    Mat debugImg = new Mat();
    Mat notused = new Mat();
    boolean markerFound = false;
    boolean inMarker = false;
    int trackEdge = EDGE_LEFT;
    boolean debugImage = false;
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
    
    public void Init()
    {
        vid = new VideoCapture(0);
        vid.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 160);
        vid.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 120);        
        //vid.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, 160);
        //vid.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, 120);        
        //vid.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, 320);
        //vid.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, 240);        
        //vid.set(6, new FourCC("YUYV").toInt());
        //vid.set(6, new FourCC("MJPG").toInt());
        vid.open(0);
    }
    
    public OpenCVLineTracker()
    {
        this.debug = new ImageServer();
        Init();
        Thread tracker = new Thread(this);
        active = true;
        tracker.setDaemon(true);
        tracker.start();
    }

    private float getMidPoint(Mat roi, int bias)
    {
        Imgproc.cvtColor(roi, mono, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(mono, blur, new Size(9, 9), 2, 2);
        double threshRet = Imgproc.threshold(blur, thresh, 0, 255, Imgproc.THRESH_BINARY_INV|Imgproc.THRESH_OTSU);
        //Imgproc.erode(thresh, erodeImg, erode);
        Imgproc.erode(thresh, erodeImg, erode);
        Imgproc.dilate(erodeImg, dilateImg, dilate);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(dilateImg, contours, notused, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        double minMaxCx = (bias <= 0 ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY);
        Core.rectangle(roi, new Point(0, 0), new Point(roi.cols()-1, roi.rows()-1), new Scalar(0, 255, 0));
        //System.out.println("Threshold " + threshRet + " contours " + contours.size());
        for(MatOfPoint cont : contours)
        {
            Moments mu = Imgproc.moments(cont, false);
            //double cy = (mu.get_m01()/mu.get_m00());
            if (mu.get_m00() > 100.0)
            {
                //double cx = (mu.get_m10()/mu.get_m00());
                Rect r = Imgproc.boundingRect(cont);
                Core.rectangle(roi, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height), new Scalar(0, 255, 255));
                //System.out.println("  Area " + mu.get_m00() + " centre " + (mu.get_m10()/mu.get_m00()) + " left " + r.x + " width " + r.width);
                if (r.width > 95)
                {
                    System.out.println("  Area " + mu.get_m00() + " centre " + (mu.get_m10()/mu.get_m00()) + " left " + r.x + " width " + r.width);
                    if (!inMarker)
                    {
                        markerFound = true;
                        inMarker = true;
                    }
                    //minMaxCx = roi.cols()/2;
                    break;
                }
                else
                    inMarker = false;
                double cx = r.x;
                if (bias <= 0)
                {
                    cx = r.x + 12;
                    if (cx < minMaxCx)
                        minMaxCx = cx;
                }
                else 
                {
                    cx = r.x + r.width - 12;
                    if (cx > minMaxCx)
                        minMaxCx = cx;
                }
            }
        }
        if (Double.isInfinite(minMaxCx))
            minMaxCx = roi.cols()/2;
        else
            Core.circle(roi, new Point(minMaxCx + bias*12, roi.rows()/2), 5, new Scalar(0, 0, 255));
        Core.circle(roi, new Point(minMaxCx, roi.rows()/2), 5, new Scalar(255, 0, 0));
        Core.circle(roi, new Point(minMaxCx, roi.rows()/2), 3, new Scalar(0, 0, 0));
        return 1.0f - 2.0f*(float)minMaxCx/roi.cols();
    }
    
    
    @Override
    public void run()
    {
        // TODO Auto-generated method stub
        Mat camImage = new Mat();
        System.out.println("Tracker running");
        vid.read(camImage);
        System.out.println("got image w " + camImage.cols() + " h " + camImage.rows());
        Mat roiImage = new Mat(camImage, new Rect(10, 2*camImage.rows()/3, camImage.cols()-20, camImage.rows()/12));
        debug.setImage(0, debugImg);
        //debug.setImage(1, roiImage);
        //debug.setImage(2,  mono);
        //debug.setImage(3,  blur); 
        //debug.setImage(4,  thresh);
        //debug.setImage(5, erodeImg );
        //debug.setImage(6, dilateImg );
        int cnt = 0;
        long totalTime = 0;
        int glitchCnt = 0;
        while (active)
        {
            long start = System.currentTimeMillis();
            vid.read(camImage);
            /*
            Imgproc.cvtColor(camImage, mono, Imgproc.COLOR_BGR2GRAY);
            Imgproc.GaussianBlur(mono, blur, new Size(9, 9), 2, 2);
            //Imgproc.threshold(blur, thresh, threshold, 255, 1);
            Imgproc.threshold(blur, thresh, 0, 255, Imgproc.THRESH_BINARY_INV|Imgproc.THRESH_OTSU);
            Imgproc.erode(thresh, erodeImg, erode);
            Imgproc.dilate(erodeImg, dilateImg, dilate);
            */
            //debug.setImage(0, camImage);
            //debug.setImage(1, roiImage);
            float ret = getMidPoint(roiImage, trackEdge);
            if (Math.abs(ret - sample) > 0.5 && glitchCnt <= 1)
            {
                //Sound.buzz();
                //camImage.copyTo(debugImg); 
                //Delay.msDelay(100000);
                System.out.println("Big delta old " + sample + " new " + ret);
                glitchCnt++;
                markerFound = inMarker = false;
            }
            else
            {
               sample = ret;
               glitchCnt = 0;
            }
            //if (markerFound && !debugImage)
                //camImage.copyTo(debugImg);
            totalTime += System.currentTimeMillis() - start;
            camImage.copyTo(debugImg);
            /*
            if (++cnt >= 10)
            {
                System.out.println("Time: " + (float)totalTime/cnt);
                totalTime = 0;
                cnt = 0;
            }*/
        }
    }
    
    public void calibrate()
    {
    }

    @Override
    public int sampleSize()
    {
        return 1;
    }

    @Override
    public void fetchSample(float[] sample, int offset)
    {
        // TODO Auto-generated method stub
        //System.out.println("Sample " + this.sample);
        sample[offset] = this.sample;
    }

    @Override
    public boolean startCalibration()
    {
        // TODO Auto-generated method stub
        Delay.msDelay(2000);
        return false;

    }

    @Override
    public void stopCalibration()
    {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean marker()
    {
        boolean ret = markerFound;
        markerFound = false;
        return ret;
    }

    @Override
    public void close()
    {
        // TODO Auto-generated method stub
        active = false;
    }

    @Override
    public void setEdge(int edge)
    {
        trackEdge = edge;
        debugImage = false;
    }


}
