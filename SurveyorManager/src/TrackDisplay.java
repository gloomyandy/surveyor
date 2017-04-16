import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.IndexColorModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import lejos.robotics.navigation.Pose;

public class TrackDisplay extends ZoomAndPanPanel
{
    final static Color[] lineColors = {Color.RED, Color.GREEN, Color.BLUE};
    BufferedImage robotImage;
    BufferedImage trackImage;
    AffineTransform robotTransform;
    AffineTransform mapTransform;
    AffineTransform imgTransform;
    Graphics2D gImg;
    Graphics2D gMap;
    Graphics2D gTrack;
    int mapWidthmm;
    int mapHeightmm;
    int mapWidthPix;
    int mapHeightPix;
    BufferedImage IndexedImage;
    WritableRaster raster;
    IndexColorModel pathCostCM;
    BufferedImage pathCostIndexedImage;
    WritableRaster pathCostRaster;
    int updateCnt = 0;
    int paintCnt = 0;
    Point targetPos=null;
    boolean showDebug;
    RobotInfo robot;
    
    public TrackDisplay(int sizePix, double sizeM, int mapSize)
    {
        super(new BufferedImage(sizePix, sizePix, BufferedImage.TYPE_INT_ARGB));
        trackImage = new BufferedImage(sizePix, sizePix, BufferedImage.TYPE_INT_ARGB);
        // Create indexed image to convert the byte indexed map into an image
        byte [] red = new byte[256];
        byte [] green = new byte[256];
        byte [] blue = new byte[256];
        byte [] alpha = new byte[256];
        byte [] solidColor = new byte[256];
        byte [] none = new byte[256];
        for(int i = 0; i < 256; i++)
        {
            red[i] = green[i] = blue[i] = (byte)i;
            alpha[i] = (byte)((i)*85/100);
            solidColor[i] = (byte)255;
        }
        alpha[255] = (byte) 0;
        alpha[252] = (byte) 255;
        alpha[254] = (byte) (255*50/100);
        alpha[253] = (byte) 0;
        IndexColorModel cm = new IndexColorModel(8, 256, red, green, blue);
        IndexedImage = new BufferedImage(sizePix, sizePix, BufferedImage.TYPE_BYTE_INDEXED, cm);
        raster = IndexedImage.getRaster();
        pathCostCM = new IndexColorModel(8, 256, none, none, solidColor, alpha);
        pathCostIndexedImage = null;

        // Load the robot image      
        try
        {
            //setImage("track.jpg");
            robotImage = ImageIO.read(new File("robot.png"));
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        System.out.println("robot " + robotImage.getWidth(null));
        robotTransform = new AffineTransform();
        robotTransform.scale(1,  -1);
        robotTransform.translate(-robotImage.getWidth(null)/2, -robotImage.getHeight(null)/2);
        robotTransform.rotate(Math.toRadians(90), robotImage.getWidth(null)/2, robotImage.getHeight(null)/2); 
        gImg = (Graphics2D)img.getGraphics();
        gImg.setBackground(Color.WHITE);
        mapWidthmm = (int) (sizeM * 1000);
        mapHeightmm = (int) (sizeM * 1000);
        mapWidthPix = mapSize;
        mapHeightPix = mapSize;
        //mapTransform.scale((double)sizePix/(mapWidth), -((double)sizePix/(mapHeight)));
        gTrack = (Graphics2D)trackImage.getGraphics();
        gTrack.translate(0, sizePix);
        gTrack.scale((double)sizePix/(mapWidthmm), -((double)sizePix/(mapHeightmm)));
        gTrack.setBackground(new Color(0, 0, 0, 0));
        gTrack.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        gTrack.setStroke(new BasicStroke(8));
        mapTransform = new AffineTransform();
        mapTransform.translate(0, sizePix);
        mapTransform.scale((double)sizePix/(mapWidthPix), -((double)sizePix/(mapHeightPix)));
        gImg.clearRect(0, 0, imgrect.width, imgrect.height);
    }
    
    protected void drawTargetPos()
    {
        if (targetPos != null)
        {
            gImg.setColor(Color.BLACK);
            int x = targetPos.x * imgrect.width/mapWidthmm;
            int y = imgrect.height - targetPos.y * imgrect.height/mapHeightmm;
            gImg.fillArc(x-5, y-5, 10, 10, 0, 360);
        }
    }
    
    protected void paintTargetPos(Graphics2D g)
    {
        if (targetPos != null)
        {
            g.setColor(Color.BLACK);
            int w = getWidth();
            int h = getHeight();
            int imageWidth = imgrect.width;
            int imageHeight = imgrect.height;
            //System.out.println("width " + w + " height " + h);
            double xoff = (w - scale * imageWidth)/2;
            double yoff = (h - scale * imageHeight)/2;
            if (xoff < 0) xoff = 0;
            if (yoff < 0) yoff = 0;
            int x = targetPos.x * imgrect.width/mapWidthmm;
            int y = imgrect.height - targetPos.y * imgrect.height/mapHeightmm;
            x *= scale;
            y *= scale;
            x += xoff;
            y += yoff;
            g.fillArc(x-5, y-5, 10, 10, 0, 360);
        }
    }
    
    protected void drawRobot(Graphics2D g2d, String name, Pose p)
    {
        AffineTransform saved = g2d.getTransform();
        //System.out.println(name + ": " + p.getX() + ", " + p.getY() + " " + p.getHeading());
        g2d.translate(p.getX(), p.getY());
        g2d.rotate(Math.toRadians(p.getHeading()));
        //g2d.fillRect(-75, -55, 150, 110);
        g2d.drawImage(robotImage, robotTransform, null);
        g2d.rotate(Math.toRadians(270));
        g2d.setColor(Color.BLACK);
        g2d.scale(1, -1);
        g2d.setFont(g2d.getFont().deriveFont(Font.BOLD, 22.0f));
        g2d.drawString(name, -robotImage.getWidth(null)/2 + 33, -robotImage.getHeight(null)/8-4);
        g2d.setTransform(saved);
    }
    
    @Override
    public void paintComponent(Graphics g) {
        synchronized(img)
        {
            super.paintComponent(g);
            paintTargetPos((Graphics2D)g);
            paintCnt = updateCnt;
            //System.out.println("paint " + paintCnt);
        }
    }
    
    public void clearTracks()
    {
        gTrack.clearRect(0, 0, mapWidthmm, mapHeightmm);
    }
    
    public void drawTrack(List<ScanInfo> scans, int last, int pose, Color color, String lab)
    {
        gTrack.setColor(color);
        if (!scans.isEmpty())
        {
            Pose prev = scans.get(0).poses[pose];
            Pose lastValid = prev;
            for(int i = 1; i <= last; i++)
            {
                Pose cur = scans.get(i).poses[pose];
                if (prev != null & cur != null)
                {
                    gTrack.drawLine((int)prev.getX(), (int)prev.getY(), (int)cur.getX(), (int)cur.getY());
                    lastValid = cur;
                }
                prev = cur;
            }
            if (lastValid != null)
                drawRobot(lastValid, lab);
        }

    }
    
    public void drawScan(List<ScanInfo> scans, double offset, int last, int p, Color color)
    {
        ScanInfo info = scans.get(last);
        Pose pose = info.poses[p];
        double[] points = info.processedScan;
        AffineTransform saved = gTrack.getTransform();
        gTrack.setColor(color);

        gTrack.translate(pose.getX(), pose.getY());
        gTrack.rotate(Math.toRadians(pose.getHeading()));
        gTrack.translate(offset, 0);
        for(int i = 0; i < points.length/2; i++)
        {
            double x = points[i*2];
            double y = points[i*2 + 1];
            gTrack.fillRect((int)x, (int)y, 12, 12);
        }
        gTrack.setTransform(saved);
    }

    public void drawRobot(Pose pose, String label)
    {
        drawRobot(gTrack, label, pose);
    }
    
    public void drawMap(List<ScanInfo> scans, int last)
    {
        byte[] mapbytes = scans.get(last).map;
        raster.setDataElements(0, 0, mapWidthPix, mapHeightPix, mapbytes);
    }
    
    public void drawPaths(RobotInfo robot, Pose target, List<Pose> path, byte[] costs, List<Point> frontiers, Color color)
    {
        this.robot = robot;
        gTrack.setColor(color);
        int width = robotImage.getWidth();
        if (target != null)
        {
            gTrack.drawArc((int)target.getX() - width/2, (int)target.getY()-width/2, width, width, 0, 360);
            gTrack.drawLine((int)target.getX() - width/2, (int)target.getY()-width/2, (int)target.getX() + width/2, (int)target.getY()+width/2);
            gTrack.drawLine((int)target.getX() - width/2, (int)target.getY()+width/2, (int)target.getX() + width/2, (int)target.getY()-width/2);
        }
        if (frontiers != null)
            for(Point p : frontiers)
                gTrack.drawArc((int)p.x - width/2, (int)p.y-width/2, width, width, 0, 360);
        if (path != null && !path.isEmpty())
        {
            Pose prev = path.get(0);
            for(int i = 1; i < path.size(); i++)
            {
                Pose cur = path.get(i);
                if (prev != null & cur != null)
                {
                    gTrack.drawLine((int)prev.getX(), (int)prev.getY(), (int)cur.getX(), (int)cur.getY());
                }
                prev = cur;
            }
        }
        if (costs != null)
        {
            if (pathCostIndexedImage == null)
            {
                pathCostIndexedImage = new BufferedImage(mapWidthPix, mapWidthPix, BufferedImage.TYPE_BYTE_INDEXED, pathCostCM);
                pathCostRaster = pathCostIndexedImage.getRaster();
            }
            pathCostRaster.setDataElements(0, 0, mapWidthPix, mapHeightPix, costs);
        }
        else
            clearPaths();
    }
    
    public void clearPaths()
    {
        pathCostRaster = null;
        pathCostIndexedImage = null;
    }
    
    public void update()
    {
        synchronized (img)
        {
            gImg.clearRect(0, 0, imgrect.width, imgrect.height);
            gImg.drawRenderedImage(IndexedImage, mapTransform);
            if (pathCostIndexedImage != null)
                gImg.drawRenderedImage(pathCostIndexedImage, mapTransform);                
            gImg.drawRenderedImage(trackImage, imgTransform);
            updateCnt++;
            //System.out.println("update " + updateCnt);
            repaint();
        }
        
    }
    
    public void syncDisplay()
    {
        synchronized(img)
        {
            if (paintCnt != updateCnt)
                repaint();
        }
    }
    
    public void mouseClicked(Point p)
    {
        Point np = new Point((int)(p.x* ((double)mapWidthmm/imgrect.width)), (int)((imgrect.height - p.y)*((double)mapHeightmm/imgrect.height)));
        targetPos = np;
        if (robot != null && robot.plan != null)
            System.out.printf("x: %d + y: %d cost %d, dist %d map %d\n", np.x, np.y, (int)robot.plan.costMap[(np.y/10)*mapWidthPix+np.x/10] & 0xff, 
                    Integer.MAX_VALUE - robot.plan.costs[np.x/10][np.y/10], (int)robot.currentScan.map[(np.y/10)*mapWidthPix+np.x/10] & 0xff);
        //drawTargetPos();
        this.repaint();
    }
    
    public Point getTargetPosition()
    {
        return targetPos;
    }
    
    public void clearTargetPosition()
    {
        targetPos = null;
        this.repaint();
    }
    
}
