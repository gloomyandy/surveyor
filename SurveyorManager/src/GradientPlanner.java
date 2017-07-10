import java.awt.Point;
import java.awt.Rectangle;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.PriorityQueue;

import lejos.robotics.navigation.Pose;

class State
{
    int x;
    int y;
    int value;
    
    public State(int x, int y, int value)
    {
        this.x = x;
        this.y = y;
        this.value = value;
    }
}

class CompareState implements Comparator<State>
{

    @Override
    public int compare(State arg0, State arg1)
    {
        if (arg0.value > arg1.value)
            return -1;
        if (arg0.value < arg1.value)
            return 1;
        return 0;
    }
    
}

public class GradientPlanner
{
    protected List<Point> frontiers;
    protected byte[] map;
    protected int mapWidth;
    protected int mapHeight;
    protected double scaleX;
    protected double scaleY;
    protected int[][] costs;
    protected byte[] costMap;
    protected int[] offsetX = {1, 1, 0, -1, -1, -1, 0, 1};
    protected int[] offsetY = {0, 1, 1, 1, 0, -1, -1, -1};
    protected float[] headings = {0, 45, 90, 135, 180, -135, -90, -45};
    protected int[] distance1X = {-1, -1, 0, 1};
    protected int[] distance1Y = {0, -1, -1, -1};
    protected int[] distance2X = {0, -1, 1, 1};
    protected int[] distance2Y = {1, 1, 0, 1};
    protected Deque<State> stack = new ArrayDeque<State>();
    protected static final int FIX_SCALE = 256;
    protected static final int DIAG_COST = (int)(Math.sqrt(2.0) * FIX_SCALE);
    protected static final int NORMAL_COST = 1*FIX_SCALE;
    protected static final int MAX_COST = 251;
    protected static final int MIN_COST = 0;
    protected static final int GOAL = Integer.MAX_VALUE;
    protected static final int EDGE_STEP = 10;
    protected static final int FREE = MIN_COST;
    protected static final int OCCUPIED = 255;
    protected static final int EXPANDED = 254;
    protected static final int UNKNOWN = 253;
    protected static final int FRONTIER = 252;
    protected final int robotEdgeDistance;
    
    protected int reprocess=0;

    
    public GradientPlanner(byte[] map, int mapWidth, int mapHeight, int mapWidthMM, int mapHeightMM)
    {
        this.map = map;
        this.mapWidth = mapWidth;
        this.mapHeight = mapHeight;
        this.scaleX = (double)mapWidth/mapWidthMM;
        this.scaleY = (double)mapHeight/mapHeightMM;
        //robotEdgeDistance = (OCCUPIED & 0xff)*FIX_SCALE - (int)((RobotInfo.ROBOT_DIAMETER*scaleX)/2*EDGE_STEP*FIX_SCALE);
        robotEdgeDistance = (OCCUPIED & 0xff)*FIX_SCALE - (int)((RobotInfo.ROBOT_DIAMETER*scaleX*6)/10*EDGE_STEP*FIX_SCALE);
    }

    /**
     * Return a visualisation of the cost map for display use.
     * @return Byte map with one byte per cell.
     */
    protected byte[] getCostMap()
    {
        return costMap;
        /*
            if (costMap == null) return null;
            double scale = 255.0;
            byte[] cm = new byte[mapWidth*mapHeight];
            for(int i = 0; i < mapWidth; i++)
                for(int j = 0; j < mapHeight; j++)
                    //if (cost[i][j] == max)
                    cm[j*mapWidth + i] = (byte)(255 - ((int)(costMap[i][j]*scale) & 0xff));
            System.out.println("built debug map");
            return cm;
        */
    }
    
    protected List<Point> getFrontiers()
    {
        return frontiers;
    }
    
    protected boolean outOfMap(int x, int y)
    {
        if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return true;
        else
            return false;
    }
    
    protected void push(PriorityQueue<State> queue, int x, int y, int value)
    {
        State s = new State(x, y, value);
        costs[x][y] = value;
        queue.add(s);
    }
    
    protected State pop(PriorityQueue<State> queue)
    {
        return queue.remove();
    }
    
  
    /**
     * Build a map that indicates the cost of being in a particular location. Values range from
     * 0.1 to 1.0 a cost above 0.5 indicates a zone that should not be entered. A value of 0.1 indicates a
     * free area. The process uses the current occupancy grid as input. It then performs a distance transform
     * on the map so that obstacles are the peaks and free areas the valleys. The cost slopes down from the
     * peaks. Obstacles are expanded by the safe size of the robot to allow a simple point to be used to
     * represent positions and paths.
     */
    protected void buildCostMap()
    {
        synchronized(this)
        {
            long s = System.currentTimeMillis();
            int value;
            int[][]cm = new int[mapWidth][mapHeight];
            // populate the cost map from the slam map
            for(int y = 0; y < mapHeight; y++)
                for(int x = 0; x < mapWidth; x++)
                {
                    int mapCell = (int)map[y*mapWidth + x] & 0xff;
                    //costMap[x][y] = mapCell == 127 ? MAX_COST/4 : mapCell >= 110 ? MIN_COST :  MAX_COST ;
                    cm[x][y] = ((mapCell >= 110 ? FREE :  OCCUPIED) & 0xff)*FIX_SCALE ;
                }
            // perform a two stage distance transform scanning the map from bottom to top and then
            // top to bottom, using a different filter in each direction
            for(int y = 1; y < mapHeight-1; y++)
                for(int x = 1; x < mapWidth-1; x++)
                {
                    for(int i = 0; i < distance1X.length; i++)
                    {
                        value = cm[x + distance1X[i]][y + distance1Y[i]] - EDGE_STEP*(i % 2 == 1 ? DIAG_COST : NORMAL_COST);
                        if (value > cm[x][y])
                            cm[x][y] = value;
                            
                    }
                }
            for(int y = mapHeight-2; y > 0; y--)
                for(int x = mapWidth-2; x > 0; x--)
                {
                    for(int i = 0; i < distance2X.length; i++)
                    {
                        value = cm[x + distance2X[i]][y + distance2Y[i]] - EDGE_STEP*(i % 2 == 1 ? DIAG_COST : NORMAL_COST);
                        if (value > cm[x][y])
                            cm[x][y] = value;
                            
                    }
                }
            costMap = new byte[mapWidth*mapHeight];
            // now expand walls etc. to provide clearance for the robot. Normalize the cost values.
            for(int y = 0; y < mapHeight; y++)
                for(int x = 0; x < mapWidth; x++)
                {
                    value = cm[x][y];
                    if (value < robotEdgeDistance)
                    {
                        if (((int)map[y*mapWidth + x] & 0xff) == 127)
                            value = UNKNOWN;
                        else
                        {
                            value = value >> 8;
                            if (value <= FREE)
                                value = FREE;
                            if ((value & ~0xff) != 0)
                                System.out.println("Bad value " + value);
                        }
                    }
                    else
                        if (value >= OCCUPIED*FIX_SCALE)
                            value = OCCUPIED;
                        else
                            value = EXPANDED;
                    costMap[y*mapWidth + x] = (byte)value;
                }
            System.out.println("cost time " + (System.currentTimeMillis() - s));
        }
    }

    /**
     * Answer the question is this a free area of the map?
     * @param x
     * @param y
     * @param depth
     * @return
     */
    protected boolean isFree(int x, int y, int depth)
    {
        if (((int)costMap[y*mapWidth + x] & 0xff) == UNKNOWN) return false;
        int freeCnt = 0;
        // We look on each side of the start point for more than a threshold number of
        // free points. If we find this then we consider the point to be a free point.
        for(int j = 0; j < offsetX.length; j++)
        {
            int xoff = offsetX[j];
            int yoff = offsetY[j];
            int cnt = 0;
            for(int i = 1; i < depth; i++)
                //if (!outOfMap(x + xoff*i, y + yoff*i) && costMap[x + xoff*i][y + yoff*i] < UNKNOWN)
                if (!outOfMap(x + xoff*i, y + yoff*i) && ((int)costMap[(y + yoff*i)*mapWidth + (x + xoff*i)] & 0xff) <= FREE)
                    cnt++;
            if (cnt > 3 && ++freeCnt >= 3)
                return true;
        }
        return false;
            
        
    }
    
    /**
     * Anser the question is the point unknown?
     * @param x
     * @param y
     * @param depth
     * @return
     */
    protected boolean isUnknown(int x, int y, int depth)
    {
        if (((int)costMap[y*mapWidth + x] & 0xff) != UNKNOWN) return false;
        boolean[] unknown = new boolean[offsetX.length];
        int adjCnt = 0;
        // We search lines radiating out from the current point, for the specified depth. We only consider
        // the area to be unknown if at least three adjacent lines are all free.
        for(int j = 0; j < offsetX.length; j++)
        {
            int xoff = offsetX[j];
            int yoff = offsetY[j];
            int cnt = 1;
            while(cnt < depth && !outOfMap(x + xoff*cnt, y + yoff*cnt) && ((int)costMap[(y + yoff*cnt)*mapWidth + (x + xoff*cnt)] & 0xff) == UNKNOWN)
                cnt++;
            unknown[j] = cnt >= depth;
            if (cnt >= depth)
            {
                if (++adjCnt >= 3)
                    return true;
            }
            else
                adjCnt = 0;
        }
        if (adjCnt == 0) return false;
        for(int i = 2 - adjCnt; i >= 0 ; i--)
            if (!unknown[i]) return false;
        return true;
            
        
    }

    
    /**
     * Answers the question is the current point a frontier. A frontier is a location
     * that has free space on one side and unknown space on another.
     * @param x
     * @param y
     * @return
     */
    protected boolean isFrontier(int x, int y)
    {
        //if (costMap[x][y] != FREE && costMap[x][y] != 0.11) return false;
        if (((int)costMap[y*mapWidth + x] & 0xff) != FREE) return false;
        boolean free = false;
        boolean unknown = false;
        for(int i = 0; i < offsetX.length; i++)
        {
            free = free || isFree(x+offsetX[i], y+offsetY[i], 15);
            unknown = unknown || isUnknown(x+offsetX[i], y+offsetY[i], 15);
            if (free && unknown)
            {
                return true;
                /*
                int unknownCnt = 0;
                int offx = x+offsetX[i];
                int offy = y+offsetY[i];
                for(int j = 0; j < offsetX.length; j++)
                    if (costMap[offx + offsetX[j]][offy+offsetY[j]] == 0.125)
                        if (++unknownCnt >= 3)
                            return true;
                */
                    
            }
        }
        return false;
    }
    
    protected void mergeFrontier(List<List<Point>> fl, int[][] fm, int x, int y, int depth)
    {
        // Search the map looking for other points that are close to this new point
        for(int j = 0; j < distance1X.length; j++)
        {
            int xoff = distance1X[j];
            int yoff = distance1Y[j];
            int cnt = 1;
            int frontierNo = 0;
            while(cnt < depth && !outOfMap(x + xoff*cnt, y + yoff*cnt))
            {
                frontierNo = fm[x + xoff*cnt][y + yoff*cnt];
                if (frontierNo > 0)
                {
                    
                    if (fm[x][y] == 0 || fm[x][y] == frontierNo)
                    {
                        List<Point> pl = fl.get(frontierNo-1);
                        pl.add(new Point(x, y));
                        fm[x][y] = frontierNo;
                    }
                    else
                    {
                        // current point already in a list, need to merge two lists
                        List<Point> pl1 = fl.get(frontierNo-1);
                        List<Point> pl2 = fl.get(fm[x][y]-1);
                        while(!pl2.isEmpty())
                        {
                            Point p = pl2.remove(0);
                            fm[p.x][p.y] = frontierNo;
                            pl1.add(p);
                        }
                        fm[x][y] = frontierNo;
                        
                    }

                }
                cnt++;
            }
        }
        if (fm[x][y] == 0)
        {
            // no other point found start a new group.
            List<Point> pl = new ArrayList<Point>();
            pl.add(new Point(x, y));
            fl.add(pl);
            fm[x][y] = fl.size();
        }
    }
    
    protected List<Point>getFrontierTargets(List<List<Point>> old)
    {
        int minSize = (int)(RobotInfo.ROBOT_DIAMETER*scaleX/2);
        List<Point> newList = new ArrayList<Point>();
        for(List<Point> pl : old)
            if (pl.size() > 10)
            {
                Rectangle r = null;
                int x = 0;
                int y = 0;
                for(Point p : pl)
                {
                    if (r == null)
                        r = new Rectangle(p.x, p.y, 1, 1);
                    else
                        r.add(p);
                    x += p.x;
                    y += p.y;
                }
                if (r.width > minSize || r.height > minSize)
                {
                    x = x/pl.size();
                    y = y/pl.size();
                    // make sure the target is reachable!
                    if (costMap[y*mapWidth + x] > EXPANDED)
                    {
                        System.out.println("Can't reach target " + x + ", " + y);
                        x = pl.get(0).x;
                        y = pl.get(0).y;
                    }
                    newList.add(new Point((int)(x/scaleX), (int)(y/scaleY)));
                }
            }
        return newList;
    }
    
    protected int findFrontiers(PriorityQueue<State> queue)
    {
        List<List<Point>> frontierList = new ArrayList<List<Point>>();
        int[][] frontierMap = new int[mapWidth][mapHeight];
        int cnt = 0;
        for(int y = 2; y < mapHeight-2; y++)
            for(int x = 2; x < mapWidth-2; x++)
                if (isFrontier(x, y))
                {
                    costMap[y*mapWidth + x] = (byte)FRONTIER;
                    //addTarget(queue, x, y);
                    mergeFrontier(frontierList, frontierMap, x, y, 10);
                    cnt++;
                }
        //System.out.println("Frontier count " + cnt + " f rects " + frontierList.size());
        frontiers = getFrontierTargets(frontierList);
        int step = mapWidth/8;
        for(int x = step; x < mapWidth; x += step)
        {
            frontiers.add(new Point((int)(x/scaleX), 0));
            frontiers.add(new Point((int)(x/scaleX), (int)((mapHeight-1)/scaleY)));            
        }
        for(int y = step; y < mapHeight; y += step)
        {
            frontiers.add(new Point(0, (int)(y/scaleY)));
            frontiers.add(new Point((int)((mapWidth-1)/scaleX), (int)(y/scaleY)));            
        }
        for(Point p : frontiers)
            addTarget(queue, (int)(p.x*scaleX), (int)(p.y*scaleY));
        System.out.println("Frontier count " + cnt + " pl " + frontierList.size() + " targets " + frontiers.size());
        return frontiers.size();
    }
    
    protected int findAreasToExplore(PriorityQueue<State> queue)
    {
        long s = System.currentTimeMillis();
        List<Point> targets = new ArrayList<Point>();
        boolean[][] isTarget = new boolean[mapWidth/10][mapHeight/10];
        int xMin = Integer.MAX_VALUE;
        int xMax = Integer.MIN_VALUE;
        int yMin = Integer.MAX_VALUE;
        int yMax = Integer.MIN_VALUE;
        int cnt1 = 0;
        int cnt2 = 0;
        for(int x10 = 0; x10 < mapWidth-10; x10 += 10)
            for(int y10 = 0; y10 < mapHeight-10; y10 += 10)
            {
                int cost = 0;
                for(int x1 = x10; x1 < x10+10; x1++)
                    for(int y1 = y10; y1 < y10+10; y1++)
                    {
                        if (outOfMap(x1, y1) || ((int)costMap[y1*mapWidth + x1] & 0xff) >= EXPANDED)
                            cost =  Integer.MIN_VALUE;
                        else
                            cost += (int)map[y1*mapWidth + x1] & 0xff;                        
                    }
                int x = x10/10;
                int y = y10/10;
                if (cost > 12000 && cost < 16500)
                {
                    isTarget[x][y] = true;
                    cnt1++;
                }
                else
                {
                    if (x < xMin) xMin = x;
                    if (x > xMax) xMax = x;
                    if (y < yMin) yMin = y;
                    if (y > yMax) yMax = y;
                }
            }
        xMin--; xMax++;
        yMin--; yMax++;
        if (xMin < 0) xMin = 0;
        if (xMax > mapWidth/10) yMax = mapHeight/10;
        if (yMin < 0) xMin = 0;
        if (yMax > mapWidth/10) yMax = mapHeight/10;
        
        for (int x = xMin; x < xMax; x++)
            for(int y = yMin; y < yMax; y++)
                if (isTarget[x][y])
                {
                    int x10 = x*10 + 5;
                    int y10 = y*10 + 5;
                    addTarget(queue, x10, y10);
                    costMap[y10*mapWidth + x10] = (byte) FRONTIER;
                    cnt2++;
                }
        System.out.println("Targets " + cnt2 + "/" + cnt1 + " time " + (System.currentTimeMillis() - s));
        return cnt2;
       
    }
    
    protected void addNeighbours(PriorityQueue<State> queue, int x, int y)
    {
        int parentCost = costs[x][y];
        //for(int i = 0; i < offsetX.length; i+=2)
        for(int i = 0; i < offsetX.length; i++)
        {
            int newX = x + offsetX[i];
            int newY = y + offsetY[i];
            if(outOfMap(newX, newY)) continue;
            int value = ((int)costMap[newY*mapWidth + newX] & 0xff);
            if (value >= EXPANDED)
                continue;
            if (value == UNKNOWN)
                value = 32;
            int moveCost = ( i % 2 == 1 ? DIAG_COST : NORMAL_COST );
            //double newCost = parentCost + moveCost + (255-((int)map[y*mapWidth + x] & 0xff));
            //double newCost = parentCost - (moveCost * mapCost(x, y));
            int newCost = parentCost - (moveCost + 16*value);
            //double newCost = parentCost - (moveCost *(costMap[newX][newY] == 0.0 ? 0.125 : costMap[newX][newY]));
            if (newCost < 0) System.out.println("-ve cost " + x + " " + y);
            //if (newCost > costs[newX][newY])
            if (costs[newX][newY] == 0)
            {
                if (costs[newX][newY] != 0)
                {    
                    reprocess++;
                    //if (reprocess < 100)
                      //  System.out.printf("rp %d %d old %d new %d\n", newX, newY, costs[newX][newY], newCost);
                }
                push(queue, newX, newY, newCost);
            }
            //else
                //if (newCost > costs[newX][newY])
                    //reprocess++;

        }
    }
    
    protected void addTarget(PriorityQueue<State> queue,int x, int y)
    {
        push(queue, x, y, 0);
        costs[x][y] = GOAL;
        addNeighbours(queue, x, y);        
    }

    protected void search(PriorityQueue<State> queue)
    {
        long s = System.currentTimeMillis();
        int cnt1 = 1;
        while(!queue.isEmpty())
        {
            cnt1++;
            State curItem = pop(queue);
            addNeighbours(queue, curItem.x, curItem.y);
        }
        System.out.println("cnt1 " + cnt1 + " reprocess " + reprocess + " time " + (System.currentTimeMillis() - s));

    }
    
    
    public boolean findPath(Pose start, Pose target)
    {
        buildCostMap();
        int x = (int)(start.getX()*scaleX);
        int y = (int)(start.getY()*scaleY);
        if (costMap[y*mapWidth + x] >= EXPANDED)
            return false;
        x = (int)(target.getX()*scaleX);
        y = (int)(target.getY()*scaleY);
        if (costMap[y*mapWidth + x] >= EXPANDED)
            return false;
        costs = new int[mapWidth][mapHeight];
        CompareState stateCompare = new CompareState();
        PriorityQueue<State> queue = new PriorityQueue<State>(100, stateCompare);
        addTarget(queue, x, y);
        search(queue);
        return true;
    }

    public boolean explore(Pose start)
    {
        buildCostMap();
        int x = (int)(start.getX()*scaleX);
        int y = (int)(start.getY()*scaleY);
        if (costMap[y*mapWidth + x] >= EXPANDED)
            return false;
        costs = new int[mapWidth][mapHeight];
        CompareState stateCompare = new CompareState();
        PriorityQueue<State> queue = new PriorityQueue<State>(100, stateCompare);
        //if (findFrontiers(queue) == 0) return false;
        if (findAreasToExplore(queue) == 0) return false;
        search(queue);
        if (costs[x][y] == 0) return false;
        return true;
    }

    public Pose nextPose(Pose p)
    {
        synchronized(this)
        {
            float heading = p.getHeading();
            if (heading < 0)
                heading += 360;
            int headingIndex = (int)(heading/45.0f + 0.5f) % 8;
            //System.out.printf("orig %f mod %f index %d inv %f\n", p.getHeading(), heading, headingIndex, headings[headingIndex]);
            int x = (int)(p.getX()*scaleX);
            int y = (int)(p.getY()*scaleY);
            if (costs[x][y] == 0 || costs[x][y] == GOAL)
                return null;
            int bestIndex = headingIndex;
            double bestCost = costs[x+offsetX[bestIndex]][y+offsetY[bestIndex]];
            for(int i = 0; i < offsetX.length; i++)
            {
                if (costs[x + offsetX[i]][y+offsetY[i]] > bestCost)
                {
                    bestIndex = i;
                    bestCost = costs[x + offsetX[i]][y+offsetY[i]];
                }
            }
            //System.out.printf("best index %d\n", bestIndex);
            return new Pose((float)((x+offsetX[bestIndex])/scaleX), (float)((y+offsetY[bestIndex])/scaleY), headings[bestIndex]);
        }
    }
    
    protected List<Pose> getPath(Pose from)
    {
        List<Pose> p = new ArrayList<Pose>();
        Pose next = nextPose(from);
        if (next == null)
            return p;
        p.add(from);
        while(next != null)
        {
            p.add(next);
            next = nextPose(next);
        }
        return p;        
    }


}
