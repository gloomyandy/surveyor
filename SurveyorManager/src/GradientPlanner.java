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
    protected static final int EDGE_STEP = 12;
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
        robotEdgeDistance = (OCCUPIED & 0xff)*FIX_SCALE - (int)((RobotInfo.ROBOT_DIAMETER*scaleX)/2*EDGE_STEP*FIX_SCALE);
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
            if (cnt > 2 && ++freeCnt >= 3)
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
        // We search lines radiating out from the current point, for the specified depth. We only consdier
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
            free = free || isFree(x+offsetX[i], y+offsetY[i], 20);
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
        if (x == 821 && y == 385) System.out.println("free " + free + " unknown " + unknown);
        return false;
    }
    
    protected void findFrontiers(PriorityQueue<State> queue)
    {
        int cnt = 0;
        for(int y = 2; y < mapHeight-2; y++)
            for(int x = 2; x < mapWidth-2; x++)
                if (isFrontier(x, y))
                {
                    costMap[y*mapWidth + x] = (byte)FRONTIER;
                    push(queue, x, y, 0);
                    costs[x][y] = GOAL;
                    addNeighbours(queue, x, y);
                    cnt++;
                }
        System.out.println("Frontier count " + cnt);
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
    
    public void addTarget(PriorityQueue<State> queue,int x, int y)
    {
        push(queue, x, y, 0);
        costs[x][y] = GOAL;
        addNeighbours(queue, x, y);        
    }
    
    public boolean findPathsTo(Pose target)
    {
        System.out.println("fpt");
        int x = (int)(target.getX()*scaleX);
        int y = (int)(target.getY()*scaleY);
        buildCostMap();
        long s = System.currentTimeMillis();
        costs = new int[mapWidth][mapHeight];
        if (costMap[y*mapWidth + x] > EXPANDED)
            return false;
        CompareState stateCompare = new CompareState();
        PriorityQueue<State> curQueue = new PriorityQueue<State>(100, stateCompare);
        findFrontiers(curQueue);
        //addTarget(curQueue, x, y);
        //push(curQueue, x, y, 0);
        //costs[x][y] = GOAL;
        //addNeighbours(curQueue, x, y);
        int cnt1 = 1;
        while(!curQueue.isEmpty())
        {
            cnt1++;
            State curItem = pop(curQueue);
            if (curItem.value > costs[curItem.x][curItem.y])
                System.out.printf("Skipping push %f %f\n", curItem.x, curItem.y);
            addNeighbours(curQueue, curItem.x, curItem.y);
        }
        System.out.println("cnt1 " + cnt1 + " reprocess " + reprocess + " time " + (System.currentTimeMillis() - s));
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
            /*
            double bestCost = 0;
            int x = (int)(p.getX()*scaleX);
            int y = (int)(p.getY()*scaleY);
            int bestX=0;
            int bestY=0;
            if (costs == null || costs[x][y] == GOAL)
                return null;
            for(int i = 0; i < offsetX.length; i++)
            {
                if (costs[x + offsetX[i]][y+offsetY[i]] >= bestCost)
                {
                    bestX = x + offsetX[i];
                    bestY = y+offsetY[i];
                    bestCost = costs[x + offsetX[i]][y+offsetY[i]];
                }
            }
            //System.out.println("loc x " + bestX + " y " + bestY + " cost " + bestCost + " c2 " + costMap[bestX][bestY]);
            return new Pose((float)(bestX/scaleX), (float)(bestY/scaleY), 0);
            */
        }
    }
    
    protected List<Pose> getPath(Pose from)
    {
        System.out.println("GPS");
        List<Pose> p = new ArrayList<Pose>();
        Pose next = nextPose(from);
        if (next == null) System.out.println("null");
        if (next == null)
            return p;
        p.add(from);
        while(next != null)
        {
            p.add(next);
            next = nextPose(next);
        }
        System.out.println("GPF");
        return p;        
    }


}
