import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.PriorityQueue;

import lejos.robotics.navigation.Pose;

class State
{
    int x;
    int y;
    double value;
    
    public State(int x, int y, double value)
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
    protected double[][] costs;
    protected double[][] costMap;
    protected int[] offsetX = {0, 1, 1, 1, 0, -1, -1, -1};
    protected int[] offsetY = {-1, -1, 0, 1, 1, 1, 0, -1};
    protected Deque<State> stack = new ArrayDeque<State>();
    protected static final double DIAG_COST = Math.sqrt(2.0);
    protected static final double NORMAL_COST = 1.0;
    protected static final double MAX_COST = 1000.0;
    protected static final double MIN_COST = 0.1;
    protected static final double UNKNOWN_COST = 100.0;
    protected static final double GOAL = 10*MAX_COST;
    public byte[] debugMap;
    
    public GradientPlanner(byte[] map, int mapWidth, int mapHeight, int mapWidthMM, int mapHeightMM)
    {
        this.map = map;
        this.mapWidth = mapWidth;
        this.mapHeight = mapHeight;
        this.scaleX = (double)mapWidth/mapWidthMM;
        this.scaleY = (double)mapHeight/mapHeightMM;
        debugMap = new byte[mapWidth*mapHeight];
        for(int i = 0; i < debugMap.length; i++)
            debugMap[i] = (byte)255;
    }
    
    protected void buildDebugMap(double[][] cost, double max)
    {
        double scale = 255.0/max;
        for(int i = 0; i < mapWidth; i++)
            for(int j = 0; j < mapHeight; j++)
                debugMap[j*mapWidth + i] = (byte)(255 - ((int)(cost[i][j]*scale) & 0xff));
        System.out.println("built debug map");
        //debugMap = map;
    }
    
    protected boolean outOfMap(int x, int y)
    {
        if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return true;
        else
            return false;
    }
    
    protected void push(PriorityQueue<State> queue, int x, int y, double value)
    {
        State s = new State(x, y, value);
        costs[x][y] = value;
        //queue.push(s);
        queue.add(s);
    }
    
    protected State pop(PriorityQueue<State> queue)
    {
        //return queue.pop();
        return queue.remove();
    }
    
    protected int mapCost(int x, int y)
    {
        int val = (int)map[y*mapWidth + x] & 0xff;
        if (val > 223)
            val = 255;
        else
            if (val < 32)
                val = 0;
        return 256 - val;
    }
    
    
    
    protected void buildCostMap()
    {
        double value;
        costMap = new double[mapWidth][mapHeight];
        // populate the cost map from the slam map
        for(int x = 0; x < mapWidth; x++)
            for(int y = 0; y < mapHeight; y++)
            {
                int mapCell = (int)map[y*mapWidth + x] & 0xff;
                costMap[x][y] = mapCell >= 128 ? MIN_COST : mapCell < 127 ? MAX_COST : UNKNOWN_COST;
            }
        //buildDebugMap(costMap, MAX_COST);
        for(int x = 1; x < mapWidth-1; x++)
            for(int y = 1; y < mapHeight-1; y++)
            {
                for(int i = 0; i < offsetX.length; i++)
                {
                    value = costMap[x + offsetX[i]][y + offsetY[i]] - 0.02*MAX_COST;
                    if (value > costMap[x][y])
                        costMap[x][y] = value;
                        
                }
            }
        
        for(int x = mapWidth-2; x > 0; x--)
            for(int y = mapHeight-2; y > 0; y--)
            {
                for(int i = 0; i < offsetX.length; i++)
                {
                    value = costMap[x + offsetX[i]][y + offsetY[i]] - 0.02*MAX_COST;
                    if (value > costMap[x][y])
                        costMap[x][y] = value;
                        
                }
            }
        //buildDebugMap(costMap, MAX_COST);
        double robotDist = 2*300.0/(2*1000.0) * MAX_COST;
        for(int x = 0; x < mapWidth; x++)
            for(int y = 0; y < mapWidth; y++)
            {
                value = costMap[x][y];
                if (value < MAX_COST - robotDist)
                {
                    value = value/(2*MAX_COST);
                    if (value < MIN_COST)
                        value = MIN_COST;
                }
                else
                    value = 1.0;
                costMap[x][y] = value;
            }
        buildDebugMap(costMap, 1.0);
    }
    
    protected void addNeighbours(PriorityQueue<State> queue, int x, int y)
    {
        double parentCost = costs[x][y];
        for(int i = 0; i < offsetX.length; i+=2)
        {
            int newX = x + offsetX[i];
            int newY = y + offsetY[i];
            if(outOfMap(newX, newY) || costMap[newX][newY] >= 0.5)
                continue;
            double moveCost = ( i % 2 == 1 ? DIAG_COST : NORMAL_COST );
            //double newCost = parentCost + moveCost + (255-((int)map[y*mapWidth + x] & 0xff));
            //double newCost = parentCost - (moveCost * mapCost(x, y));
            double newCost = parentCost - (moveCost * costMap[newX][newY]);
            if (newCost < 0) System.out.println("-ve cost " + x + " " + y);
            //if (newCost > costs[newX][newY])
            if (costs[newX][newY] == 0.0)
                push(queue, newX, newY, newCost);
        }
    }
    
    public void findPathsTo(Pose target)
    {
        int x = (int)(target.getX()*scaleX);
        int y = (int)(target.getY()*scaleY);
        buildCostMap();
        costs = new double[mapWidth][mapHeight];
        CompareState stateCompare = new CompareState();
        PriorityQueue<State> curQueue = new PriorityQueue<State>(100, stateCompare);
        push(curQueue, x, y, 0);
        costs[x][y] = GOAL;
        int cnt1 = 1;
        int cnt2 = 0;
            while(!curQueue.isEmpty())
            {
                cnt1++;
                State curItem = pop(curQueue);
                if (curItem.value <= costs[curItem.x][curItem.y])
                addNeighbours(curQueue, curItem.x, curItem.y);
            }
        System.out.println("cnt1 " + cnt1 + " cnt2 " + cnt2);
    }

    public Pose nextPose(Pose p)
    {   
        double bestCost = 0;
        int x = (int)(p.getX()*scaleX);
        int y = (int)(p.getY()*scaleY);
        int bestX=0;
        int bestY=0;
        if (costs[x][y] == GOAL)
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
        System.out.println("loc x " + bestX + " y " + bestY + " cost " + bestCost + " c2 " + costMap[bestX][bestY]);
        return new Pose((float)(bestX/scaleX), (float)(bestY/scaleY), 0);
    }

}
