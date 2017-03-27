import java.awt.Point;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.PriorityQueue;

import lejos.robotics.navigation.Pose;

class State
{
    int x;
    int y;
    //double value;
    int value;
    
    //public State(int x, int y, double value)
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
    //protected double[][] costs;
    int[][] costs;
    protected int[] offsetX = {0, 1, 1, 1, 0, -1, -1, -1};
    protected int[] offsetY = {-1, -1, 0, 1, 1, 1, 0, -1};
    protected Deque<State> stack = new ArrayDeque<State>();
    //protected static final double DIAG_COST = Math.sqrt(2.0);
    //protected static final double NORMAL_COST = 1.0;
    protected static final int DIAG_COST = 2;
    protected static final int NORMAL_COST = 1;
    protected static final int GOAL = Integer.MAX_VALUE;
    
    public GradientPlanner(byte[] map, int mapWidth, int mapHeight, int mapWidthMM, int mapHeightMM)
    {
        this.map = map;
        this.mapWidth = mapWidth;
        this.mapHeight = mapHeight;
        this.scaleX = (double)mapWidth/mapWidthMM;
        this.scaleY = (double)mapHeight/mapHeightMM;
    }
    
    protected boolean outOfMap(int x, int y)
    {
        if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return true;
        else
            return false;
    }
    
    //protected void push(Deque<State> queue, int x, int y, int value)
    protected void push(PriorityQueue<State> queue, int x, int y, int value)
    {
        State s = new State(x, y, value);
        costs[x][y] = value;
        //queue.push(s);
        queue.add(s);
    }
    
    //protected State pop(Deque<State> queue)
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
    
    //protected void addNeighbours(Deque<State> queue, int x, int y)
    protected void addNeighbours(PriorityQueue<State> queue, int x, int y)
    {
        int parentCost = costs[x][y];
        for(int i = 0; i < offsetX.length; i++)
        {
            int newX = x + offsetX[i];
            int newY = y + offsetY[i];
            if(outOfMap(newX, newY))
                continue;
            int moveCost = ( i % 2 == 1 ? DIAG_COST : NORMAL_COST );
            //double newCost = parentCost + moveCost + (255-((int)map[y*mapWidth + x] & 0xff));
            int newCost = parentCost - (int)(moveCost * mapCost(x, y));
            if (newCost < 0) System.out.println("-ve cost " + x + " " + y);
            //if (newCost > costs[newX][newY])
            if (costs[newX][newY] == 0)
                push(queue, newX, newY, newCost);
        }
    }
    
    public void findPathsTo(Pose target)
    {
        int x = (int)(target.getX()*scaleX);
        int y = (int)(target.getY()*scaleY);
        costs = new int[mapWidth][mapHeight];
        /*
        for(int i = 0; i < mapWidth; i++)
            for(int j = 0; j < mapHeight; j++)
                costs[i][j] = Double.MAX_VALUE;
                */
        CompareState stateCompare = new CompareState();
        PriorityQueue<State> curQueue = new PriorityQueue<State>(100, stateCompare);
        //push(curQueue, x, y, GOAL);
        push(curQueue, x, y, 0);
        costs[x][y] = GOAL;
        int cnt1 = 1;
        int cnt2 = 0;
            while(!curQueue.isEmpty())
            {
                cnt1++;
                State curItem = pop(curQueue);
                //if (curItem.value <= costs[curItem.x][curItem.y])
                addNeighbours(curQueue, curItem.x, curItem.y);
            }
        System.out.println("cnt1 " + cnt1 + " cnt2 " + cnt2);
    }

    /*
    public void findPathsTo(Pose target)
    {
        int x = (int)(target.getX()*scaleX);
        int y = (int)(target.getY()*scaleY);
        costs = new int[mapWidth][mapHeight];
        Deque<State> curQueue = new ArrayDeque<State>();
        Deque<State> nextQueue = new ArrayDeque<State>();
        //push(curQueue, x, y, GOAL);
        push(curQueue, x, y, 0);
        costs[x][y] = GOAL;
        int cnt1 = 1;
        int cnt2 = 0;
        while(!curQueue.isEmpty())
        {
            while(!curQueue.isEmpty())
            {
                cnt1++;
                State curItem = curQueue.pop();
                if (curItem.value <= costs[curItem.x][curItem.y])
                addNeighbours(nextQueue, curItem.x, curItem.y);
            }
            cnt2++;
            Deque<State> temp = curQueue;
            curQueue = nextQueue;
            nextQueue = temp;;
        }
        System.out.println("cnt1 " + cnt1 + " cnt2 " + cnt2);
    }
*/   
    public Pose nextPose(Pose p)
    {   
        int bestCost = 0;
        int x = (int)(p.getX()*scaleX);
        int y = (int)(p.getY()*scaleY);
        int bestX=0;
        int bestY=0;
        if (costs[x][y] == GOAL)
            return null;
        for(int i = 0; i < offsetX.length; i++)
        {
            if (costs[x + offsetX[i]][y+offsetY[i]] > bestCost)
            {
                bestX = x + offsetX[i];
                bestY = y+offsetY[i];
                bestCost = costs[x + offsetX[i]][y+offsetY[i]];
            }
        }
        return new Pose((float)(bestX/scaleX), (float)(bestY/scaleY), 0);
    }
    /*
    protected void push(Deque<State> queue, int x, int y, double value)
    {
        State s = new State(x, y, value);
        costs[x][y] = value;
        queue.push(s);
    }

    protected State pop(Deque<State> queue)
    {
        return queue.pop();
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
    
    protected void addNeighbours(Deque<State> queue, int x, int y)
    {
        double parentCost = costs[x][y];
        for(int i = 0; i < offsetX.length; i++)
        {
            int newX = x + offsetX[i];
            int newY = y + offsetY[i];
            if(outOfMap(newX, newY))
                continue;
            double moveCost = ( i % 2 == 1 ? DIAG_COST : NORMAL_COST );
            //double newCost = parentCost + moveCost + (255-((int)map[y*mapWidth + x] & 0xff));
            double newCost = parentCost + moveCost * mapCost(x, y);
            if (newCost < costs[newX][newY])
                push(queue, newX, newY, newCost);
        }
    }
    
    public void findPathsTo(Pose target)
    {
        int x = (int)(target.getX()*scaleX);
        int y = (int)(target.getY()*scaleY);
        costs = new double[mapWidth][mapHeight];
        for(int i = 0; i < mapWidth; i++)
            for(int j = 0; j < mapHeight; j++)
                costs[i][j] = Double.MAX_VALUE;
        Deque<State> curQueue = new ArrayDeque<State>();
        Deque<State> nextQueue = new ArrayDeque<State>();
        push(curQueue, x, y, 0.0);
        int cnt1 = 1;
        int cnt2 = 0;
        while(!curQueue.isEmpty())
        {
            while(!curQueue.isEmpty())
            {
                cnt1++;
                State curItem = curQueue.pop();
                //if (curItem.value >= costs[curItem.x][curItem.y])
                addNeighbours(nextQueue, curItem.x, curItem.y);
            }
            cnt2++;
            Deque<State> temp = curQueue;
            curQueue = nextQueue;
            nextQueue = temp;;
        }
        System.out.println("cnt1 " + cnt1 + " cnt2 " + cnt2);
    }
    
    public Pose nextPose(Pose p)
    {   
        double bestCost = Double.MAX_VALUE;
        int x = (int)(p.getX()*scaleX);
        int y = (int)(p.getY()*scaleY);
        int bestX=0;
        int bestY=0;
        if (costs[x][y] == 0.0)
            return null;
        for(int i = 0; i < offsetX.length; i++)
        {
            if (costs[x + offsetX[i]][y+offsetY[i]] < bestCost)
            {
                bestX = x + offsetX[i];
                bestY = y+offsetY[i];
                bestCost = costs[x + offsetX[i]][y+offsetY[i]];
            }
        }
        return new Pose((float)(bestX/scaleX), (float)(bestY/scaleY), 0);
    }
    */

}
