package s0574186_s0573682;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;
import java.awt.*;
import java.awt.geom.Path2D;
import java.util.*;

public class myAI2 extends AI {
    public myAI2(Info info) {
        super(info);
        enlistForTournament(574186, 573682);
    }

    @Override
    public String getName() {
        return "Goth Jedi";
    }

    @Override
    public Color getColor() {
        return Color.BLACK;
    }

    // 1 D58o9E4Qif avz
    // 2 eaeIxGwdVa yo
    // k0RvVQuHMI R49qQWhUlj
    // 3 r8m0oYbMTD e2X7dxDrKH
    Point pos;
    int lastScore = 0;
    Node[][] graph; // = buildGraph();
    ArrayList<Point> pearls = new ArrayList<>();
    ArrayList<Node> openPearls; // = sortPearls();
    ArrayList<Node> way = new ArrayList<>(0);
    ArrayList<Node> airWay = new ArrayList<>(0);

    @Override
    public PlayerAction update() {
        /*pos = new Point(info.getX(), info.getY());
        float velocity = info.getMaxVelocity();
        float direction;
        float air = info.getAir();

        // if a pearl got collected
        if (info.getScore() > lastScore){
            //check which pearl it was and remove from queues
            for (int i=0; i<openPearls.size(); i++) {
                float d = getDistance(pos, pearls.get(i));
                if (d < 20) {
                    way.remove(openPearls.get(i));
                    openPearls.remove(openPearls.get(i));
                    pearls.remove(i);
                    break;
                }
            }
            lastScore= info.getScore();


            if (air < Math.abs(pearls.get(0).y*deep())+getDistance(pos, pearls.get(0))) {
                if (pathFree(pos, new Point(pos.x, 0))) {
                    airWay.clear();
                    airWay.add(new Node(new Point(pos.x, 0)));

                } else {
                    airWay = quickestWay(getClosestNode(pos), getClosestNode(new Point(pos.x, 0)));
                }
            }
        }
        if (air < Math.abs(pearls.get(0).y*deep())+getDistance(pos, pearls.get(0)) && pos.x != pearls.get(0).x && pos.y==0) {
           //  make a getSurface meth, for when there's a obs in da way
            direction = seek(pos, new Point(pearls.get(0).x, 0));
            return new DivingAction(velocity,direction);
        }
        if (air < Math.abs(pos.y*deep()) && airWay.size()==0) {
            if (pathFree(pos, new Point(pos.x, 0))) {
                if (air < Math.abs(pos.y*deep())) {
                    airWay.clear();
                    airWay.add(new Node(new Point(pos.x, 0)));
                }
            } else {
                airWay = quickestWay(getClosestNode(pos), getClosestNode(new Point(pos.x, 0)));
            }
        }
        if (airWay.size() > 0) {
            while (way.size() > 1 && pathFree(pos, way.get(1).coordinates)) {
                way.remove(0);
            }
            if ((airWay.size() > 1 && getDistance(pos, airWay.get(0).coordinates) < 10)
                || (airWay.size() == 1 && air > 900) ) {
                airWay.remove(0);
            }
            if (airWay.size() > 0) {
                direction = seek(pos, airWay.get(0).coordinates);
                return new DivingAction(velocity, direction);
            }
            way.clear();
            way = quickestWay(getClosestNode(pos), openPearls.get(0));

        }
        // check if there's a path you can follow
        if (way.size() > 0) {
            // if ya can go straight to the next waypoint, skip the current
            while (way.size() > 1 && pathFree(pos, way.get(1).coordinates)) {
                way.remove(0);
            }
            // if ya close enough to the next waypoint, ya can remove it
            if (way.size() > 0 && getDistance(pos, way.get(0).coordinates) < 10) {
                way.remove(0);
            }
            // now, if there's still path left, follow it
            if (way.size() > 0) {
                direction = seek(pos, way.get(0).coordinates);
                if (pathFree(pos, pearls.get(0))) direction = seek(pos, pearls.get(0));
                return new DivingAction(velocity, direction);
            }
        }

        // else, check if you can go there straight
        if (pathFree(pos, pearls.get(0))) {
            direction = seek(pos, pearls.get(0));
        } // if ya can't get there and there's no path, make one
        else {
            way = quickestWay(getClosestNode(pos), openPearls.get(0));
            if (way.size() != 0) direction = seek(pos, way.get(0).coordinates);
            else  direction = seek(pos,pearls.get(0));
        }

        return new DivingAction(velocity, direction);*/
        return new DivingAction(0,0);
    }

    private float deep() {
        float deep;
        if (openPearls.size() == 1) {
            return 0;
        } else if (pathFree(pearls.get(0), new Point(pearls.get(0).x, 0))) {
            return 0.72f;
        } else {
            ArrayList<Node> w = quickestWay(getClosestNode(pearls.get(0)), getClosestNode(new Point(pearls.get(0).x, 0)));
            return (w.get(w.size()-1).cost /  Math.abs(pearls.get(0).y))*0.72f;
        }
    }

    /* get closest Pearl
    ArrayList<Point> pearls =  new ArrayList<>(Arrays.asList(info.getScene().getPearl()));
    Point goal = getPearl(pos);
    private Point getPearl(Point pos) {
        if (pos==null) pos = new Point(info.getScene().getWidth()/2, 0);

        float distance = Integer.MAX_VALUE;
        Point closest = null;

        for (Point pearl : pearls) {
            float dis = getDistance(pos, pearl);
            if (dis < distance) {
                closest = pearl;
                distance = dis;
            }
        }

        return closest;
    }*/
    // sort in one direction
    private  ArrayList<Node> sortPearls() {
        ArrayList<Point> pearls = new ArrayList<>(Arrays.asList(info.getScene().getPearl()));
        Point[] sortedPearls = new Point[pearls.size()];
        int w = 0;
        ArrayList<Point> deeps = new ArrayList<>();


        // all <-693
        for (int i=0; i<sortedPearls.length; i++) {
            if (pearls.get(i).y<=-694) {
                deeps.add(pearls.get(i));
            }
        }

        // sort deeps from left to right
        int s = deeps.size();
        for (int i=0; i<s; i++) {
            double x = 0;
            int closest = 0;
            for (int j=0; j<deeps.size(); j++) {
                if (deeps.get(j).x>x) {
                    x = deeps.get(j).x;
                    closest = j;
                }
            }
            sortedPearls[sortedPearls.length-1-w] = deeps.get(closest);
            pearls.remove(deeps.get(closest));
            deeps.remove(closest);
            w++;
        }

        //get average deeps.x
        float av = 0;
        for (int i=0; i<w; i++) {
            av+=sortedPearls[sortedPearls.length-1-i].x;
        }
        av/=w;

        if (av >= info.getScene().getWidth()/2f) {
            // if deep are more on the right side, sort l->r
            for (int i=0; i<sortedPearls.length-w; i++) {
                double x = Integer.MAX_VALUE;
                int closest = 0;
                for (int j=0; j<pearls.size(); j++) {
                    if (pearls.get(j).x<x) {
                        x = pearls.get(j).x;
                        closest = j;
                    }
                }
                sortedPearls[i] = pearls.get(closest);
                pearls.remove(closest);
            }
        } else {
            // if deep are more on the left side, sort r->l
            for (int i=0; i<sortedPearls.length-w; i++) {
                double x = 0;
                int closest = 0;
                for (int j=0; j<pearls.size(); j++) {
                    if (pearls.get(j).x>x) {
                        x = pearls.get(j).x;
                        closest = j;
                    }
                }
                sortedPearls[i] = pearls.get(closest);
                pearls.remove(closest);
            }
        }



        ArrayList<Node> p = new ArrayList<>();

        // gonna find the closest Node for every Pearl
        for (Point pearl: sortedPearls) {
            this.pearls.add(pearl);
            p.add(getClosestNode(pearl));
        }

        return p;
    }

    private boolean collidesWithObstacles(Point p){
        Path2D[] obstacles = info.getScene().getObstacles();
        for (Path2D obstacle : obstacles) {
            if (obstacle.intersects(p.x-3, p.y-3, 6, 6)) {
                return true;
            }
        }
        return false;
    }

    private boolean pathFree(Point start, Point goal) {
        Path2D[] obstacles = info.getScene().getObstacles();
        float dis = getDistance(start, goal);

        for (float j=0; j<=1; j+=1/(dis/10)) {
            double pX = start.x * (1 - j) + goal.x * j;
            double pY = start.y * (1 - j) + goal.y * j;


            for (Path2D obstacle : obstacles) {
                if (obstacle.intersects(pX-3, pY-3, 6, 6)) {
                    return false;
                }
            }
        }

        return true;
    }

    private float seek(Point pos, Point target) {
        double x = target.x - pos.x;
        double y = target.y - pos.y;

        return (float) Math.atan2(y, x);
    }

    private float flee(Point obstacle)  {
        double x = info.getX() - obstacle.x;
        double y = info.getY() - obstacle.y;

        return (float) Math.atan2(y, x);
    }

    private Node[][] buildGraph() {
        float width = info.getScene().getWidth()/10f;
        float height = info.getScene().getHeight()/10f;
        Node[][] graph = new Node[(int) (width+1)][(int) (height+1)];

        for (int y=0; y<=height; y++) {
            for (int x=0; x<=width; x++) {
                int pX = x*10;
                int pY = -y*10;


                Node n = new Node(pX, pY);

                if (collidesWithObstacles(n.coordinates)) {
                    n.valid=false;
                } else {
                    if (x>0) {
                        //left x-1, y
                        connectNodes(n , graph[x-1][y]);
                    }
                    if (y>0) {
                        //up x, y-1
                        connectNodes(n , graph[x][y-1]);
                    }
                }
                graph[x][y] = n;

            }
        }
        return graph;
    }

    private void connectNodes(Node n, Node m){
        if (m.valid) {
            float dis= getDistance(n.coordinates, m.coordinates);
            n.edges.put(m, dis);
            m.edges.put(n, dis);
        }
    }

    private float getDistance(Point start, Point goal) {
        float x = goal.x - start.x;
        float y = goal.y - start.y;

        return  (float) Math.sqrt( Math.pow(x, 2) + Math.pow(y, 2));
    }

    private Node getSmallestCost(ArrayList<Node> Q, Node goal) {
        float cost = Float.MAX_VALUE;
        ArrayList<Node> smallest = new ArrayList<>();

        for (Node n : Q) {
            if (n.cost < cost) {
                cost = n.cost;
                smallest.add(n);
            }
        }

        // if closest still null that means the smallest Cost was Integer.MAX-VALUE
        // in that case just return the first Node
        if (smallest.size() == 0) {
            return Q.get(0);
        }

        // if there are multiple nodes with the same smallest cost
        // return the one who is closest to the goal
        if (smallest.size() > 1) {
            Node closest = null;
            cost = Float.MAX_VALUE;
            for (Node n : smallest) {
                float dis = getDistance(n.coordinates, goal.coordinates);
                if (dis < cost) {
                    cost = dis;
                    closest = n;
                }
            }
            return closest;
        }


        return smallest.get(0);
    }

    private Node getClosestNode(Point pos) {
        int x = pos.x/10;
        int y = -pos.y/10;

        if (!graph[x][y].valid){
            if (graph[x+1][y].valid){
                return graph[x+1][y];
            }
            if (graph[x-1][y].valid){
                return graph[x-1][y];
            }
            if (graph[x][y+1].valid){
                return graph[x][y+1];
            }
            if (graph[x][y-1].valid){
                return graph[x][y-1];
            }
            //get some other
        }

        return graph[x][y];
    }

    private ArrayList<Node> quickestWay(Node start, Node goal) {
        // sort all Nodes
        dijkstra(start, goal);
        ArrayList<Node> way = new ArrayList<>();

        // v ??? Zielknoten
        Node v = goal;
        // Solange v ??? Startknoten
        while (v != start) {
            // Ausgabe: v
            way.add(v);
            // v ??? Vorg??nger von v in F
            v = v.previous;
        }

        // reverse, so the List goes start -> ziel
        Collections.reverse(way);
        return way;
    }

    private void dijkstra(Node start, Node goal) {
        ArrayList<Node> Q = new ArrayList<>();
        ArrayList<Node> F = new ArrayList<>();

        if (start.cost != 0) {
            start.cost = 0;
            start.previous=null;
        }

        // {Knoten#, Kosten, Vorg??nger}
        Q.add(start);

        // Solange Knoten erreichbar [Q nicht leer]
        while (Q.size()!=0) {
            // v ??? Entferne Knoten mit kleinsten Kosten k aus Q
            Node v = getSmallestCost(Q, goal);
            Q.remove(v);

            // F??r alle von v ausgehenden Kanten (v, n)
            for (Node n : v.edges.keySet()) {
                if (n != v) {
                    // Falls n nicht fertig [in Liste F]
                    if (!F.contains(n)) {

                        // Falls n bisher nicht erreichbar [nicht in Q]
                        if (!Q.contains(n)) {
                            // n ist erreichbar mit Kosten ??? [n zu Q hinzuf??gen]
                            n.cost = Float.MAX_VALUE;
                            Q.add(n);
                        }
                        // Falls k + Kantengewicht(v,n) < Kosten von n in Q?
                        if (v.cost + v.edges.get(n) < n.cost) {
                            // Kosten von n in Q ??? k + Kantengewicht(v,n)
                            n.cost = v.cost + v.edges.get(n);
                            // Setze v als Vorg??nger von n
                            n.previous = v;
                        }
                    }
                }
            }

            // v ist fertig [v zu F hinzuf??gen]
            F.add(v);
            if (goal != null && F.contains(start) && F.contains(goal)) return;
        }
    }
}