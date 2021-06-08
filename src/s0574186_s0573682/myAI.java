package s0574186_s0573682;
import lenz.htw.ai4g.ai.*;
import java.awt.*;
import java.awt.geom.Path2D;
import java.util.*;


public class myAI extends AI {


    Point pos;
    int lastScore = 0;
    Node[][] graph;
    Node lastPearl;
    ArrayList<Point> pearls = new ArrayList<>();
    ArrayList<Node> openPearls;
    ArrayList<Node> way = new ArrayList<>(0);
    ArrayList<Node> airWay = new ArrayList<>(0);


    public myAI(Info info) {
        super(info);
        enlistForTournament(574186, 573682);
        long time = System.currentTimeMillis();
        graph = buildGraph();
        long arrayFillTime = System.currentTimeMillis() - time;
        System.out.println("Filled Array in " + arrayFillTime + "ms");
        openPearls = sortPearls();
    }

    @Override
    public String getName() {
        return "Underwater Jedi";
    }

    @Override
    public Color getColor() {
        return new Color(138, 116, 88);
        //return new Color(44, 130, 129);
    }
    // BALLOON_SET
    // STREAMLINED_WIG
    // MOTORIZED_FLIPPERS
    // CORNER_CUTTER

    // 1 D58o9E4Qif avz
    // 2 eaeIxGwdVa yo
    // 3 07pGyuCN4mE T5ODOIjpTv
    // 4 fljn8vd0qW 6dY8fXG1gr


    @Override
    public PlayerAction update() {
        pos = new Point(info.getX(), info.getY());
        float velocity = info.getMaxVelocity(); //Geschwindigkeit
        float direction;
        float air = info.getAir();

        // if a pearl got collected
        if (info.getScore() > lastScore){
            //check which pearl it was and remove from queues
            for (int i=0; i<openPearls.size(); i++) {
                float d = getDistance(pos, pearls.get(i));
                if (d < 20) {
                    lastPearl = openPearls.get(i);
                    way.remove(openPearls.get(i));
                    openPearls.remove(openPearls.get(i));
                    pearls.remove(i);
                    break;
                }
            }
            lastScore = info.getScore();

            //look if there is enough air to get to the next pearl
            if (air < Math.abs(openPearls.get(0).topCost*deep())+getDistance(pos, pearls.get(0))) {
                //if not swim up
                airWay = lastPearl.topPath;
            }
        }

        info.getMaxAir();


        // stick to the surface.....?
        if (air < Math.abs(pearls.get(0).y*deep())+getDistance(pos, pearls.get(0)) && getDistance(pos,openPearls.get(0).top.coordinates) > 10 && pos.y==0) {
            direction = seek(pos, openPearls.get(0).top.coordinates);
            return new DivingAction(velocity,direction);
        }

        if (air < Math.abs(pos.y*deep()) && airWay.size()==0) {
            airWay = lastPearl.topPath;
        }

        if (airWay.size() > 0) {
            // while next is direct reachable, yeet the current
            while (way.size() > 1 && pathFree(pos, way.get(1).coordinates)) {
                way.remove(0);
            }
            // if ya close enough to the next Node OR ya got yer air back --> yeet
            if ((airWay.size() > 1 && getDistance(pos, airWay.get(0).coordinates) < 10)
                    || (airWay.size() == 1 && air == info.getMaxAir())) {
                airWay.remove(0);
            }
            // else follow de nodes
            if (airWay.size() > 0) {
                direction = seek(pos, airWay.get(0).coordinates);
                return new DivingAction(velocity, direction);
            }
            // if ya got yer air back, we now need a new path to the pearl
            way.clear();
            way = quickestWay(getClosestNode(pos), openPearls.get(0));
        }
        //way wird überschrieben aber es wird in die schleife gegriffen
        boolean airUpgrade = false;
        if(info.getFortune()< 2 && !airUpgrade) {
            System.out.println("zieht das?");
            way = quickestWay(getClosestNode(pos), getClosestNode(getClosestTrash(pos)));

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

        return new DivingAction(velocity, direction);
    }


    private float deep() {
        float coeff = 0.72f;
        if (openPearls.size() == 1) {
            return 0;
        }
        else return coeff;
    }

    //get closest Pearl
    //ArrayList<Point> pearls =  new ArrayList<>(Arrays.asList(info.getScene().getPearl()));
    //goal = getPearl(pos);
    private Point getClosestTrash(Point pos) {
        Point[] trashPos = info.getScene().getRecyclingProducts();
        //if (pos==null) pos = new Point(info.getScene().getWidth()/2, 0);

        float distance = Integer.MAX_VALUE;
        Point closest = null;

        for (Point trash : trashPos) {
            float dis = getDistance(pos, trash);
            if (dis < distance) {
                closest = trash;
                distance = dis;
            }
        }

        return closest;
    }

    // sort in one direction, deeps last
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
        int amountDeepPearls = deeps.size();
        for (int i=0; i<amountDeepPearls; i++) {
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
            Node n = getClosestNode(pearl);
            p.add(n);
            // also search the quickest path upwards
            n.topPath = getTopPath(n.coordinates);
            n.top =n.topPath.get(n.topPath.size()-1);

            if (n.topPath.size() > 1) {
                // gotta set the cost right
                // for that we need the second to last node of the top Path
                Node m = n.topPath.get(n.topPath.size() - 2);
                // cost of that + (--) the deepness of it
                n.topCost = m.cost - m.coordinates.y;
            } else
                n.topCost = -n.coordinates.y;
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

    private ArrayList<Node> getTopPath(Point pearl) {
        ArrayList<Node> path = new ArrayList<>();
        Point top = new Point(pearl.x, 0);
        Node pearlNode= getClosestNode(pearl);
        Node topNode = getClosestNode(top);

        if (pathFree(pearl, top)) {
            path.add(topNode);
        } else {
            //path = quickestWay(getClosestNode(pearl), getClosestNode(top));

            dijkstra(topNode, pearlNode);
            Node v = pearlNode;
            while (v != topNode) {
                if (pathFree(v.coordinates, new Point(v.coordinates.x, 0))) {
                    path.add(v);
                    path.add(v.previous);
                    path.add(getClosestNode(new Point(v.coordinates.x, 0)));
                    break;
                } else {
                    path.add(v);
                    v = v.previous;
                }
            }
        }

        return path;
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

        // v ← Zielknoten
        Node v = goal;
        // Solange v ≠ Startknoten
        while (v != start) {
            // Ausgabe: v
            way.add(v);
            // v ← Vorgänger von v in F
            v = v.previous;
        }

        // reverse, so the List goes start -> ziel
        Collections.reverse(way);
        return way;
    }

    //TODO maybe wanna use A* instead
    private void dijkstra(Node start, Node goal) {
        ArrayList<Node> Q = new ArrayList<>();
        ArrayList<Node> F = new ArrayList<>();

        if (start.cost != 0) {
            start.cost = 0;
            start.previous=null;
        }

        // {Knoten#, Kosten, Vorgänger}
        Q.add(start);

        // Solange Knoten erreichbar [Q nicht leer]
        while (Q.size()!=0) {
            // v ← Entferne Knoten mit kleinsten Kosten k aus Q
            Node v = getSmallestCost(Q, goal);
            Q.remove(v);

            // Für alle von v ausgehenden Kanten (v, n)
            for (Node n : v.edges.keySet()) {
                if (n != v) {
                    // Falls n nicht fertig [in Liste F]
                    if (!F.contains(n)) {

                        // Falls n bisher nicht erreichbar [nicht in Q]
                        if (!Q.contains(n)) {
                            // n ist erreichbar mit Kosten ∞ [n zu Q hinzufügen]
                            n.cost = Float.MAX_VALUE;
                            Q.add(n);
                        }
                        // Falls k + Kantengewicht(v,n) < Kosten von n in Q?
                        if (v.cost + v.edges.get(n) < n.cost) {
                            // Kosten von n in Q ← k + Kantengewicht(v,n)
                            n.cost = v.cost + v.edges.get(n);
                            // Setze v als Vorgänger von n
                            n.previous = v;
                        }
                    }
                }
            }

            // v ist fertig [v zu F hinzufügen]
            F.add(v);
            if (goal != null && F.contains(start) && F.contains(goal)) return;
        }
    }
}

