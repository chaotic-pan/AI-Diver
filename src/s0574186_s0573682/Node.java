package s0574186_s0573682;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;

public class Node {
    public boolean valid;
    public Point coordinates;
    public HashMap<Node, Float> edges = new HashMap<>();
    public float cost;
    public Node previous;
    ArrayList<Node> topPath;
    Node top;
    float topCost;

    public Node(int x, int y) {
        coordinates = new Point(x,y);
        valid = true;
    }

    public Node(Point coordinates) {
        this.coordinates = coordinates;
        valid = true;
    }
}