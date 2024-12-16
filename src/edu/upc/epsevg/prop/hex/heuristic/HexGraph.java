/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package edu.upc.epsevg.prop.hex.heuristic;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.MoveNode;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.awt.Point;
import java.util.HashMap;
import java.util.Map;


/* Auxiliary data structure Node */
class Node {
    
    /* Basic data */
    private Point point;             // Cordenades x y             
    private int stone;               // 0 = buit, 1 = player 1, -1 = player 2
    
    
    /* Dijkstra data */
    private boolean state;           
    private int distance;            // Distancia desde el punt inicial
    private Node predecessor;        // Node anterior al camí més curt
    private List<Node> neighbors;

    /* Constructor */
    public Node(int x, int y, int stone) {
        this.point = new Point(x, y);
        this.stone = stone;
        
        this.state = false;
        this.distance = Integer.MAX_VALUE;
        this.predecessor = null;
        this.neighbors = new ArrayList<>();

    }

    /* Getters */
    public Point getPoint() {
        return point;
    }

    public int getStone() {
        return stone;
    }
    
    public Boolean getState() {
        return state;
    }
    
    public int getDistance() {
        return distance;
    }
    
    public Node getPredecessor() {
        return predecessor;
    }
    
    public List<Node> getNeighbors() {
        return neighbors;
    }

    
    /* Setters */
    public void setStone(int stone) {
        this.stone = stone;
    }
    
    public void setState(Boolean state) {
        this.state = state;
    }
    
    public void setDistance(int distance) {
        this.distance = distance;
    }
    
    public void setPredecessor(Node predecessor) {
        this.predecessor = predecessor;
    }
    
    
    /* Other methods */
   
    public void addNeighbor(Node neighbor) {
        neighbors.add(neighbor);
    }
    
    @Override
    public String toString() {
        return "Node{" +
                "point=" + point +
                ", stone=" + stone +
                ", neighbors=" + neighbors.size() +
                ", state=" + state + 
                '}';
    }
}

/**
 *
 * Implementation of a hashmap to store the Hex board graph.
 * Every node is the key of their neighbours.
 * 
 * @author GERARD
 */
public class HexGraph {
    private Map<Point, Node> nodes;
    
    /* Constructor */
    public HexGraph(int size, HexGameStatus s) {
        this.nodes = new HashMap<>();
        initializeHexBoard(size, s);
    }

    private void initializeHexBoard(int size, HexGameStatus s) {
    
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                
                Point point = new Point(x, y);
                
                nodes.put(point, new Node(x, y, s.getPos(point)));
                
            }
        }
        
        /* Neighbours connection... */
        int[][] directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, 1}, {1, -1}
        };

        for (Node node : nodes.values()) {
            for (int[] dir : directions) {
                int nx = node.getPoint().x + dir[0];
                int ny = node.getPoint().y + dir[1];
                
                if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                    Point neighborPoint = new Point(nx, ny);
                    node.addNeighbor(nodes.get(neighborPoint));
                }
            }
        }
    }
    
    /* Getters */
    public int getNodeStone(int x, int y) {
        Node node = nodes.get(new Point(x, y));
        return (node != null) ? node.getStone() : 999; 
    }
    
    public Node getNode(int x, int y) {
        Node node = nodes.get(new Point(x, y));
        return (node != null) ? node : null;
    }
    
    
    /* Setters */
    public void setNodeStone(int x, int y, int stone) {
        Node node = nodes.get(new Point(x, y));
        if (node != null) {
            node.setStone(stone);
        } else {
            System.out.println("El node (" + x + "," + y + ") no existeix.");
        }
    }


    /* Other methods */
    public void printNeighbors(int x, int y) {
        Node node = nodes.get(new Point(x, y));
        if (node != null) {
            System.out.println("Veïns del node (" + x + "," + y + "):");
            for (Node neighbor : node.getNeighbors()) {
                System.out.println("  " + neighbor.getPoint());
            }
        } else {
            System.out.println("El node (" + x + "," + y + ") no existeix.");
        }
    }

    public void printGraph() {
        for (Node node : nodes.values()) {
            System.out.println(node);
        }
    }
}
