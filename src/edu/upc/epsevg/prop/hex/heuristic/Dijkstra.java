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
import java.util.*;


/**
 *
 * @author GERARD
 */
public class Dijkstra {
    
    public static class PathResult {
        private List<Point> path;
        private int totalCost;

        public PathResult(List<Point> path, int totalCost) {
            this.path = path;
            this.totalCost = totalCost;
        }

        public List<Point> getPath() {
            return path;
        }

        public int getTotalCost() {
            return totalCost;
        }
    }

    public static PathResult dijkstraShortestPath(HexGraph graph, int player) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(Node::getDistance));
        
        int size = 11;
        
        
        // ** Configuració de nodes incials ** // 
        for (int i = 0; i < size; i++) {
            Point startPoint = (player == 1) ? new Point(0, i) : new Point(i, 0);
            
            Node startNode = graph.getNode(startPoint.x, startPoint.y);
            if (startNode != null) {
                startNode.setDistance(0); 
                pq.add(startNode);
            }
        }
        
        Node endNode = null;
        
        
        // ** Procesament de nodes ** //
        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            
            
            if (currentNode.getState()) continue; 
            currentNode.setState(true);

            if (((player == 1) && (currentNode.getPoint().x == size - 1)) || 
                ((player == -1) && (currentNode.getPoint().y == size - 1))) {
                
                endNode = currentNode;
                break; 
            }
            
            
            // ** Procesem els veïns del node actual... ** //
            for (Node neighbor : currentNode.getNeighbors()) {
                if (neighbor.getState()) continue;

                int weight = calculateEdgeWeight(currentNode, neighbor, player);
                int newDist = currentNode.getDistance() + weight;

                if (newDist < neighbor.getDistance()) {
                    neighbor.setDistance(newDist); 
                    neighbor.setPredecessor(currentNode); 
                    pq.add(neighbor);
                }
            }
        }

        // ** Reconstrucció del camí més curt ** //
        if (endNode != null) {
            List<Point> path = reconstructPath(endNode);
            return new PathResult(path, endNode.getDistance());
        }
        
        return new PathResult(new ArrayList<>(), Integer.MAX_VALUE);
    }

    
    private static int calculateEdgeWeight(Node current, Node neighbor, int player) {
        if (neighbor.getStone() == player) return 1;      
        if (neighbor.getStone() == -player) return 50000; 
        return 2;                                         
    }

    
    private static List<Point> reconstructPath(Node endNode) {
        List<Point> path = new ArrayList<>();
        Node current = endNode;

        while (current != null) {
            path.add(0, current.getPoint());
            current = current.getPredecessor();
        }

        return path;
    }
    
    
}
