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
import edu.upc.epsevg.prop.hex.heuristic.HexGraph.Node;
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
    


    public static void dijkstraShortestPath(HexGraph graph, int player) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(Node::getDistance));
        
        int size = 11;
        
        Point superNode = (player == 1) ? new Point(-1, 5) : new Point(5, -1);
        Node startNode = graph.getNode(superNode.x, superNode.y);
        startNode.setDistance(0);
        pq.add(startNode);
        
        
        // ** Configuració de nodes incials ** // 
        /*for (int i = 0; i < size; i++) {
            
            Point startPoint = (player == 1) ? new Point(0, i) : new Point(i, 0);
            
            Node initialNode = graph.getNode(startPoint.x, startPoint.y);

        }*/
        
        
        // ** Procesament de nodes ** //
        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            
            
            if (currentNode.getState()) continue; 
            currentNode.setState(true);

            
            // ** Procesem els veïns del node actual... ** //
            for (Node neighbor : currentNode.getNeighbors()) {
                if (neighbor.getState()) continue;

                int weight = calculateEdgeWeight(currentNode, neighbor, player);
                int newDist = currentNode.getDistance() + weight;

                if (newDist < neighbor.getDistance()) {
                    neighbor.setDistance(newDist); 
                    neighbor.clearPredecessors();
                    neighbor.addPredecessor(currentNode); 
                    pq.add(neighbor);
                }
                else if (newDist == neighbor.getDistance()) {
                    neighbor.addPredecessor(currentNode);
                }    
            }
        }
    }

  
    private static int calculateEdgeWeight(Node current, Node neighbor, int player) {
        if(current.getStone() != 10 && neighbor.getStone() != 10){
            if(neighbor.getStone() == player) return 2;
            else if (neighbor.getStone() == -player) return 9999;
            else return 10;
        }
        else return 0;
    }
    
    
    public static int reconstructPaths(HexGraph.Node startNode, HexGraph.Node endNode) {
        List<List<HexGraph.Node>> allPaths = new ArrayList<>();
        List<HexGraph.Node> currentPath = new ArrayList<>();

        // Reconstruct all paths recursively
        reconstructPathsHelper(endNode, startNode, currentPath, allPaths);

        // Return the number of alternative paths
        return allPaths.size();
    }

    private static void reconstructPathsHelper(HexGraph.Node current, HexGraph.Node startNode, List<HexGraph.Node> currentPath, List<List<HexGraph.Node>> allPaths) {
        currentPath.add(current);

        if (current.equals(startNode)) {
            List<HexGraph.Node> path = new ArrayList<>(currentPath);
            Collections.reverse(path);
            allPaths.add(path);
        } else {
            for (HexGraph.Node predecessor : current.getPredecessor()) {
                reconstructPathsHelper(predecessor, startNode, currentPath, allPaths);
            }
        }

        currentPath.remove(currentPath.size() - 1);
    }
        
}
