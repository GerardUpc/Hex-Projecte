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
 * Implementació de l'algorisme d'avaluació del tauler, partint del supernode inicial, visita tots els nodes fins a
 * trobar el camí minim i els camins minims alternatius fins l'altre supernode.
 * 
 */
public class Dijkstra {
    
    
    /**
    * Implementa l'algorisme de Dijkstra per trobar els camins més curts en un HexGraph.
    * Inicialitza les distàncies, processa els nodes i actualitza els seus veïns.
    *
    * @param graph  el HexGraph en el qual es calculen els camins més curts
    * @param player l'identificador del jugador (1 o -1) utilitzat per determinar els pesos de les arestes
    */
    public static void dijkstraShortestPath(HexGraph graph, int player) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(Node::getDistance));
        
        Point superNode = (player == 1) ? new Point(-1, 5) : new Point(5, -1);
        Node startNode = graph.getNode(superNode.x, superNode.y);
        startNode.setDistance(0);
        pq.add(startNode);
        
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

    
    /**
    * Calcula el pes d'una aresta entre dos nodes en funció del seu estat actual i l'identificador del jugador.
    * Assigna un pes depenent de si el node veí pertany al jugador, a l'oponent o és neutral.
    *
    * @param current  el node actual que s'està processant
    * @param neighbor el node veí al qual s'està avaluant l'aresta
    * @param player   l'identificador del jugador (1 o -1)
    * @return el pes de l'aresta entre el node actual i el veí
    */
    private static int calculateEdgeWeight(Node current, Node neighbor, int player) {
        if(current.getStone() != 10 && neighbor.getStone() != 10){
            if(neighbor.getStone() == player) return 2;
            else if (neighbor.getStone() == -player) return 9999;
            else return 10;
        }
        
        else return 0;
    }
    
    
    /**
    * Reconstrueix i compta tots els camins únics entre un node inicial i un node final en el HexGraph.
    * Aquest mètode retorna el nombre total de camins únics trobats.
    *
    * @param startNode el node on comencen els camins
    * @param endNode   el node on acaben els camins
    * @return el nombre total de camins únics des de startNode fins a endNode
    */
    public static int reconstructPaths(HexGraph.Node startNode, HexGraph.Node endNode) {
        List<List<HexGraph.Node>> allPaths = new ArrayList<>();
        List<HexGraph.Node> currentPath = new ArrayList<>();

        reconstructPathsHelper(endNode, startNode, currentPath, allPaths);

        return allPaths.size();
    }

    
    /**
    * Mètode auxiliar per reconstruir de manera recursiva els camins entre nodes en el HexGraph.
    * Construeix els camins en ordre invers, començant des del node final i travessant fins al node inicial
    * mitjançant els predecessors.
    *
    * @param current     el node actual que s'està processant
    * @param startNode   el node on comencen els camins
    * @param currentPath el camí actual que s'està construint durant la recursió
    * @param allPaths    la llista de tots els camins reconstruïts
    */
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
