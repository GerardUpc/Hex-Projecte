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

/**
 *
 * Implementació d'una hashtable per a emmagatzemar el graf del tauler, el punt (x, y) del node es la clau del node.
 * 
 * Hi ha dos super nodes, depenent de qui s'analitzi el tauler, A i A' o B i B' 
 * ¡Suposem que el tauler es de 11x11!
 * 
 */
public class HexGraph {
    
    
    // =================================
    //   Auxiliary data structure Node
    // =================================
    public class Node {
    
        /* Basic data */
        private Point point;              // Cordenades x y             
        private int stone;                // 0 = buit, 1 = player 1, -1 = player 2
    
    
        /* Dijkstra data */
        private boolean state;           
        private int distance;             // Distancia desde el punt inicial
        private List<Node> predecessors;  // Node anterior al camí més curt
        private List<Node> neighbors;

        /* Constructor */
        public Node(int x, int y, int stone) {
            this.point = new Point(x, y);
            this.stone = stone;
        
            this.state = false;
            this.distance = Integer.MAX_VALUE;
            this.predecessors = new ArrayList<>();
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
    
        public List<Node> getPredecessor() {
            return predecessors;
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
    

        /* Other methods */
   
        public void addNeighbor(Node neighbor) {
            neighbors.add(neighbor);
        }
    
        public void addPredecessor(Node predecessor) {
            predecessors.add(predecessor);
        }
    
        public void clearPredecessors() {
            predecessors.clear();
        }
    
        @Override
        public String toString() {
            return "Node{" +
                    "point=" + point +
                    ", stone=" + stone +
                    ", neighbors=" + neighbors.size() +
                    ", state=" + state + 
                    ", predecesors=" + predecessors.size() +
                    '}';
        }
    }
    
    public Map<Point, Node> nodes;
    
    
    /**
     * Constructor del graf.
     * Inicialitza els nodes del graf en funció de l'estat del tauler i afegeix les connexions entre ells.
     *
     * @param size   mida del tauler (especificat com 11x11 per defecte)
     * @param s      estat actual del joc Hex
     * @param player jugador actual (1 o -1)
     */
    public HexGraph(int size, HexGameStatus s, int player) {
        
        this.nodes = new HashMap<>();
        initializeHexBoard(size, s, player);
    }
    
    
    /**
     * Inicialitza el tauler hexagonal creant els nodes i connectant-los amb els seus veïns.
     * També afegeix supernodes en funció del jugador que s'analitza.
     *
     * @param size   mida del tauler
     * @param s      estat actual del joc Hex
     * @param player jugador actual (1 o -1)
     */
    private void initializeHexBoard(int size, HexGameStatus s, int player) {
    
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                
                Point point = new Point(x, y);
                
                nodes.put(point, new Node(x, y, s.getPos(point)));
                
            }
        }
        
        /* Creació dels super nodes... */ 
        int[][] cordinates = {
            {-1, 5}, {11, 5}, {5, -1}, {5, 11}
        };
        
        for(int[] cord : cordinates) {
            
            Point point = new Point(cord[0], cord[1]);
                           
            nodes.put(point, new Node(cord[0], cord[1], 10));  // Valor de la stone a 10 suposa un supernode.
            
        }
        
        /* Connexió dels veïns... */
        int[][] directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, 1}, {1, -1}
        };

        for (Node node : nodes.values()) {
            
            if(node.getStone() != 10) {
                if(player == 1){
                    /* NODE A */
                    if(node.getPoint().x == 0) {
                        Point superNodePoint = new Point(-1, 5);
                        node.addNeighbor(nodes.get(superNodePoint));
                    }
                    /* NODE A' */
                    if(node.getPoint().x == 10) {
                        Point superNodePoint = new Point(11, 5);
                        node.addNeighbor(nodes.get(superNodePoint));
                    }
                } else {
                    /* NODE B */
                    if(node.getPoint().y == 0) {
                        Point superNodePoint = new Point(5, -1);
                        node.addNeighbor(nodes.get(superNodePoint));
                    }
                    /* NODE B' */
                    if(node.getPoint().y == 10) {
                        Point superNodePoint = new Point(5, 11);
                        node.addNeighbor(nodes.get(superNodePoint));
                    }
                }
                
                for (int[] dir : directions) {
                    int nx = node.getPoint().x + dir[0];
                    int ny = node.getPoint().y + dir[1];
                
                    if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                        Point neighborPoint = new Point(nx, ny);
                        node.addNeighbor(nodes.get(neighborPoint));
                    }
                }
            }
            else {
                if(node.getPoint().y == 5) {
                    /* NODE A */
                    if(node.getPoint().x == -1) {
                        for(int i = 0; i < size; i++){ 
                            Point neighborPoint = new Point(0, i);
                            node.addNeighbor(nodes.get(neighborPoint));   
                        }
                    }
                    else {
                    /* NODE A' */
                        for(int i = 0; i < size; i++){ 
                            Point neighborPoint = new Point(10, i);
                            node.addNeighbor(nodes.get(neighborPoint));   
                        }
                    }
                }
                else if (node.getPoint().x == 5) {
                    /* NODE B */
                    if(node.getPoint().y == -1) {
                        for(int i = 0; i < size; i++){ 
                            Point neighborPoint = new Point(i, 0);
                            node.addNeighbor(nodes.get(neighborPoint));   
                        }
                    }
                    else {
                    /* NODE B' */
                        for(int i = 0; i < size; i++){ 
                            Point neighborPoint = new Point(i, 10);
                            node.addNeighbor(nodes.get(neighborPoint));   
                        }
                    }
                }
            }       
        }
    }  
    
    
    /**
     * Retorna l'estat de la pedra (stone) en un node específic.
     *
     * @param x coordenada X del node
     * @param y coordenada Y del node
     * @return estat de la pedra al node (0, 1, -1 o 999 si no existeix)
     */
    public int getNodeStone(int x, int y) {
        Node node = nodes.get(new Point(x, y));
        return (node != null) ? node.getStone() : 999; 
    }
    
    
    /**
     * Retorna el node en la posició especificada.
     *
     * @param x coordenada X del node
     * @param y coordenada Y del node
     * @return el node en la posició especificada o null si no existeix
     */
    public Node getNode(int x, int y) {
        Node node = nodes.get(new Point(x, y));
        return (node != null) ? node : null;
    }
    
    
    /**
     * Estableix l'estat de la pedra (stone) en un node específic.
     *
     * @param x     coordenada X del node
     * @param y     coordenada Y del node
     * @param stone estat de la pedra a establir
     */
    public void setNodeStone(int x, int y, int stone) {
        Node node = nodes.get(new Point(x, y));
        if (node != null) {
            node.setStone(stone);
        } else {
            System.out.println("El node (" + x + "," + y + ") no existeix.");
        }
    }
    
    
    /**
     * Imprimeix tots els veïns d'un node específic.
     *
     * @param x coordenada X del node
     * @param y coordenada Y del node
     */
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

    
    /**
     * Imprimeix el graf complet, incloent els detalls de tots els nodes.
     */
    public void printGraph() {
        for (Node node : nodes.values()) {
            System.out.println(node);
        }
    }
}
