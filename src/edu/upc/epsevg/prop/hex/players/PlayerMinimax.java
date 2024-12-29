package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.heuristic.Dijkstra;
import edu.upc.epsevg.prop.hex.heuristic.HexGraph;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import static edu.upc.epsevg.prop.hex.heuristic.Dijkstra.reconstructPaths;
import edu.upc.epsevg.prop.hex.heuristic.HexGraph.Node;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Minimax a profundidad fija con:
 *  - Poda Alfa-Beta
 *  - Tablas de Transposición (Zobrist hashing)
 *  - Move Ordering (ordenación de movimientos)
 *  - Evaluación con Dijkstra
 */
public class PlayerMinimax implements IPlayer, IAuto {
    
    private String name;
    private int depth;            // Profundidad fija
    private int player;           // player=1 (jugador1) o -1 (jugador2)
    private PlayerType playertype;
    private boolean poda = true;  // Activar/desactivar poda

    // Contador de hojas evaluadas (para debug)
    private int leafCount;

    // ================================
    //       TABLA DE TRANSPOSICIÓN
    // ================================
    private Map<Long, TTEntry> transpositionTable;

    // ================================
    //          ZOBRIST HASHING
    // ================================
    // Para un tablero 11x11 y 3 posibles estados: 0(vacío), 1(player1), -1(player2)
    // Mapearemos:  0 -> index=0, 1 -> index=1, -1 -> index=2
    private long[][][] zobristTable; 

    // -----------------------------------------------------------------------------------
    // CLASE AUXILIAR: Entrada de la Tabla de Transposición
    // -----------------------------------------------------------------------------------
    private static class TTEntry {
        // Flag del nodo (típico en transposition tables)
        enum Flag {
            EXACT,   // Valor exacto
            LOWER,   // Límite inferior (NODE ALPHA)
            UPPER    // Límite superior (NODE BETA)
        }
        int value;   // Valor almacenado
        int depth;   // Profundidad a la que se obtuvo este valor
        Flag flag;   // Tipo de nodo (EXACT, LOWER, UPPER)
        int alpha;   // Alfa en el momento de almacenar
        int beta;    // Beta en el momento de almacenar

        TTEntry(int value, int depth, Flag flag, int alpha, int beta) {
            this.value = value;
            this.depth = depth;
            this.flag = flag;
            this.alpha = alpha;
            this.beta = beta;
        }
    }

    // -----------------------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------------------
    public PlayerMinimax(int depth) {
        this.name = "PlayerMinimax_Transposition";
        this.depth = depth;
        
        // Inicializamos la Tabla de Transposición
        this.transpositionTable = new HashMap<>();
        
        // Iniciamos Zobrist hashing con valores aleatorios
        initZobrist(11);
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * Si hubiera límite de tiempo, se podría usar. Aquí no hacemos nada.
     */
    @Override
    public void timeout() {
        // No action
    }

    // -----------------------------------------------------------------------------------
    // MÉTODO PRINCIPAL: Escoge movimiento usando Minimax con profundidad fija
    // -----------------------------------------------------------------------------------
    @Override
    public PlayerMove move(HexGameStatus s) {
        // Identificamos si somos player1 o player2
        player = s.getCurrentPlayerColor(); 
        if (player == 1) {
            playertype = PlayerType.PLAYER1;
        } else {
            playertype = PlayerType.PLAYER2;
        }
        
        // Reseteamos contador de hojas
        leafCount = 0;
        
        // Variables para la raíz de minimax
        int bestValue = Integer.MIN_VALUE;
        int alpha = Integer.MIN_VALUE;
        int beta  = Integer.MAX_VALUE;
        
        Point bestMove = new Point(0,0);
        
        // Generamos y ordenamos los movimientos (move ordering)
        List<Point> moves = generateOrderedMoves(s, true);
        
        // Recorremos cada movimiento en profundidad "depth"
        for (Point mv : moves) {
            // Clonamos el estado y aplicamos la jugada
            HexGameStatus newBoard = new HexGameStatus(s);
            newBoard.placeStone(mv);
            
            // Llamamos a Minimax (nivel del rival => isMaximizing=false)
            int value = minMax(newBoard, depth - 1, false, alpha, beta, mv);
            
            HexGraph TEST = new HexGraph(11, newBoard, player);
            Dijkstra.dijkstraShortestPath(TEST, player);
            int pepe =  evaluateBoard(newBoard, mv);
            
            // Actualizamos mejor valor
            if(value > bestValue) {
                bestValue = value;
                bestMove = mv;
            }
            alpha = Math.max(alpha, bestValue);
            
            // Poda
            if (beta <= alpha && poda) {
                break;
            }
        }
        
        System.out.println("LeafCount (nodos hoja evaluados) = " + leafCount);
        System.out.println("Mejor valor de la raíz = " + bestValue);
        
        // Devolvemos el movimiento calculado
        return new PlayerMove(bestMove, 0, 0, SearchType.MINIMAX);
    }

    // -----------------------------------------------------------------------------------
    // FUNCIÓN RECURSIVA MINIMAX CON PODA Y TT
    // -----------------------------------------------------------------------------------
    private int minMax(HexGameStatus board, int depth, boolean isMaximizing, int alpha, int beta, Point move) {
        // 1) Consultar la Tabla de Transposición
        long key = computeZobristKey(board);
        TTEntry ttEntry = transpositionTable.get(key);
        
        if(ttEntry != null && ttEntry.depth >= depth) {
            // Ya tenemos un valor calculado a esta profundidad o mayor
            switch(ttEntry.flag) {
                case EXACT:
                    return ttEntry.value;
                case LOWER:
                    // valor >= ttEntry.value
                    alpha = Math.max(alpha, ttEntry.value);
                    break;
                case UPPER:
                    // valor <= ttEntry.value
                    beta = Math.min(beta, ttEntry.value);
                    break;
            }
            if(alpha >= beta && poda) {
                return ttEntry.value;
            }
        }
        
        // 2) Comprobamos si se acabó la partida o la profundidad
        if(board.isGameOver()) {
            // Ajusta la escala según prefieras: multiplicar por (depth+1) premia ganar antes
            if(board.GetWinner() == playertype) {
                return 5555555 * (depth+1);
            } else {
                return -5555555 * (depth+1);
            }
        }
        if(depth == 0 || board.getMoves().isEmpty()) {
            int val = evaluateBoard(board, move);
            // Lo guardamos en TT como EXACT
            transpositionTable.put(key, new TTEntry(val, depth, TTEntry.Flag.EXACT, alpha, beta));
            return val;
        }
        
        // 3) Minimax
        int originalAlpha = alpha;
        int bestValue = isMaximizing ? Integer.MIN_VALUE : Integer.MAX_VALUE;
        
        // Generamos y ordenamos los movimientos
        List<Point> moves = generateOrderedMoves(board, isMaximizing);

        if(isMaximizing) {
            for(Point mv : moves) {
                HexGameStatus newBoard = new HexGameStatus(board);
                newBoard.placeStone(mv);

                int eval = minMax(newBoard, depth - 1, false, alpha, beta, move);
                bestValue = Math.max(bestValue, eval);
                alpha = Math.max(alpha, bestValue);

                if(beta <= alpha && poda) {
                    break;
                }
            }
        } else {
            for(Point mv : moves) {
                HexGameStatus newBoard = new HexGameStatus(board);
                newBoard.placeStone(mv);

                int eval = minMax(newBoard, depth - 1, true, alpha, beta, move);
                bestValue = Math.min(bestValue, eval);
                beta = Math.min(beta, bestValue);

                if(beta <= alpha && poda) {
                    break;
                }
            }
        }

        // 4) Guardar en TT con el flag adecuado
        TTEntry.Flag flag;
        if(bestValue <= originalAlpha) {
            flag = TTEntry.Flag.UPPER; // valor <= alpha
        } else if(bestValue >= beta) {
            flag = TTEntry.Flag.LOWER; // valor >= beta
        } else {
            flag = TTEntry.Flag.EXACT; 
        }
        
        transpositionTable.put(key, new TTEntry(bestValue, depth, flag, alpha, beta));
        
        return bestValue;
    }

    // -----------------------------------------------------------------------------------
    // EVALUACIÓN CON DIJKSTRA
    // -----------------------------------------------------------------------------------
    private int evaluateBoard(HexGameStatus board, Point movement) {
        leafCount++;  

        int gamma = 20;
        int delta = 1;
        
        // Grafo para "nuestro" color
        HexGraph a = new HexGraph(11, board, player);
        Dijkstra.dijkstraShortestPath(a, player);
        
        Node myStartNode = (player == 1) 
            ? a.getNode(-1, 5) 
            : a.getNode(5, -1);
        

        Node myLastNode = (player == 1) 
            ? a.getNode(11, 5) 
            : a.getNode(5, 11);
        

        int myMinPathCost = myLastNode.getDistance();
        int myAltPaths = reconstructPaths(myStartNode, myLastNode);
        
        
        // Grafo para el rival
        HexGraph b = new HexGraph(11, board, -player);
        Dijkstra.dijkstraShortestPath(b, -player);
        
        Node oppStartNode = (player == 1) 
            ? b.getNode(5, -1) 
            : b.getNode(-1, 5);
        
        Node oppLastNode = (player == 1) 
            ? b.getNode(5, 11) 
            : b.getNode(11, 5);
        
        
        int oppMinCost  = oppLastNode.getDistance();
        int oppAltPaths = reconstructPaths(oppStartNode, oppLastNode);

        // Fórmula simple: 
        //  (Mis factores) - (Factores del rival)
        return (-gamma * myMinPathCost + delta * myAltPaths) - 3*(-gamma * oppMinCost + delta * oppAltPaths);
    }

    // -----------------------------------------------------------------------------------
    // MOVE ORDERING: generamos movimientos y los ordenamos
    // -----------------------------------------------------------------------------------
    private List<Point> generateOrderedMoves(HexGameStatus board, boolean isMaximizing) {
        List<Point> moves = new ArrayList<>();
        int size = board.getSize();

        // 1) Recolectar celdas vacías
        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                if(board.getPos(x,y) == 0) {
                    moves.add(new Point(x,y));
                }
            }
        }

        // 2) Ordenar. Ejemplo: cercanía al centro.
        double cx = size / 2.0;
        double cy = size / 2.0;

        moves.sort(Comparator.comparingDouble((Point p) -> {
            double dx = p.x - cx;
            double dy = p.y - cy;
            return Math.hypot(dx, dy);
        }));
        
        // Si prefieres que el MAX explore primero las casillas más cercanas,
        // pero el MIN las más alejadas, podrías invertir el orden cuando isMaximizing = false.
        // Por ejemplo:
        // if(!isMaximizing) Collections.reverse(moves);

        return moves;
    }

    // -----------------------------------------------------------------------------------
    // INICIALIZACIÓN DE ZOBRIST
    // -----------------------------------------------------------------------------------
    private void initZobrist(int size) {
        Random rand = new Random();
        zobristTable = new long[size][size][3]; 
        // 3 estados => index 0(vacío), 1(player1), 2(player2) 
        // pero OJO: en nuestro caso "player2" es -1, haremos un mapping manual.

        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                for(int st=0; st<3; st++) {
                    // st = 0 (vacío), 1 (player1), 2 (player2)
                    zobristTable[x][y][st] = rand.nextLong();
                }
            }
        }
    }

    // -----------------------------------------------------------------------------------
    // CÁLCULO DE LA CLAVE ZOBRIST DE UN TABLERO
    // -----------------------------------------------------------------------------------
    private long computeZobristKey(HexGameStatus board) {
        long h = 0L;
        int size = board.getSize();
        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                int c = board.getPos(x,y); // c = 0, 1, -1
                int st; 
                if(c == 0) {
                    st = 0; // vacío
                } else if(c == 1) {
                    st = 1; // player1
                } else {
                    // c == -1 => player2
                    st = 2;
                }
                // XOR con el valor zobrist correspondiente
                h ^= zobristTable[x][y][st];
            }
        }
        return h;
    }
}
