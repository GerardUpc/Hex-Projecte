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
 * Minimax de profunditat fixa amb:
 *  - Poda Alfa-Beta
 *  - Taules de transposició (Zobrist hashing)
 *  - Move Ordering (ordenació de movimients)
 *  - Evaluació amb Dijkstra
 */
public class MecMec_IterativeDepth implements IPlayer, IAuto {
    
    private String name;
    private int depth;             // Profunditat fixada
    private int player;            // player=1 (jugador1) o -1 (jugador2)
    private PlayerType playertype;
    private boolean poda = true;   // Activar/desactivar poda
    private boolean timeoutOccurred;
    private int leafCount;         // Comptador de fulles evaluades

    // ================================
    //       TAULA DE TRANSPOSICIÓ
    // ================================
    private Map<Long, TTEntry> transpositionTable;

    // ================================
    //          ZOBRIST HASHING
    // ================================
    // Per a un tauler 11x11 i 3 posibles estats: 0(buit), 1(player1), -1(player2)
    // Mapegem: 0 -> index=0, 1 -> index=1, -1 -> index=2
    private long[][][] zobristTable; 

    
    // -----------------------------------------------------------------------------------
    // CLASE AUXILIAR: Entrada de la Taula de Transposició
    // -----------------------------------------------------------------------------------
    private static class TTEntry {
        // Flag del node
        enum Flag {
            EXACT,   // Valor exacte
            LOWER,   // Limit inferior (NODE ALPHA)
            UPPER    // Limit superior (NODE BETA)
        }
        int value;   // Valor emmagatzemat 
        int depth;   // Profunditat a la que es va obtenir el valor
        Flag flag;   // Tipus de node (EXACT, LOWER, UPPER)
        int alpha;   // Alfa en el moment d'emmagatzemar
        int beta;    // Beta en el moment d'emmagatzemar

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
    
    /**
     * Constructor que inicialitza els paràmetres del jugador, 
     * incloent la profunditat fixa de cerca, taules de transposició i hashing Zobrist.
     *
     * @param depth profunditat fixa de cerca
     */
    public MecMec_IterativeDepth(int depth) {
        this.name = "MecMec_IterativeDepth";
        this.depth = depth;
         this.timeoutOccurred = false;
        this.transpositionTable = new HashMap<>();
        initZobrist(11);
    }
    
    
    /**
     * Retorna el nom del jugador.
     *
     * @return el nom del jugador ("MecMec_FixedDepth")
     */
    @Override
    public String getName() {
        return name;
    }


     @Override
    public void timeout() {
        // No action
        timeoutOccurred = true;
        //System.out.println("Timeout detectat: el joc ha excedit el límit de temps!");
    }

    
    // -------------------------------------------------------------------------------------------------------------------
    // METODE PRINCIPAL: Construeix l'arbre de decisions desde l'arrel, amb 121 fills, el escollit sera el millor moviment
    // -------------------------------------------------------------------------------------------------------------------
    
    /**
     * Determina el millor moviment utilitzant l'algorisme Minimax amb poda Alfa-Beta.
     *
     * @param s estat actual del joc Hex
     * @return el moviment escollit amb informació addicional
     */
     @Override
    public PlayerMove move(HexGameStatus s) {
        int actualdepth = 0;
        this.timeoutOccurred = false;
        player = s.getCurrentPlayerColor(); 
        if (player == 1) {
            playertype = PlayerType.PLAYER1;
        } else {
            playertype = PlayerType.PLAYER2;
        }

        leafCount = 0;

        Point bestMove = new Point(0, 0);
        int bestValue = Integer.MIN_VALUE;

        // Iterative Deepening Loop
        for (int currentDepth = 1; ; currentDepth++) {
            int alpha = Integer.MIN_VALUE;
            int beta = Integer.MAX_VALUE;
            int currentBestValue = Integer.MIN_VALUE;
            Point currentBestMove = new Point(0, 0);

            List<Point> moves = generateOrderedMoves(s);

            for (Point mv : moves) {
                if (timeoutOccurred) {
                    break;
                }

                HexGameStatus newBoard = new HexGameStatus(s);
                newBoard.placeStone(mv);

                int value = minMax(newBoard, currentDepth - 1, false, alpha, beta, mv);

                if (value > currentBestValue) {
                    currentBestValue = value;
                    currentBestMove = mv;
                }

                alpha = Math.max(alpha, currentBestValue);

                if (beta <= alpha && poda) {
                    break;
                }
            }

            if (timeoutOccurred) {
                    break;
            }

            // Update best move and best value after each depth search
            bestMove = currentBestMove;
            bestValue = currentBestValue;
            actualdepth++;
        }

        return new PlayerMove(bestMove, leafCount, actualdepth, SearchType.MINIMAX);
    }

    
    
    // -----------------------------------------------------------------------------------
    // FUNCIÓ RECURSIVA MINIMAX AMB PODA I TT
    // -----------------------------------------------------------------------------------
    
    /**
     * Implementa l'algorisme Minimax amb poda Alfa-Beta i suport per taules de transposició.
     *
     * @param board estat del tauler
     * @param depth profunditat restant
     * @param isMaximizing indica si el node actual és de maximització
     * @param alpha valor alfa per a la poda
     * @param beta valor beta per a la poda
     * @param move moviment actual considerat
     * @return el valor heurístic del node
     */
    private int minMax(HexGameStatus board, int depth, boolean isMaximizing, int alpha, int beta, Point move) {
        if (timeoutOccurred) {
                    return 0;
                }
        // 1) Consultem la taula de transposició
        long key = computeZobristKey(board);
        TTEntry ttEntry = transpositionTable.get(key);
        
        if(ttEntry != null && ttEntry.depth >= depth) {
            // Ja tenim un valor calculat a aquesta profundidat o major
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
        
        // 2) Comprovem si s'ha acabat la partida o la profunditat
        if(board.isGameOver()) {
            if(board.GetWinner() == playertype) {
                return 100000 * (depth);
            } else {
                return -100000 * (depth);
            }
        }
        if(depth == 0 || board.getMoves().isEmpty()) {
            int val = evaluateBoard(board);
            // Ho guardem a TT com EXACT
            transpositionTable.put(key, new TTEntry(val, depth, TTEntry.Flag.EXACT, alpha, beta));
            return val;
        }
        
        // 3) Minimax
        int originalAlpha = alpha;
        int bestValue = isMaximizing ? Integer.MIN_VALUE : Integer.MAX_VALUE;
        
        // Generem i ordenem els moviments
        List<Point> moves = generateOrderedMoves(board);

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

        // 4) Ho guardem a TT amb el flag adequat
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
    // EVALUACIÓ AMB DIJKSTRA
    // -----------------------------------------------------------------------------------
    
    /**
     * Avalua l'estat del tauler utilitzant l'algorisme de Dijkstra per calcular camins curts i alternatius.
     *
     * @param board estat actual del tauler
     * @return un valor heurístic que representa la qualitat del tauler
     */
    private int evaluateBoard(HexGameStatus board) {
        leafCount++;  
        
        int remainingMoves = board.getMoves().size();
        
       double[] factors = calculateDynamicFactors(121, remainingMoves);
       double delta = factors[0];
       double gamma = factors[1];

       int minPathCostRange = 22;
       int maxPathCostRange = 110; 
       
       int minAltPathsRange = 1;
       int maxAltPathsRange = 1500; 

       // Graf per al "nostre" color
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

        // Normalització per al "nostre" costat
        double myNormalizedMinPathCost = normalizeTo100(myMinPathCost, minPathCostRange, maxPathCostRange);
        double myNormalizedAltPaths = normalizeTo100(myAltPaths, minAltPathsRange, maxAltPathsRange);

        // Graf per al rival
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

        // Normalització per al rival
        double oppNormalizedMinCost = normalizeTo100(oppMinCost, minPathCostRange, maxPathCostRange);
        double oppNormalizedAltPaths = normalizeTo100(oppAltPaths, minAltPathsRange, maxAltPathsRange);

        // Fórmula normalitzada
        return (int)((-gamma * myNormalizedMinPathCost + delta * myNormalizedAltPaths)
                 - 6*(-gamma * oppNormalizedMinCost + delta * oppNormalizedAltPaths));
    }

    
    /**
     * Normalitza un valor al rang [0, 100].
     *
     * @param value valor a normalitzar
     * @param min valor mínim del rang
     * @param max valor màxim del rang
     * @return valor normalitzat
     */
    private double normalizeTo100(int value, int min, int max) {
        if (max - min == 0) return 0; // Per evitar divisió per zero
        return (double)(value - min) / (max - min) * 100;
    }

    /**
     * Calcula els valors dinàmics de delta i gamma basats en el progrés de la partida.
     *
     * @param totalMoves Nombre total de moviments al principi de la partida.
     * @param remainingMoves Nombre de moviments que queden.
     * @return Un array de dos valors: [delta, gamma].
     */
    private double[] calculateDynamicFactors(int totalMoves, int remainingMoves) {
        // Configuració inicial i final dels valors
        double deltaInitial = 8.0;
        double deltaFinal = 1.0;
        double gammaInitial = 1.0;
        double gammaFinal = 3.0;

        // Percentatge de partida restant
        double progress = (double) remainingMoves / totalMoves;

        // Càlcul de delta (disminueix de forma quadràtica)
        double delta = deltaFinal + (deltaInitial - deltaFinal) * Math.pow(progress, 2);

        // Càlcul de gamma (augmenta de forma lineal)
        double gamma = gammaInitial + (gammaFinal - gammaInitial) * (1 - progress);

        return new double[]{1.0, 3.0};
    }

    
    // -----------------------------------------------------------------------------------
    // MOVE ORDERING: generem movimients i els ordenem
    // -----------------------------------------------------------------------------------
    
    /**
     * Genera i ordena els moviments possibles basant-se en la proximitat al centre del tauler.
     *
     * @param board estat actual del tauler
     * @return llista de moviments ordenats
     */
    private List<Point> generateOrderedMoves(HexGameStatus board) {
        List<Point> moves = new ArrayList<>();
        int size = board.getSize();

        // 1) Recolectar celes buides
        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                if(board.getPos(x,y) == 0) {
                    moves.add(new Point(x,y));
                }
            }
        }

        // 2) Ordenem per proximitat al centre
        double cx = size / 2.0;
        double cy = size / 2.0;

        moves.sort(Comparator.comparingDouble((Point p) -> {
            double dx = p.x - cx;
            double dy = p.y - cy;
            return Math.hypot(dx, dy);
        }));
        
        return moves;
    }
    
    
    // -----------------------------------------------------------------------------------
    // INICIALITZACIÓ DE ZOBRIST
    // -----------------------------------------------------------------------------------
    
    /**
     * Inicialitza els valors de la taula Zobrist per a un tauler d'una mida donada.
     *
     * @param size mida del tauler
     */
    private void initZobrist(int size) {
        Random rand = new Random();
        zobristTable = new long[size][size][3]; 
        // 3 estados => index 0(vacío), 1(player1), -1(player2) 

        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                for(int st=0; st<3; st++) {
                    // st = 0(vacío), 1(player1), -1(player2)
                    zobristTable[x][y][st] = rand.nextLong();
                }
            }
        }
    }

    
    // -----------------------------------------------------------------------------------
    // CÁLCUL DE LA CLAU ZOBRIST D'UN TAULER
    // -----------------------------------------------------------------------------------
    
    /**
     * Calcula la clau Zobrist corresponent a l'estat actual del tauler.
     *
     * @param board estat del tauler
     * @return clau Zobrist
     */
    private long computeZobristKey(HexGameStatus board) {
        long h = 0L;
        int size = board.getSize();
        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                int c = board.getPos(x,y); // c = 0, 1, -1
                int st; 
                if(c == 0) {
                    st = 0; // buit
                } else if(c == 1) {
                    st = 1; // player1
                } else {
                    // c == -1 => player2
                    st = 2;
                }
                // XOR amb el valor zobrist corresponent
                h ^= zobristTable[x][y][st];
            }
        }
        return h;
    }
 
}
