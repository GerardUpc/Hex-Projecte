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
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
/**
 *
 * @author alexsaiztalavera
 */
public class PlayerYBWC implements IPlayer, IAuto{
     private String name;
    private int depth;            // Profundidad fija
    private int player;           // player=1 (jugador1) o -1 (jugador2)
    private PlayerType playertype;
    private boolean poda = true;  // Activar/desactivar poda
    private boolean timeoutOccurred;
    // Contador de hojas evaluadas (para debug)
    private int leafCount;
    private ExecutorService executor;

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
    public PlayerYBWC(int depth) {
        this.name = "PlayerMinimax_Transposition";
        this.depth = depth;
        this.timeoutOccurred = false;
        // Inicializamos la Tabla de Transposición
        this.transpositionTable = new HashMap<>();
        
        // Iniciamos Zobrist hashing con valores aleatorios
        initZobrist(11);
        this.executor = Executors.newCachedThreadPool();
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
        timeoutOccurred = true;
        System.out.println("Timeout detectat: el joc ha excedit el límit de temps!");
    }

    // -----------------------------------------------------------------------------------
    // MÉTODO PRINCIPAL: Escoge movimiento usando Minimax con profundidad fija
    // -----------------------------------------------------------------------------------
    @Override
    public PlayerMove move(HexGameStatus s) {
        this.timeoutOccurred = false;
        player = s.getCurrentPlayerColor();
        playertype = (player == 1) ? PlayerType.PLAYER1 : PlayerType.PLAYER2;
        leafCount = 0;

        List<Point> moves = generateOrderedMoves(s, true);
        Point bestMove = executeYBWC(moves, s);

        if (bestMove == null && !moves.isEmpty()) {
            bestMove = moves.get(0);
        }

        return new PlayerMove(bestMove, leafCount, depth, SearchType.MINIMAX);
    }
    
    private Point executeYBWC(List<Point> moves, HexGameStatus state) {
        AtomicInteger alpha = new AtomicInteger(Integer.MIN_VALUE);
        int beta = Integer.MAX_VALUE;
        Point bestMove = null;

        // Processar el primer moviment seqüencialment
        Point firstMove = moves.get(0);
        HexGameStatus firstState = new HexGameStatus(state);
        firstState.placeStone(firstMove);
        int firstValue = minMax(firstState, depth - 1, false, alpha.get(), beta, firstMove);

        if (firstValue > alpha.get()) {
            alpha.set(firstValue);
            bestMove = firstMove;
        }

        // Processar els moviments restants en paral·lel
        ExecutorService executor = Executors.newFixedThreadPool(moves.size() - 1);
        List<Future<MoveEvaluation>> futures = new ArrayList<>();

        for (int i = 1; i < moves.size(); i++) {
            Point mv = moves.get(i);
            futures.add(executor.submit(() -> {
                HexGameStatus newState = new HexGameStatus(state);
                newState.placeStone(mv);
                int eval = minMax(newState, depth - 1, false, alpha.get(), beta, mv);
                return new MoveEvaluation(mv, eval);
            }));
        }

        // Reunim els resultats dels fils
        try {
            for (Future<MoveEvaluation> future : futures) {
                MoveEvaluation result = future.get();
                synchronized (alpha) {
                    if (result.value > alpha.get()) {
                        alpha.set(result.value);
                        bestMove = result.move;
                    }
                    if (beta <= alpha.get()) {
                        break; // Poda alfa-beta
                    }
                }
            }
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
        } finally {
            executor.shutdown();
        }

        return bestMove;
    }
    // -----------------------------------------------------------------------------------
    // FUNCIÓN RECURSIVA MINIMAX CON PODA Y TT
    // -----------------------------------------------------------------------------------
    private int minMax(HexGameStatus board, int depth, boolean isMaximizing, int alpha, int beta, Point move) {
        if (timeoutOccurred || depth == 0 || board.isGameOver()) {
            return evaluateBoard(board,move);
        }

        long key = computeZobristKey(board);
        TTEntry ttEntry = transpositionTable.get(key);
        if (ttEntry != null && ttEntry.depth >= depth) {
            switch (ttEntry.flag) {
                case EXACT:
                    return ttEntry.value;
                case LOWER:
                    alpha = Math.max(alpha, ttEntry.value);
                    break;
                case UPPER:
                    beta = Math.min(beta, ttEntry.value);
                    break;
            }
            if (alpha >= beta) {
                return ttEntry.value;
            }
        }

        int bestValue = isMaximizing ? Integer.MIN_VALUE : Integer.MAX_VALUE;
        List<Point> moves = generateOrderedMoves(board, isMaximizing);

        for (Point mv : moves) {
            HexGameStatus newBoard = new HexGameStatus(board);
            newBoard.placeStone(mv);
            int eval = minMax(newBoard, depth - 1, !isMaximizing, alpha, beta, mv);

            if (isMaximizing) {
                bestValue = Math.max(bestValue, eval);
                alpha = Math.max(alpha, bestValue);
            } else {
                bestValue = Math.min(bestValue, eval);
                beta = Math.min(beta, bestValue);
            }

            if (beta <= alpha) {
                break;
            }
        }

        TTEntry.Flag flag = (bestValue <= alpha) ? TTEntry.Flag.UPPER : (bestValue >= beta) ? TTEntry.Flag.LOWER : TTEntry.Flag.EXACT;
        transpositionTable.put(key, new TTEntry(bestValue, depth, flag, alpha, beta));

        return bestValue;
    }
     
    // -----------------------------------------------------------------------------------
    // MOVE ORDERING: generamos movimientos y los ordenamos
    // -----------------------------------------------------------------------------------
   
    
     private List<Point> generateOrderedMoves(HexGameStatus board, boolean isMaximizing) {
        List<Point> moves = new ArrayList<>();
        int size = board.getSize();

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (board.getPos(x, y) == 0) {
                    moves.add(new Point(x, y));
                }
            }
        }

        moves.sort(Comparator.comparingDouble(p -> {
            double cx = size / 2.0;
            double cy = size / 2.0;
            double dx = p.x - cx;
            double dy = p.y - cy;
            return Math.hypot(dx, dy);
        }));

        return moves;
    }
    
    

    // -----------------------------------------------------------------------------------
    // EVALUACIÓN CON DIJKSTRA
    // -----------------------------------------------------------------------------------
    private int evaluateBoard(HexGameStatus board, Point movement) {
        leafCount++;  
        
        int remainingMoves = board.getMoves().size();
        
       double[] factors = calculateDynamicFactors(121, remainingMoves);
       double delta = factors[0];
       double gamma = factors[1];

        // Rang esperat (ajusta aquests valors segons les característiques del joc)
        int minPathCostRange = 22;
        int maxPathCostRange = 110; // Suposant un màxim cost de camí raonable
        int minAltPathsRange = 1;
        int maxAltPathsRange = 1500; // Suposant un màxim raonable de camins alternatius

    // Grafo per al "nostre" color
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

    // Grafo per al rival
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

    // Fórmula normalitzada: 
    return (int)((-gamma * myNormalizedMinPathCost + delta * myNormalizedAltPaths)
               - 6*(-gamma * oppNormalizedMinCost + delta * oppNormalizedAltPaths));
}

/**
 * Normalitza un valor al rang [0, 100].
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
    private static class MoveEvaluation {
        Point move;
        int value;

        MoveEvaluation(Point move, int value) {
            this.move = move;
            this.value = value;
        }
    }
    
}