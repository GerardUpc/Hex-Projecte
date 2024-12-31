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

/**
 * Minimax de profunditat amb timeout amb:
 *  - Poda Alfa-Beta
 *  - Taules de transposició (Zobrist hashing)
 *  - Move Ordering (ordenació de movimients)
 *  - Evaluació amb Dijkstra
 *  - Paralelisme amb YBWC
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
    private long[][][] zobristTable; 

    private static final ExecutorService executor = Executors.newCachedThreadPool();

    // -----------------------------------------------------------------------------------
    // CLASE AUXILIAR: Entrada de la Taula de Transposició
    // -----------------------------------------------------------------------------------
    private static class TTEntry {
        enum Flag {
            EXACT,
            LOWER,
            UPPER
        }
        int value;
        int depth;
        Flag flag;
        int alpha;
        int beta;

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
    public MecMec_IterativeDepth(int depth) {
        this.name = "MecMec_IterativeDepth";
        this.depth = depth;
        this.timeoutOccurred = false;
        this.transpositionTable = new HashMap<>();
        initZobrist(11);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void timeout() {
        timeoutOccurred = true;
    }

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

        for (int currentDepth = 1; currentDepth <= depth; currentDepth++) {
            int alpha = Integer.MIN_VALUE;
            int beta = Integer.MAX_VALUE;
            int currentBestValue = Integer.MIN_VALUE;
            Point currentBestMove = new Point(0, 0);

            List<Point> moves = generateOrderedMoves(s);

            if (moves.isEmpty() || timeoutOccurred) break;

            // Evaluate first move sequentially (Young Brothers Wait Concept)
            Point firstMove = moves.get(0);
            HexGameStatus newBoard = new HexGameStatus(s);
            newBoard.placeStone(firstMove);
            int value = minMax(newBoard, currentDepth - 1, false, alpha, beta, firstMove);
            if (value > currentBestValue) {
                currentBestValue = value;
                currentBestMove = firstMove;
            }
            alpha = Math.max(alpha, currentBestValue);

            // Parallel evaluation of the rest of the moves
            List<Future<int[]>> futures = new ArrayList<>();

            int finalAlpha = alpha;
            int finalBeta = beta;
            int finalCurrentDepth = currentDepth;

            for (int i = 1; i < moves.size(); i++) {
            Point mv = moves.get(i);
            futures.add(executor.submit(() -> {
                if (timeoutOccurred) return null;
                HexGameStatus parallelBoard = new HexGameStatus(s);
                parallelBoard.placeStone(mv);
                int eval = minMax(parallelBoard, finalCurrentDepth - 1, false, finalAlpha, finalBeta, mv);
                return new int[]{eval, mv.x, mv.y};
            }));
}


            for (Future<int[]> future : futures) {
                if (timeoutOccurred) break;
                try {
                    int[] result = future.get();
                    if (result != null) {
                        int eval = result[0];
                        Point mv = new Point(result[1], result[2]);
                        if (eval > currentBestValue) {
                            currentBestValue = eval;
                            currentBestMove = mv;
                        }
                        alpha = Math.max(alpha, currentBestValue);
                        if (beta <= alpha && poda) {
                            break;
                        }
                    }
                } catch (InterruptedException | ExecutionException e) {
                    e.printStackTrace();
                }
            }

            if (timeoutOccurred) break;

            bestMove = currentBestMove;
            bestValue = currentBestValue;
            actualdepth = currentDepth;
        }

        return new PlayerMove(bestMove, leafCount, actualdepth, SearchType.MINIMAX);
    }

    private int minMax(HexGameStatus board, int depth, boolean isMaximizing, int alpha, int beta, Point move) {
        if (timeoutOccurred) return 0;
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
            if (alpha >= beta && poda) {
                return ttEntry.value;
            }
        }

        if (board.isGameOver()) {
            if (board.GetWinner() == playertype) {
                return 100000 * (depth);
            } else {
                return -100000 * (depth);
            }
        }
        if (depth == 0 || board.getMoves().isEmpty()) {
            int val = evaluateBoard(board);
            transpositionTable.put(key, new TTEntry(val, depth, TTEntry.Flag.EXACT, alpha, beta));
            return val;
        }

        int originalAlpha = alpha;
        int bestValue = isMaximizing ? Integer.MIN_VALUE : Integer.MAX_VALUE;

        List<Point> moves = generateOrderedMoves(board);

        if (isMaximizing) {
            for (Point mv : moves) {
                HexGameStatus newBoard = new HexGameStatus(board);
                newBoard.placeStone(mv);
                int eval = minMax(newBoard, depth - 1, false, alpha, beta, move);
                bestValue = Math.max(bestValue, eval);
                alpha = Math.max(alpha, bestValue);
                if (beta <= alpha && poda) {
                    break;
                }
            }
        } else {
            for (Point mv : moves) {
                HexGameStatus newBoard = new HexGameStatus(board);
                newBoard.placeStone(mv);
                int eval = minMax(newBoard, depth - 1, true, alpha, beta, move);
                bestValue = Math.min(bestValue, eval);
                beta = Math.min(beta, bestValue);
                if (beta <= alpha && poda) {
                    break;
                }
            }
        }

        TTEntry.Flag flag;
        if (bestValue <= originalAlpha) {
            flag = TTEntry.Flag.UPPER;
        } else if (bestValue >= beta) {
            flag = TTEntry.Flag.LOWER;
        } else {
            flag = TTEntry.Flag.EXACT;
        }

        transpositionTable.put(key, new TTEntry(bestValue, depth, flag, alpha, beta));

        return bestValue;
    }

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

        double myNormalizedMinPathCost = normalizeTo100(myMinPathCost, minPathCostRange, maxPathCostRange);
        double myNormalizedAltPaths = normalizeTo100(myAltPaths, minAltPathsRange, maxAltPathsRange);

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

        double oppNormalizedMinCost = normalizeTo100(oppMinCost, minPathCostRange, maxPathCostRange);
        double oppNormalizedAltPaths = normalizeTo100(oppAltPaths, minAltPathsRange, maxAltPathsRange);

        return (int)((-gamma * myNormalizedMinPathCost + delta * myNormalizedAltPaths)
                 - 6*(-gamma * oppNormalizedMinCost + delta * oppNormalizedAltPaths));
    }

    private double normalizeTo100(int value, int min, int max) {
        if (max - min == 0) return 0;
        return (double)(value - min) / (max - min) * 100;
    }

    private double[] calculateDynamicFactors(int totalMoves, int remainingMoves) {
        double deltaInitial = 8.0;
        double deltaFinal = 1.0;
        double gammaInitial = 1.0;
        double gammaFinal = 3.0;

        double progress = (double) remainingMoves / totalMoves;

        double delta = deltaFinal + (deltaInitial - deltaFinal) * Math.pow(progress, 2);
        double gamma = gammaInitial + (gammaFinal - gammaInitial) * (1 - progress);

        return new double[]{1.0, 3.0};
    }

    private List<Point> generateOrderedMoves(HexGameStatus board) {
        List<Point> moves = new ArrayList<>();
        int size = board.getSize();

        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                if(board.getPos(x,y) == 0) {
                    moves.add(new Point(x,y));
                }
            }
        }

        double cx = size / 2.0;
        double cy = size / 2.0;

        moves.sort(Comparator.comparingDouble((Point p) -> {
            double dx = p.x - cx;
            double dy = p.y - cy;
            return Math.hypot(dx, dy);
        }));
        
        return moves;
    }

    private void initZobrist(int size) {
        Random rand = new Random();
        zobristTable = new long[size][size][3]; 

        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                for(int st=0; st<3; st++) {
                    zobristTable[x][y][st] = rand.nextLong();
                }
            }
        }
    }

    private long computeZobristKey(HexGameStatus board) {
        long h = 0L;
        int size = board.getSize();
        for(int x=0; x<size; x++) {
            for(int y=0; y<size; y++) {
                int c = board.getPos(x,y);
                int st; 
                if(c == 0) {
                    st = 0;
                } else if(c == 1) {
                    st = 1;
                } else {
                    st = 2;
                }
                h ^= zobristTable[x][y][st];
            }
        }
        return h;
    }
}
