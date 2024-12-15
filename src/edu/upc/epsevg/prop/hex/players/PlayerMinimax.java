/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package edu.upc.epsevg.prop.hex.players;

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

/**
 *
 * @author GERARD i ALEX
 */
public class PlayerMinimax implements IPlayer, IAuto {
    
    public String name;
    public int depth;
    public int player;
    public int leafCount = 0;
    public Boolean poda = true;
    
    
    /**
     * Constructor per defecte.
     *
     * @param depth profunditat màxima, ha de ser major o igual a 1.
     * 
     */
    public PlayerMinimax(int depth) {
        this.name = "PlayerMinimax";
        this.depth = depth;
    }
  
    @Override
    public String getName() {
        return name;
    }
    
    /**
     * Ens avisa que hem de parar la cerca en curs perquè s'ha exhaurit el temps
     * de joc.
     */
    @Override
    public void timeout() {
        // Nothing to do! I'm so fast, I never timeout 8-) MICKEY
    }

    /**
     * Decideix el moviment del jugador donat un tauler i un color de peça que
     * ha de posar.
     *
     * @param s Tauler i estat actual de joc.
     * @return el moviment que fa el jugador.
     */
    @Override
    public PlayerMove move(HexGameStatus s) {
        
        player = s.getCurrentPlayerColor();
        leafCount = 0;
        
        int bestValue = Integer.MIN_VALUE;
        
        int alfa = Integer.MIN_VALUE;
        
        Point bestMove = new Point(-1, -1);
        
        int size = s.getSize();
        
        // ** NODES DE PROFUNDITAT 1 DE MINIMAX (121 TAULERS) ** //
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                
                if (s.getPos(x, y) != 0) continue;
                    
                HexGameStatus newBoard = new HexGameStatus(s);
                newBoard.placeStone(new Point(x, y));
                
                int value = minMax(newBoard, depth - 1, false, alfa, Integer.MAX_VALUE);
                
                if(value > bestValue) {
                    bestValue = value;
                    bestMove.x = x;
                    bestMove.y = y;
                    
                }
                
                alfa = Math.max(alfa, bestValue);
                
            }
        }
        
        System.out.println("Nodes finals: " + leafCount);
        
        return new PlayerMove(bestMove, 0, 0, SearchType.MINIMAX);
        
    }
   
    private int minMax(HexGameStatus board, int depth, boolean isMaximizing, int alfa, int beta) {

        if (depth == 0 || (board.getMoves().isEmpty())) return evaluateBoard(board);
        
        // **ALGORITME EN NIVELL MAX** //
        if (isMaximizing) { 
            
            int maxEval = Integer.MIN_VALUE;
            
            int size = board.getSize(); 
           
            // TODO: Recorrer primer caselles més optimes per a millorar la poda
            for (int x = 0; x < size; x++) {
                for (int y = 0; y < size; y++) {
                    
                    if (board.getPos(x, y) != 0) continue;
                    
                    HexGameStatus newBoard = new HexGameStatus(board);
                    newBoard.placeStone(new Point(x, y));
            
                    int eval = minMax(newBoard, depth - 1, true, alfa, beta);
                    
                    maxEval = Math.max(maxEval, eval);
                    alfa = Math.max(maxEval, alfa);

                    if (beta <= alfa && poda) break;
                    
                }
            }
            
            return maxEval;
        
        // **ALGORITME EN NIVELL MIN** //
        } else {
            
            int minEval = Integer.MAX_VALUE;
            
            int size = board.getSize(); 
            
            // TODO: Recorrer primer caselles més optimes per a millorar la poda
            for (int x = 0; x < size; x++) {
                for (int y = 0; y < size; y++) {
                
                    if (board.getPos(x, y) != 0) continue;
                    
                    HexGameStatus newBoard = new HexGameStatus(board);
                    newBoard.placeStone(new Point(x, y));
                    
                    int eval = minMax(newBoard, depth - 1, true, alfa, beta);
                    
                    minEval = Math.min(minEval, eval);
                    beta = Math.min(minEval, beta);

                    if (beta <= alfa && poda) break;
                    
                }
            }
            
            return minEval;
            
        }
    }


    int evaluateBoard(HexGameStatus board) {
        leafCount++;
        
        return 0;
    
    }
    
}