package edu.upc.epsevg.prop.hex;

import edu.upc.epsevg.prop.hex.players.MecMec_FixedDepth;
import edu.upc.epsevg.prop.hex.players.HumanPlayer;
import edu.upc.epsevg.prop.hex.players.RandomPlayer;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.players.H_E_X_Player;
import edu.upc.epsevg.prop.hex.HeadlessGame;
import edu.upc.epsevg.prop.hex.players.MecMec_IterativeDepth;



import javax.swing.SwingUtilities;

/**
 * Checkers: el joc de taula.
 * @author bernat
 */
public class Game {
        /**
     * @param args
     */
    public static void main(String[] args) { 
        
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                
                IPlayer player2 = new H_E_X_Player(2/*GB*/);
                
                
                //IPlayer player1 = new RandomPlayer("dwdw");
                
                IPlayer player1 = new MecMec_IterativeDepth(4);
                
                //IPlayer player2 = new HumanPlayer("Human");
                                
                new Board(player1 , player2, 11,  10, false);
                //HeadlessGame jaja = new HeadlessGame(new PlayerMinimax(3), new H_E_X_Player(2/*GB*/), 11, 30, 1);
      
 
                
             }
        });
    }
}
