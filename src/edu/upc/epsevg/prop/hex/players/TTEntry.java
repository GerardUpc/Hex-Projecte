/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package edu.upc.epsevg.prop.hex.players;

/**
 *
 * @author GERARD
 */

public class TTEntry {
    public enum Flag {
        EXACT,    // Valor exacto
        LOWER,    // Nodo de tipo alfa (corte por debajo)
        UPPER     // Nodo de tipo beta (corte por encima)
    }

    public int value;      // La evaluación guardada
    public int depth;      // Profundidad a la que se calculó
    public Flag flag;      // Tipo de nodo
    public int alpha;      // Alfa en el momento de almacenar
    public int beta;       // Beta en el momento de almacenar

    public TTEntry(int value, int depth, Flag flag, int alpha, int beta) {
        this.value = value;
        this.depth = depth;
        this.flag = flag;
        this.alpha = alpha;
        this.beta = beta;
    }
}

