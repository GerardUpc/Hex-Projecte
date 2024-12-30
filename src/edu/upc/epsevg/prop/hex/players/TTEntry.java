package edu.upc.epsevg.prop.hex.players;

public class TTEntry {
    public enum Flag {
        EXACT,    // Valor exacte
        LOWER,    // Nodo de tipus alfa 
        UPPER     // Nodo de tipus beta 
    }

    public int value;      // La evaluación guardada
    public int depth;      // Profunditat a la que es va calcular
    public Flag flag;      // Tipus de node
    public int alpha;      // Alfa en el moment de emmagatzemar
    public int beta;       // Beta en el moment de emmagatzemar

    public TTEntry(int value, int depth, Flag flag, int alpha, int beta) {
        this.value = value;
        this.depth = depth;
        this.flag = flag;
        this.alpha = alpha;
        this.beta = beta;
    }
}

