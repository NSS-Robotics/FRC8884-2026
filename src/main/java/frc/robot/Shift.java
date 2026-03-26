package frc.robot;

public enum Shift {
    TRANSITION,
    ALLIANCEA, // our hub active thing
    ALLIANCEB, // their hub active 
    ENDGAME;

    public String toString() {
        if (this == TRANSITION) return "Transition";
        if (this == ALLIANCEA) return "OUR";
        if (this == ALLIANCEB) return "THEIR";
        if (this == ENDGAME) return "Endgame";

        return "No such shift.";
    }
}
