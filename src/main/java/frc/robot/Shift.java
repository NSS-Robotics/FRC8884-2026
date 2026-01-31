package frc.robot;

public enum Shift {
    TRANSITION,
    ALLIANCEA,
    ALLIANCEB,
    ENDGAME;

    public String toString() {
        if (this == TRANSITION) return "Transition";
        if (this == ALLIANCEA) return "Alliance A";
        if (this == ALLIANCEB) return "Alliance B";
        if (this == ENDGAME) return "Endgame";

        return "No such shift.";
    }
}
