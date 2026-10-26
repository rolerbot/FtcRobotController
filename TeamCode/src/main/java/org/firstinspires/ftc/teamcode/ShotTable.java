package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

// Tabel distanta -> viteza shooter + pozitie hood. Intre doua puncte se face media proportionala.
// Adaugi puncte masurate pe robot, de exemplu: Add(150, 1170, 0.57).
public class ShotTable {

    public static class Shot {
        public final double distanceCm;
        public final double velocity;
        public final double hood;

        public Shot(double distanceCm, double velocity, double hood) {
            this.distanceCm = distanceCm;
            this.velocity = velocity;
            this.hood = hood;
        }
    }

    private final List<Shot> shots = new ArrayList<>();

    public void Add(double distanceCm, double velocity, double hood) {
        shots.add(new Shot(distanceCm, velocity, hood));
        Collections.sort(shots, new Comparator<Shot>() {
            @Override
            public int compare(Shot a, Shot b) {
                return Double.compare(a.distanceCm, b.distanceCm);
            }
        });
    }

    public boolean IsEmpty() {
        return shots.isEmpty();
    }

    // Returneaza null daca tabelul e gol
    public Shot Get(double distanceCm) {
        if (shots.isEmpty())
            return null;

        Shot first = shots.get(0);
        Shot last = shots.get(shots.size() - 1);
        if (distanceCm <= first.distanceCm)
            return first;
        if (distanceCm >= last.distanceCm)
            return last;

        for (int i = 0; i < shots.size() - 1; i++) {
            Shot a = shots.get(i);
            Shot b = shots.get(i + 1);
            if (distanceCm >= a.distanceCm && distanceCm <= b.distanceCm) {
                double t = (distanceCm - a.distanceCm) / (b.distanceCm - a.distanceCm);
                return new Shot(distanceCm, Lerp(a.velocity, b.velocity, t), Lerp(a.hood, b.hood, t));
            }
        }
        return last;
    }

    private double Lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
