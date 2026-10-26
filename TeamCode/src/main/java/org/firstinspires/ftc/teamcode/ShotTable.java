package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

// Tabel distanta -> viteza shooter. Intre doua puncte se face media proportionala.
// Adaugi puncte masurate pe robot, de exemplu: Add(150, 1170).
public class ShotTable {

    public static class Shot {
        public final double distanceCm;
        public final double velocity;

        public Shot(double distanceCm, double velocity) {
            this.distanceCm = distanceCm;
            this.velocity = velocity;
        }
    }

    private final List<Shot> shots = new ArrayList<>();

    public void Add(double distanceCm, double velocity) {
        shots.add(new Shot(distanceCm, velocity));
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
                return new Shot(distanceCm, a.velocity + (b.velocity - a.velocity) * t);
            }
        }
        return last;
    }
}
