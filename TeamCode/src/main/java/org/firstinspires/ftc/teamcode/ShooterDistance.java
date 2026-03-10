package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ShooterDistance {

    public static class ShotParameters {
        public double distanceCm;
        public double rpm;
        public double hoodAngle;

        public ShotParameters(double distanceCm, double rpm, double hoodAngle) {
            this.distanceCm = distanceCm;
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
        }
    }

    private final List<ShotParameters> table = new ArrayList<>();

    public ShooterDistance() {
        LoadDefaultTable();
    }

    private void LoadDefaultTable() {
        table.clear();
        table.add(new ShotParameters(120, 1110, 0.557));
        table.add(new ShotParameters(135, 1140, 0.555));
        table.add(new ShotParameters(150, 1170, 0.571));
        table.add(new ShotParameters(165, 1205, 0.583));
        table.add(new ShotParameters(180, 1200, 0.584));
        table.add(new ShotParameters(195, 1220, 0.581));
        table.add(new ShotParameters(210, 1230, 0.578));
        table.add(new ShotParameters(225, 1240, 0.575));
        table.add(new ShotParameters(240, 1270, 0.577));
        table.add(new ShotParameters(255, 1360, 0.589));
        table.add(new ShotParameters(270, 1390, 0.593));
        table.add(new ShotParameters(285, 1410, 0.594));
        table.add(new ShotParameters(300, 1455, 0.591));
        table.add(new ShotParameters(315, 1480, 0.59));
        table.add(new ShotParameters(330, 1490, 0.59));
        table.add(new ShotParameters(345, 1500, 0.588));
        table.add(new ShotParameters(360, 1520, 0.59));
        table.add(new ShotParameters(375, 1570, 0.59));
        SortTable();
    }

    public ShotParameters GetShotParams(double distanceCm) {
        if (table.isEmpty())
            return new ShotParameters(distanceCm, 1300, 0.51);

        if (distanceCm <= table.get(0).distanceCm)
            return table.get(0);
        if (distanceCm >= table.get(table.size() - 1).distanceCm)
            return table.get(table.size() - 1);

        for (int i = 0; i < table.size() - 1; i++) {
            ShotParameters a = table.get(i);
            ShotParameters b = table.get(i + 1);
            if (distanceCm >= a.distanceCm && distanceCm <= b.distanceCm) {
                double t = (distanceCm - a.distanceCm) / (b.distanceCm - a.distanceCm);
                return new ShotParameters(
                        distanceCm,
                        Lerp(a.rpm, b.rpm, t),
                        Lerp(a.hoodAngle, b.hoodAngle, t));
            }
        }

        return table.get(table.size() - 1);
    }

    private double Lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private void SortTable() {
        Collections.sort(table, new Comparator<ShotParameters>() {
            @Override
            public int compare(ShotParameters a, ShotParameters b) {
                return Double.compare(a.distanceCm, b.distanceCm);
            }
        });
    }
}