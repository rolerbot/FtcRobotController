package org.firstinspires.ftc.teamcode;

enum culoare{verde, mov};
public class Coi
{
    private culoare col;
    private double pozitie;

    public Coi()
    {
        this.col = culoare.verde;
        this.pozitie = 0;
    }

    public  Coi(culoare cul, double poz)
    {
        this.col = cul;
        this.pozitie = poz;
    }

    public void setCol(culoare col)
    {
        this.col = col;
    }

    public double getPozitie()
    {
        return pozitie;
    }

    public void setPozitie(double pozitie)
    {
        this.pozitie = pozitie;
    }
}
