package org.firstinspires.ftc.teamcode;

enum culoare{verde, mov};
public class Artefact
{
    private culoare col;
    private double pozitie;

    public Artefact()
    {
        this.col = culoare.verde;
        this.pozitie = 0;
    }

    public Artefact(culoare cul, double poz)
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
