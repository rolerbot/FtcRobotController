# Configurare PIDF pentru Shooter

## 📋 Ce s-a schimbat?

Shooter-ul folosește acum **control prin velocity (PIDF)** în loc de **control prin power**.

### Înainte:
```java
PowerShooterMotors(0.6);  // Power control (0-1)
```

### Acum:
```java
SetShooterVelocity(1600);  // Velocity control (RPM)
```

---

## 🎯 Valorile tale găsite prin tuning

Din **ShooterTuning.java** ai găsit:
- **F = 13.8**
- **P = -0.11**

⚠️ **NOTĂ IMPORTANTĂ:** Valoarea **P = -0.11** (negativă) este neobișnuită! 

### De ce P este negativ?
- Probabil ai apăsat prea mult **D-PAD DOWN** în timpul tuning-ului
- În mod normal, P ar trebui să fie **pozitiv** (între 0 și 5)

### Recomandare:
**Re-tunează** folosind ghidul și asigură-te că:
1. Tunezi **F mai întâi** până când Error < 50
2. Apoi adaugi **P pozitiv** (apasă D-PAD UP) până când viteza se stabilizează
3. Dacă motoarele oscilează, **scazi P puțin** (dar rămâne pozitiv!)

---

## 🔧 Unde sunt valorile PIDF în cod?

**Fișier:** `Shooter.java`

```java
// PIDF Configuration - valori din tuning
private final double shooterF = 13.8;
private final double shooterP = -0.11;  // ATENȚIE: Valoare negativă
private final double shooterI = 0.0;
private final double shooterD = 0.0;
```

### Cum să modifici valorile:

1. **Deschide** `Shooter.java`
2. **Găsește** secțiunea "PIDF Configuration"
3. **Schimbă** valorile `shooterF` și `shooterP` cu cele găsite în tuning
4. **Salvează** fișierul
5. **Re-compilează** și testează pe robot

---

## ⚙️ Viteze configurabile

În `Shooter.java` sunt definite două viteze:

```java
// Velocity targets (RPM)
private final double highVelocity = 1600;      // Aruncare normală
private final double prepareVelocity = 900;    // Pregătire (echivalent 0.45 power)
```

### Ce înseamnă fiecare:

| Viteză | Când se folosește | Valoare implicită | Echivalent power |
|--------|-------------------|-------------------|------------------|
| **highVelocity** | Când se aruncă mingea | 1600 RPM | ~0.6 power |
| **prepareVelocity** | Când se pregătește shooter-ul | 900 RPM | ~0.45 power |

### Cum să ajustezi vitezele:

1. **highVelocity** - Viteza la care ai făcut tuning-ul
   - Dacă ai tunat la 1600 RPM → lasă 1600
   - Dacă ai tunat la altă viteză → schimbă valoarea

2. **prepareVelocity** - Viteza de pregătire (mai mică)
   - Ar trebui să fie **mai mică** decât highVelocity
   - Recomandare: 50-60% din highVelocity
   - Exemplu: dacă highVelocity = 1600, atunci prepareVelocity ≈ 900-1000

---

## 🧪 Cum să testezi

### 1. Verifică că motoarele au encodere conectate
- Deschide Driver Station
- Verifică că encoderele citesc valori când rotești manual motoarele

### 2. Testează shooter-ul
- Rulează OpMode-ul principal (TeleOp)
- Apasă butonul de aruncare (A)
- Observă dacă mingile sunt aruncate consistent

### 3. Monitorizează telemetria
Adaugă în `Shooter.java` (opțional):
```java
telemetry.Log("Motor Velocity", MotorAruncare1.getVelocity());
telemetry.Log("Target Velocity", highVelocity);
```

---

## 🔄 Cum să re-tunezi valorile PIDF

Dacă shooter-ul nu funcționează bine (prea slab, prea puternic, inconsistent):

### Pasul 1: Rulează ShooterTuning
1. Selectează **ShooterTuning** în Driver Station
2. Apasă **INIT**, apoi **START**

### Pasul 2: Tunează F
1. Apasă **B** pentru step size 10.0
2. Apasă **D-PAD LEFT** repetat până când Current Velocity ≈ Target Velocity
3. Scade step size la 1.0, apoi 0.1 pentru ajustare fină
4. **F este bun** când Error < 50

### Pasul 3: Tunează P
1. Apasă **B** pentru step size 0.01
2. Apasă **D-PAD UP** de 3-5 ori
3. Observă: dacă oscilează, apasă **D-PAD DOWN**
4. **P este bun** când Error < 30 și nu oscilează

### Pasul 4: Aplică valorile în Shooter.java
```java
private final double shooterF = 12.5;  // Exemplu - folosește-ți valoarea ta
private final double shooterP = 2.8;   // Ar trebui să fie POZITIV
```

---

## 📊 Comparație Power vs Velocity Control

| Aspect | Power Control (vechi) | Velocity Control (nou) |
|--------|----------------------|------------------------|
| **Consistență** | ❌ Variază cu bateria | ✅ Constant, indiferent de baterie |
| **Precizie** | ⚠️ Aproximativă | ✅ Foarte precisă |
| **Distanță aruncare** | ⚠️ Se schimbă când bateria scade | ✅ Constantă |
| **Necesită tuning** | ❌ Nu | ✅ Da (o singură dată) |
| **Complexitate** | ✅ Simplu | ⚠️ Mediu |

---

## ❓ Probleme frecvente

### Problema 1: Motoarele nu se învârt
**Cauză:** Motoarele nu sunt în modul RUN_USING_ENCODER sau encoderele nu sunt conectate  
**Soluție:** 
- Verifică că encoderele sunt conectate corect
- Verifică că `Initialize()` setează `RUN_USING_ENCODER`

### Problema 2: Viteza oscilează (sare sus-jos)
**Cauză:** P prea mare  
**Soluție:** Scade valoarea lui `shooterP`

### Problema 3: Nu atinge viteza dorită
**Cauză:** F prea mic  
**Soluție:** Crește valoarea lui `shooterF`

### Problema 4: Mingile se aruncă prea slab
**Cauză:** `highVelocity` prea mic  
**Soluție:** Crește `highVelocity` (ex: de la 1600 la 2000)

### Problema 5: Mingile se aruncă prea tare
**Cauză:** `highVelocity` prea mare  
**Soluție:** Scade `highVelocity` (ex: de la 1600 la 1200)

---

## 🎓 Înțelegere avansată

### De ce PIDF?
**PIDF** = **P**roportional + **I**ntegral + **D**erivative + **F**eedForward

- **F (FeedForward):** Forța de bază necesară pentru a atinge viteza
- **P (Proportional):** Corecție proporțională cu eroarea
- **I (Integral):** Corecție pentru erori acumulate (nu folosim)
- **D (Derivative):** Corecție pentru schimbări bruște (nu folosim)

### Formulă simplificată:
```
Motor Power = F × (Target Velocity / Max Velocity) + P × Error
```

Unde:
- **Error** = Target Velocity - Current Velocity
- **F** face majoritatea lucrului (95%)
- **P** corectează erorile mici (5%)

---

## ✅ Checklist Final

Înainte de competiție, asigură-te că:

- [ ] Ai tunat valorile PIDF folosind ShooterTuning
- [ ] Valorile F și P sunt salvate în `Shooter.java`
- [ ] P este **pozitiv** (dacă este negativ, re-tunează!)
- [ ] `highVelocity` este setată la viteza la care ai făcut tuning
- [ ] Ai testat shooter-ul și mingile se aruncă consistent
- [ ] Distanța de aruncare este corectă pentru jocul tău
- [ ] Telemetria arată Error < 50 RPM în timpul aruncării

---

**Succes! 🚀**

