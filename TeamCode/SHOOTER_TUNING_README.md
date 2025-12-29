# Ghid de Tuning pentru Motoarele Shooter-ului

## Pregătire

1. **Pornește robotul** și conectează Driver Station-ul
2. **Selectează OpMode-ul** "ShooterTuning" din lista de TeleOp
3. **Apasă INIT**, apoi **START**
4. **Urmărește telemetria** pe Driver Station

---

## ⚙️ SETĂRI INIȚIALE

În `ShooterTuning.java` sunt definite:
- **highVelocity = 3000** (viteza mare, pentru aruncări de putere)
- **lowVelocity = 1500** (viteza redusă, pentru aruncări scurte sau teste)

### De ce două viteze?

**POȚI folosi doar o singură viteză** dacă:
- ✅ Robotul tău folosește întotdeauna aceeași putere de aruncare
- ✅ Nu ai nevoie de aruncări la distanțe diferite
- ✅ Vrei un tuning mai simplu și mai rapid

**Folosește DOUĂ viteze** dacă:
- 🎯 Ai nevoie de aruncări la distanțe diferite (aproape vs. departe)
- 🎯 Vrei să testezi consistența tuning-ului pe un interval mai larg
- 🎯 Vrei să verifici că valorile PIDF sunt stabile la diferite puteri
- 🎯 Ai nevoie de viteze intermediare (ex: 2000, 2500 RPM)

### Ce se întâmplă cu vitezele intermediare?

**Veste bună:** Dacă tunezi la 3000 și 1500, vitezele intermediare (2000, 2200, 2500, etc.) vor funcționa **destul de bine** cu aceleași valori PIDF!

**De ce?** 
- Valorile PIDF sunt *aproximativ liniare* pentru motoarele DC
- Dacă funcționează la capetele intervalului (1500 și 3000), vor funcționa și la mijloc

**Așteptări realiste:**
- ✅ La 3000 RPM: Error < 30 (tunat perfect)
- ✅ La 1500 RPM: Error < 30 (tunat perfect)
- ✔️ La 2250 RPM: Error 30-75 (netestas, dar acceptabil)
- ✔️ La 2000 RPM: Error 30-75 (netestas, dar acceptabil)

**💡 Concluzie:** Dacă folosești 2-3 viteze diferite în joc (ex: tir scurt, mediu, lung), tunează la viteza cea mai mare și cea mai mică, iar cele intermediare vor funcționa automat!

---

## PAȘII DE TUNING

### ⚡ PAS 1: Tunează F (FeedForward) - CEL MAI IMPORTANT!

**Obiectiv:** Să ajungi la 95% din viteza țintă folosind doar F

#### Ce faci:

1. **Setează step size la 10.0**
   - Apasă **B** până când vezi `Step Size: 10.0`

2. **Începe să crești F**
   - Apasă **D-PAD LEFT** repetat
   - Urmărește pe telemetrie cum crește `Current Velocity`

3. **Continuă până când:**
   - `Current Velocity` se apropie de `Target Velocity` (3000)
   - `Error` devine sub 150-200

4. **Ajustare fină:**
   - Când Error < 250, apasă **B** pentru step size 1.0
   - Când Error < 100, apasă **B** pentru step size 0.1
   - Continuă să ajustezi F cu **D-PAD LEFT/RIGHT**

5. **F este bun când:**
   - Error este între -50 și +50
   - Viteza este stabilă (nu oscilează)

**Exemplu:**
```
Target Velocity: 3000
Current Velocity: 2950  ← aproape, doar mai ajustează puțin F
Error: 50               ← când scade sub 50, ești gata cu F
```

---

### 🎯 PAS 2: Tunează P (Proportional)

**Obiectiv:** Elimină eroarea rămasă și stabilizează viteza

#### Ce faci:

1. **Setează step size la 0.01**
   - Apasă **B** până la `Step Size: 0.01`

2. **Crește P treptat**
   - Apasă **D-PAD UP** de 3-5 ori
   - Așteaptă 2-3 secunde după fiecare apăsare

3. **Observă comportamentul:**
   - ✅ Error scade → BINE! Continuă
   - ❌ Viteza oscilează (sare sus-jos) → P prea mare, apasă **D-PAD DOWN**
   - ⚠️ Error rămâne constant → Mai mult P necesar

4. **P este bun când:**
   - Error < 30
   - Nu există oscilații
   - Viteza se stabilizează rapid (sub 1 secundă)

**Exemplu:**
```
Error: 25     ← excelent!
F: 12.340
P: 2.500      ← valoare bună
```

---

### 🔄 PAS 3: (OPȚIONAL) Testează la viteza joasă

**NOTĂ:** Acest pas este opțional. Poți să-l sari dacă folosești doar o singură viteză.

1. **Apasă A** pentru a schimba la viteza joasă (1500)
2. **Verifică** dacă Error rămâne mic (sub 50-75)
3. **Dacă Error > 75:**
   - Ajustează ușor F (step size 0.1)
   - Ajustează ușor P dacă e necesar

4. **Apasă A** din nou pentru a reveni la viteza mare (3000)
5. **Verifică** că totul e încă bun

**💡 Dacă nu folosești viteza joasă:** Ignoră acest pas și continuă direct la PAS 4.

---

### 📝 PAS 4: Notează valorile finale

Când totul arată bine la ambele viteze:

```
┌─────────────────────────────┐
│  VALORI FINALE DE TUNING    │
├─────────────────────────────┤
│  F = ______________         │
│  P = ______________         │
│  Data: ____________         │
└─────────────────────────────┘
```

---

## CONTROALE

| Buton | Funcție | Când să-l folosești |
|-------|---------|---------------------|
| **A** | Schimbă viteza (3000 ↔ 1500) | Pentru a testa la ambele viteze (OPȚIONAL) |
| **B** | Schimbă step size (10 → 1 → 0.1 → 0.01 → 0.001) | La început folosește 10, apoi scade treptat |
| **D-PAD LEFT** | Crește F | Când viteza e prea mică |
| **D-PAD RIGHT** | Scade F | Când viteza e prea mare |
| **D-PAD UP** | Crește P | Pentru a reduce eroarea și stabiliza |
| **D-PAD DOWN** | Scade P | Când motoarele oscilează |

---

## TELEMETRIE - CE ÎNSEAMNĂ

```
Target Velocity: 3000        ← Viteza dorită (3000 sau 1500)
Current Velocity: 2975       ← Viteza actuală a motorului
Error: 25                    ← Diferența (cât mai mică = mai bine)
-------------------------
F(D-left): 12.340           ← Valoarea F curentă
P(D-UP): 2.500              ← Valoarea P curentă
Step Size: 0.1              ← Cât se schimbă la fiecare apăsare
```

---

## INDICATORI DE PERFORMANȚĂ

### ✅ EXCELENT
- Error: < 30
- Fără oscilații
- Stabilizare rapidă (< 0.5s)

### ✔️ BUN
- Error: 30-75
- Oscilații minime
- Stabilizare acceptabilă (< 1s)

### ❌ SLAB
- Error: > 150
- Oscilații vizibile
- Stabilizare lentă (> 1.5s)

---

## TROUBLESHOOTING

### Problema: Nu atinge viteza țintă
**Cauză:** F prea mic  
**Soluție:** Crește F cu D-PAD LEFT

### Problema: Depășește viteza țintă
**Cauză:** F prea mare  
**Soluție:** Scade F cu D-PAD RIGHT

### Problema: Viteza oscilează (sare sus-jos)
**Cauză:** P prea mare  
**Soluție:** Scade P cu D-PAD DOWN

### Problema: Se stabilizează foarte lent
**Cauză:** P prea mic  
**Soluție:** Crește P cu D-PAD UP (dar nu prea mult!)

### Problema: Error rămâne constant la ~50-100
**Cauză:** F nu e perfect ajustat  
**Soluție:** Ajustează fin F cu step size 0.1

---

## VALORI ESTIMATE (pentru motoare REV/GoBuilda)

Acestea sunt doar estimări - valorile tale vor varia!

- **F:** între 11-15 (depinde de baterie și motoare)
- **P:** între 1-5 (start cu 2-3)

**NOTĂ:** Pentru velocități mai mici (3000 vs 6000), este posibil să ai nevoie de valori F ușor diferite.

---

## SFATURI IMPORTANTE

### ✅ DO (Fă):
- ✅ Începe ÎNTOTDEAUNA cu F, apoi ajustează P
- ✅ Folosește step size mare (10) la început
- ✅ Scade step size pe măsură ce te apropii de valoarea optimă
- ✅ Așteaptă 2-3 secunde după fiecare ajustare
- ✅ Testează la AMBELE viteze dacă folosești două (OPȚIONAL)
- ✅ Notează valorile când găsești setări bune
- ✅ Dacă folosești o singură viteză, tunează doar la highVelocity (3000)

### ❌ DON'T (Nu face):
- ❌ Nu modifica P înainte de F
- ❌ Nu face schimbări mari când ești aproape de optim
- ❌ Nu te grăbi - tuning-ul durează 5-10 minute
- ❌ Nu ignora oscilațiile - înseamnă P prea mare
- ❌ Nu complicați lucrurile - dacă o viteză e suficientă, nu folosi două

---

## SECVENȚA COMPLETĂ (Rezumat)

### Varianta SIMPLĂ (o singură viteză - RECOMANDAT pentru începători):

1. **START** → Rulează OpMode-ul
2. **B** → Setează step size 10.0
3. **D-PAD LEFT** (repetat) → Crește F până Error < 150
4. **B** → Setează step size 1.0
5. **D-PAD LEFT/RIGHT** → Ajustează fin F până Error < 50
6. **B** → Setează step size 0.01
7. **D-PAD UP** (3-5 ori) → Crește P pentru stabilizare
8. **Observă** → Dacă oscilează, scade P
9. **NOTEAZĂ** → Valorile F și P finale
10. **GATA!** 🎉

### Varianta AVANSATĂ (două viteze - pentru validare suplimentară):

1. **START** → Rulează OpMode-ul
2. **B** → Setează step size 10.0
3. **D-PAD LEFT** (repetat) → Crește F până Error < 150
4. **B** → Setează step size 1.0
5. **D-PAD LEFT/RIGHT** → Ajustează fin F până Error < 50
6. **B** → Setează step size 0.01
7. **D-PAD UP** (3-5 ori) → Crește P pentru stabilizare
8. **Observă** → Dacă oscilează, scade P
9. **A** → Schimbă la viteza joasă (1500)
10. **Verifică** → Error ar trebui să rămână mic
11. **A** → Înapoi la viteza mare (3000)
12. **NOTEAZĂ** → Valorile F și P finale

---

## TIMP ESTIMAT

⏱️ **5-10 minute** pentru tuning complet

---

## NOTE FINALE

- Valorile PIDF pot varia în funcție de:
  - Voltajul bateriei (baterie plină vs. descărcată)
  - Temperatura motoarelor
  - Uzura mecanică

- **Recomandare:** Re-tunează înainte de fiecare competiție

- **Salvează valorile** pentru fiecare baterie dacă observi diferențe mari

### 🎯 Despre vitezele intermediare:

- Dacă ai tunat la **3000** și **1500**, orice viteză între ele (2000, 2200, 2500, etc.) va funcționa **suficient de bine** cu aceleași valori PIDF
- **Nu este nevoie** să tunezi pentru fiecare viteză în parte
- Eroarea poate fi ușor mai mare (50-100 RPM) la viteze neprobate, dar în general este acceptabil
- Dacă observi probleme la o anumită viteză intermediară, poți face un tuning fin specific pentru acea viteză

### 📊 Exemplu practic:

```
Tunat la 3000 RPM: F=12.5, P=2.8 → Error < 30 ✅
Tunat la 1500 RPM: F=12.5, P=2.8 → Error < 30 ✅

Folosind aceleași valori:
La 2500 RPM: Error ≈ 40-60 ✔️ (acceptabil, fără tuning)
La 2000 RPM: Error ≈ 40-70 ✔️ (acceptabil, fără tuning)
La 2250 RPM: Error ≈ 35-65 ✔️ (acceptabil, fără tuning)
```

**Concluzie:** Testarea la două viteze extreme garantează performanță bună pe tot intervalul!

---

**Succes la tuning! 🚀**

