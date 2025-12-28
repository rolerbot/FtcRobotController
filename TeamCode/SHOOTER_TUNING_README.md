# Ghid de Tuning pentru Motoarele Shooter-ului

## Pregătire

1. **Pornește robotul** și conectează Driver Station-ul
2. **Selectează OpMode-ul** "ShooterTuning" din lista de TeleOp
3. **Apasă INIT**, apoi **START**
4. **Urmărește telemetria** pe Driver Station

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
   - `Current Velocity` se apropie de `Target Velocity` (6000)
   - `Error` devine sub 300-400

4. **Ajustare fină:**
   - Când Error < 500, apasă **B** pentru step size 1.0
   - Când Error < 100, apasă **B** pentru step size 0.1
   - Continuă să ajustezi F cu **D-PAD LEFT/RIGHT**

5. **F este bun când:**
   - Error este între -100 și +100
   - Viteza este stabilă (nu oscilează)

**Exemplu:**
```
Target Velocity: 6000
Current Velocity: 5750  ← prea mic, crește F
Error: 250              ← când scade sub 100, ești gata cu F
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
   - Error < 50
   - Nu există oscilații
   - Viteza se stabilizează rapid (sub 1 secundă)

**Exemplu:**
```
Error: 45     ← excelent!
F: 12.340
P: 2.500      ← valoare bună
```

---

### 🔄 PAS 3: Testează la viteza joasă

1. **Apasă A** pentru a schimba la viteza joasă (1500)
2. **Verifică** dacă Error rămâne mic (sub 50-100)
3. **Dacă Error > 100:**
   - Ajustează ușor F (step size 0.1)
   - Ajustează ușor P dacă e necesar

4. **Apasă A** din nou pentru a reveni la viteza mare (6000)
5. **Verifică** că totul e încă bun

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
| **A** | Schimbă viteza (6000 ↔ 1500) | Pentru a testa la ambele viteze |
| **B** | Schimbă step size (10 → 1 → 0.1 → 0.01 → 0.001) | La început folosește 10, apoi scade treptat |
| **D-PAD LEFT** | Crește F | Când viteza e prea mică |
| **D-PAD RIGHT** | Scade F | Când viteza e prea mare |
| **D-PAD UP** | Crește P | Pentru a reduce eroarea și stabiliza |
| **D-PAD DOWN** | Scade P | Când motoarele oscilează |

---

## TELEMETRIE - CE ÎNSEAMNĂ

```
Target Velocity: 6000        ← Viteza dorită
Current Velocity: 5950       ← Viteza actuală a motorului
Error: 50                    ← Diferența (cât mai mică = mai bine)
-------------------------
F(D-left): 12.340           ← Valoarea F curentă
P(D-UP): 2.500              ← Valoarea P curentă
Step Size: 0.1              ← Cât se schimbă la fiecare apăsare
```

---

## INDICATORI DE PERFORMANȚĂ

### ✅ EXCELENT
- Error: < 50
- Fără oscilații
- Stabilizare rapidă (< 0.5s)

### ✔️ BUN
- Error: 50-100
- Oscilații minime
- Stabilizare acceptabilă (< 1s)

### ❌ SLAB
- Error: > 200
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

### Problema: Error rămâne constant la ~100-200
**Cauză:** F nu e perfect ajustat  
**Soluție:** Ajustează fin F cu step size 0.1

---

## VALORI ESTIMATE (pentru motoare REV/GoBuilda)

Acestea sunt doar estimări - valorile tale vor varia!

- **F:** între 11-15 (depinde de baterie și motoare)
- **P:** între 1-5 (start cu 2-3)

---

## SFATURI IMPORTANTE

### ✅ DO (Fă):
- ✅ Începe ÎNTOTDEAUNA cu F, apoi ajustează P
- ✅ Folosește step size mare (10) la început
- ✅ Scade step size pe măsură ce te apropii de valoarea optimă
- ✅ Așteaptă 2-3 secunde după fiecare ajustare
- ✅ Testează la AMBELE viteze (high și low)
- ✅ Notează valorile când găsești setări bune

### ❌ DON'T (Nu face):
- ❌ Nu modifica P înainte de F
- ❌ Nu face schimbări mari când ești aproape de optim
- ❌ Nu te grăbi - tuning-ul durează 5-10 minute
- ❌ Nu ignora oscilațiile - înseamnă P prea mare
- ❌ Nu uita să testezi la viteza joasă

---

## SECVENȚA COMPLETĂ (Rezumat)

1. **START** → Rulează OpMode-ul
2. **B** → Setează step size 10.0
3. **D-PAD LEFT** (repetat) → Crește F până Error < 300
4. **B** → Setează step size 1.0
5. **D-PAD LEFT/RIGHT** → Ajustează fin F până Error < 100
6. **B** → Setează step size 0.01
7. **D-PAD UP** (3-5 ori) → Crește P pentru stabilizare
8. **Observă** → Dacă oscilează, scade P
9. **A** → Schimbă la viteza joasă
10. **Verifică** → Error ar trebui să rămână mic
11. **A** → Înapoi la viteza mare
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

---

**Succes la tuning! 🚀**

