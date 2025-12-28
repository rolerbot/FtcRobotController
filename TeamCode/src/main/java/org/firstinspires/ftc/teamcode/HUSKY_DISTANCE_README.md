# HuskyDistance - Ghid de Utilizare

## Descriere
Clasa `HuskyDistance` permite calcularea distanței reale față de un AprilTag detectat de camera Husky Lens.

## Caracteristici
- ✅ Detectează AprilTag-uri folosind Husky Lens
- ✅ Calculează distanța reală în centimetri și inch
- ✅ Afișează poziția tag-ului în cadru (X, Y)
- ✅ Afișează dimensiunile tag-ului în pixeli
- ✅ Mod de calibrare pentru ajustarea precisiei
- ✅ Telemetrie detaliată

## Configurare Inițială

### 1. Hardware
Asigură-te că Husky Lens este configurat în Hardware Map:
```xml
<HuskyLens name="huskylens" port="I2C Bus 0" />
```

### 2. Calibrare (IMPORTANT!)

Constanta `FOCAL_LENGTH` trebuie calibrată pentru camera ta:

**Pași:**
1. Rulează `HuskyDistanceTest` OpMode
2. Plasează un AprilTag la **50 cm distanță** (măsoară cu ruleta!)
3. Apasă butonul **A** pentru a activa modul de calibrare
4. Pe telemetrie vei vedea: `Calculated Focal Length: XXX.XX`
5. Actualizează constanta în `HuskyDistance.java`:
   ```java
   private static final double FOCAL_LENGTH = XXX.XX; // Valoarea calculată
   ```
6. Recompilează și testează din nou

### 3. Mărimea Tag-ului
Dacă folosești tag-uri cu altă mărime decât 16.5cm, actualizează:
```java
private static final double TAG_SIZE_CM = 16.5; // Mărimea ta
```

## Utilizare în Cod

### Exemplu Simplu:
```java
// În OpMode
TelemetryCustom telemetry = new TelemetryCustom(telemetry);
HuskyDistance huskyDistance = new HuskyDistance(telemetry);

@Override
public void runOpMode() {
    huskyDistance.Initialize(hardwareMap);
    huskyDistance.SetTargetTag(1); // Detectează doar tag-ul cu ID=1
    
    waitForStart();
    
    while (opModeIsActive()) {
        huskyDistance.Run(); // Actualizează și afișează pe telemetrie
        
        // Verifică dacă tag-ul este detectat
        if (huskyDistance.IsTagDetected()) {
            double distance = huskyDistance.GetDistance(); // În cm
            
            // Folosește distanța pentru autonomous
            if (distance < 30) {
                // Robotul e prea aproape
            }
        }
        
        telemetry.update();
    }
}
```

### Metode Disponibile:

#### `SetTargetTag(int tagId)`
Setează ID-ul tag-ului pe care vrei să-l urmărești.
- `tagId = 0` → detectează **orice** tag
- `tagId = 1, 2, 3...` → detectează **doar** tag-ul specific

```java
huskyDistance.SetTargetTag(2); // Doar tag-ul cu ID=2
```

#### `GetDistance()`
Returnează distanța în **centimetri**.
```java
double distanceCm = huskyDistance.GetDistance();
// Returnează -1 dacă tag-ul nu e detectat
```

#### `GetDistanceInches()`
Returnează distanța în **inch**.
```java
double distanceInch = huskyDistance.GetDistanceInches();
```

#### `IsTagDetected()`
Verifică dacă tag-ul este vizibil.
```java
if (huskyDistance.IsTagDetected()) {
    // Tag-ul e vizibil
}
```

#### `GetTagX()` / `GetTagY()`
Returnează poziția centrului tag-ului în cadrul imaginii (pixeli).
```java
int x = huskyDistance.GetTagX(); // 0-320 (depinde de rezoluție)
int y = huskyDistance.GetTagY(); // 0-240
```

Folositor pentru **alinierea** robotului cu tag-ul:
```java
int centerX = 160; // Centrul camerei (rezoluție 320x240)
int tagX = huskyDistance.GetTagX();

if (tagX < centerX - 10) {
    // Tag-ul e la stânga, rotește la stânga
} else if (tagX > centerX + 10) {
    // Tag-ul e la dreapta, rotește la dreapta
} else {
    // Tag-ul e centrat
}
```

#### `CalibrateMode(double knownDistanceCm)`
Mod special pentru calibrare. Afișează distanța focală calculată.
```java
// Plasează tag-ul la 50cm
huskyDistance.CalibrateMode(50.0);
// Citește valoarea de pe telemetrie și actualizează FOCAL_LENGTH
```

## Telemetrie Afișată

Când rulezi `huskyDistance.Run()`, vei vedea pe Driver Station:

```
=== HUSKY DISTANCE ===
Target Tag ID: 1
Tag Detected: YES
Tag Position X: 160
Tag Position Y: 120
Tag Width (px): 85
Tag Height (px): 84
Distance (cm): 44.71
Distance (inches): 17.60
```

## Exemple de Utilizare în Autonomous

### 1. Apropiere de Tag până la Distanță Țintă:
```java
// Apropie-te de tag până la 30cm
huskyDistance.SetTargetTag(1);

while (opModeIsActive() && huskyDistance.GetDistance() > 30) {
    huskyDistance.Run();
    
    if (huskyDistance.IsTagDetected()) {
        // Mergi înainte încet
        drive.moveForward(0.3);
    } else {
        // Tag-ul s-a pierdut, oprește
        drive.stop();
        break;
    }
}
drive.stop();
```

### 2. Aliniere Orizontală cu Tag-ul:
```java
int centerX = 160; // Centrul camerei
int tolerance = 10; // Pixeli de toleranță

while (opModeIsActive()) {
    huskyDistance.Run();
    
    if (huskyDistance.IsTagDetected()) {
        int tagX = huskyDistance.GetTagX();
        
        if (Math.abs(tagX - centerX) < tolerance) {
            // Aliniat!
            break;
        } else if (tagX < centerX) {
            // Rotește la stânga
            drive.rotate(-0.2);
        } else {
            // Rotește la dreapta
            drive.rotate(0.2);
        }
    }
}
drive.stop();
```

### 3. Combinație: Apropiere + Aliniere:
```java
// Pas 1: Aliniază-te cu tag-ul
alignWithTag(huskyDistance, drive);

// Pas 2: Apropie-te până la distanța țintă
approachTag(huskyDistance, drive, 25.0); // 25cm țintă
```

## Limitări și Note

### ⚠️ Limitări Husky Lens:
1. **Precizie limitată**: Husky Lens nu este la fel de precis ca o cameră stereo sau Limelight
2. **Lighting**: Funcționează mai bine cu lumină bună și uniformă
3. **Unghi**: Tag-ul trebuie văzut frontal (nu din lateral)
4. **Distanță**: Funcționează optim între 20-150 cm

### 💡 Sfaturi:
- Calibrează `FOCAL_LENGTH` în condiții similare cu cele de competiție (lumină, distanță)
- Testează la mai multe distanțe pentru a verifica acuratețea
- Folosește un **filtru** pentru a evita sărăturile de date:
  ```java
  // Media ultimelor 3 măsurători
  double[] lastDistances = new double[3];
  int index = 0;
  
  double currentDistance = huskyDistance.GetDistance();
  lastDistances[index % 3] = currentDistance;
  index++;
  
  double avgDistance = (lastDistances[0] + lastDistances[1] + lastDistances[2]) / 3.0;
  ```

## Depanare

### "Tag Detected: NO" - Tag-ul nu se detectează
- ✅ Verifică dacă Husky Lens este în modul **Tag Recognition**
- ✅ Asigură-te că tag-ul este un **AprilTag valid**
- ✅ Verifică conexiunea I2C
- ✅ Tag-ul trebuie să fie **bine iluminat**

### Distanța este incorectă
- ✅ **Calibrează `FOCAL_LENGTH`** - acesta e cel mai important!
- ✅ Verifică dacă `TAG_SIZE_CM` este corect
- ✅ Asigură-te că tag-ul este **print-at la scară corectă** (16.5cm x 16.5cm)
- ✅ Tag-ul trebuie văzut **frontal**, nu din unghi

### Distanța variază mult
- ✅ Folosește un filtru (medie pe mai multe frame-uri)
- ✅ Îmbunătățește iluminarea
- ✅ Stabilizează camera (montare rigidă)

## Autor
Creat pentru echipa FTC - 2025 Season

