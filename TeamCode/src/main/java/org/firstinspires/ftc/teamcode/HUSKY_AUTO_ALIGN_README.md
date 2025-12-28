# Auto-Aliniere Husky - Ghid de Utilizare

## Descriere
Clasa `Husky` include acum funcționalitate de **auto-aliniere** cu tag-ul echipei tale (ID 4 pentru Blue sau ID 5 pentru Red).

## Cum funcționează Auto-Aliniearea?

### **Pas 1: Rotație orizontală**
Robotul se rotește până când tag-ul este **în partea dreaptă a camerei** (poziția X > 200 pixeli).

### **Pas 2: Ajustare distanță**
După ce tag-ul este în dreapta, robotul:
- Merge **înainte** dacă distanța > 140 cm
- Merge **înapoi** dacă distanța < 100 cm
- Se **oprește** când distanța este între 100-140 cm (~120 cm țintă)

### **Pas 3: Completare**
Când distanța este optimă, robotul se oprește și procesul este finalizat.

---

## Utilizare

### **1. Configurare în OpMode**

```java
// Inițializare
TelemetryCustom telemetry = new TelemetryCustom(telemetry);
GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
GamepadEx gamepad2Ex = new GamepadEx(gamepad2);
Drivetrain drivetrain = new Drivetrain(gamepad1Ex, gamepad2Ex);
Husky husky = new Husky(telemetry, gamepad1Ex, drivetrain);

@Override
public void runOpMode() {
    drivetrain.Initialize(hardwareMap);
    husky.Initialize(hardwareMap);
    
    // Setează echipa (4 = Blue, 5 = Red)
    husky.SetTeamId(4); // Pentru echipa Blue
    
    waitForStart();
    
    while (opModeIsActive()) {
        drivetrain.Run();  // Control manual normal
        husky.Run();       // Procesare Husky + auto-aliniere
        
        telemetry.update();
    }
}
```

### **2. Control în TeleOp**

#### **Buton: RIGHT BUMPER**
- **Prima apăsare**: Începe auto-aliniearea
- **A doua apăsare**: Oprește auto-aliniearea (în orice moment)

#### **Comportament:**
1. **Tag-ul NU este detectat**: 
   - Robotul se oprește
   - Telemetrie: "NO TAG DETECTED!"
   
2. **Tag-ul este detectat**:
   - **Stare 1 (ROTATING)**: Robotul se rotește la dreapta până când tag-ul e în partea dreaptă a camerei
   - **Stare 2 (ADJUSTING_DISTANCE)**: Robotul se mișcă înainte/înapoi pentru a ajunge la 100-140 cm
   - **Stare 3 (COMPLETED)**: Auto-aliniearea este finalizată, robotul se oprește

---

## Parametri Configurabili

Poți ajusta acești parametri în clasa `Husky`:

```java
private static final double TARGET_DISTANCE = 120.0; // Distanța țintă (cm)
private static final double DISTANCE_TOLERANCE_MIN = 100.0; // Distanța minimă acceptabilă (cm)
private static final double DISTANCE_TOLERANCE_MAX = 140.0; // Distanța maximă acceptabilă (cm)
private static final int CAMERA_CENTER_X = 160; // Centrul camerei (px)
private static final int CAMERA_RIGHT_THRESHOLD = 200; // Pragul pentru "dreapta" (px)
private static final double ROTATION_SPEED = 0.3; // Viteza de rotație (0.0 - 1.0)
private static final double APPROACH_SPEED = 0.4; // Viteza de apropiere (0.0 - 1.0)
```

### **Explicații:**
- `TARGET_DISTANCE`: Distanța ideală la care vrei să te oprești (120 cm = ~47 inch)
- `DISTANCE_TOLERANCE_MIN/MAX`: Fereastra de toleranță (100-140 cm)
- `CAMERA_RIGHT_THRESHOLD`: Poziția X în pixeli pentru "dreapta camerei" (0-320, centrul e 160)
- `ROTATION_SPEED`: Puterea motoarelor la rotație (0.3 = 30% putere)
- `APPROACH_SPEED`: Puterea motoarelor la apropiere (0.4 = 40% putere)

---

## Telemetrie Afișată

Când auto-aliniearea este activă, vei vedea:

```
=== HUSKY DISTANCE ===
Team ID: 4 (Blue)
Tag Detected: YES
Tag Position X: 205
Tag Position Y: 120
Tag Width (px): 85
Tag Height (px): 84
Distance (cm): 125.50

=== AUTO-ALIGN ===
ACTIVE
State: ROTATING
Rotating: RIGHT
```

**SAU** când ajustează distanța:

```
=== AUTO-ALIGN ===
ACTIVE
State: ADJUSTING_DISTANCE
Current Distance: 125.50 cm
Target Distance: 120.00 cm
Moving: BACKWARD
```

---

## Secvența Tipică de Utilizare

### **Scenario 1: Poziționare în Autonomous**
```java
// La începutul autonomous
husky.SetTeamId(4); // Blue team

// Așteaptă să detecteze tag-ul
while (!husky.IsTagDetected() && opModeIsActive()) {
    husky.Run();
    sleep(50);
}

// Activează auto-aliniearea programatic (opțional - sau apasă butonul manual)
// ... cod pentru activare automată poate fi adăugat

// Lasă robotul să se alinieze
// (În mod normal apeși Right Bumper manual în TeleOp)
```

### **Scenario 2: TeleOp Manual**
```java
// Driver-ul conduce robotul aproape de tag
// Driver-ul apasă RIGHT BUMPER
// Robotul se aliniază automat
// Driver-ul poate opri oricând cu RIGHT BUMPER din nou
```

---

## Probleme Comune & Soluții

### **1. Robotul se rotește prea repede**
**Soluție**: Scade `ROTATION_SPEED` (ex: de la 0.3 la 0.2)

### **2. Robotul nu se oprește la distanța corectă**
**Soluții**:
- Verifică calibrarea `FOCAL_LENGTH` în clasa `Husky`
- Ajustează `DISTANCE_TOLERANCE_MIN` și `DISTANCE_TOLERANCE_MAX`
- Verifică că tag-ul are exact 16.5 cm x 16.5 cm

### **3. Robotul nu vede tag-ul**
**Soluții**:
- Verifică că tag-ul are ID 4 (Blue) sau 5 (Red)
- Verifică că `IdTeam` este setat corect cu `husky.SetTeamId(4)` sau `(5)`
- Asigură-te că Husky Lens este în modul **Tag Recognition**
- Verifică iluminarea - tag-ul trebuie să fie bine iluminat

### **4. Tag-ul este detectat dar rotația nu funcționează**
**Soluții**:
- Verifică că `Drivetrain` este inițializat corect
- Verifică că metodele `Rotate()` și `MoveForward()` funcționează
- Testează manual: apelează `drivetrain.Rotate(0.3)` și vezi dacă robotul se rotește

### **5. Auto-aliniearea nu pornește când apeși RIGHT BUMPER**
**Soluții**:
- Verifică că `husky.Run()` este apelat în bucla principală
- Verifică că butonul RIGHT BUMPER nu este folosit de alte funcții
- Verifică log-urile pe telemetrie pentru "Auto-Align: STARTED"

---

## Notă Importantă: Controlul Manual vs Auto

**În timpul auto-alinieării**:
- Joystick-urile **NU** vor funcționa (robotul e controlat automat)
- Poți **opri** auto-aliniearea oricând cu RIGHT BUMPER
- După oprire, controlul manual revine la normal

**Recomandare**: Testează auto-aliniearea într-un spațiu deschis înainte de competiție!

---

## Exemple de Modificări

### **Schimbă distanța țintă la 100 cm:**
```java
private static final double TARGET_DISTANCE = 100.0;
private static final double DISTANCE_TOLERANCE_MIN = 80.0;
private static final double DISTANCE_TOLERANCE_MAX = 120.0;
```

### **Pune tag-ul în stânga camerei în loc de dreapta:**
```java
// În funcția AlignHorizontally():
private static final int CAMERA_LEFT_THRESHOLD = 100; // În loc de RIGHT

// Apoi modifică logica:
if (tagX < CAMERA_LEFT_THRESHOLD) // În loc de >
{
    alignmentState = AlignmentState.ADJUSTING_DISTANCE;
}
else if (tagX > CAMERA_CENTER_X) // Invers
{
    drivetrain.Rotate(-ROTATION_SPEED); // Rotație la stânga (negativ)
}
```

### **Adaugă buton diferit (ex: LEFT BUMPER):**
```java
// În Initialize():
AlignButton = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
```

---

## Autor
Creat pentru echipa FTC - 2025 Season
Clasa Husky cu funcționalitate de auto-aliniere integrată

