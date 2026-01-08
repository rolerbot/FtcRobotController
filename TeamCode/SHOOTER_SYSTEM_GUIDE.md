# Shooter System - Current Status & Backup Plan

## ✅ Current System (ACTIVE)
**File:** `Shooter.java`
**Control Method:** Voltage-compensated PIDF

### How It Works:
- Uses `ShooterVoltageHelper` to automatically adjust `F` based on battery voltage
- Filters voltage spikes from drivetrain (ALPHA = 0.02)
- Updates every 500ms to prevent motor config spam
- Formula: `power = F * targetVelocity + P * error`

### Configuration:
```java
BASE_SHOOTER_F = 13.3;  // Tuned at 12V
shooterP = 0.01;
shooterI = 0.0;
shooterD = 0.0;
```

### Tuning Instructions:
1. Charge battery to 12.0-12.5V
2. Adjust `BASE_SHOOTER_F` until no overshoot/undershoot at target RPM
3. System will automatically compensate for voltage changes (10.8V - 13.5V)

---

## 🔧 Backup System (IF NEEDED)
**File:** `FlywheelTuning.java` + `PIDFController.java`
**Control Method:** Pedro Pathing kV/kS feedforward

### How It Works:
- Uses `kV` (velocity gain) + `kS` (static friction) for more robust control
- Formula: `power = kV * targetVelocity + kS + P * error`
- Better at handling low speeds and friction

### To Switch to Backup:
1. Remove `@Disabled` from `FlywheelTuning.java`
2. Run the tuning OpMode
3. Use gamepad controls to tune:
   - **Dpad Up/Down:** Adjust target velocity
   - **Y/A:** Increase/Decrease kV
   - **B/X:** Increase/Decrease kS
   - **Right Bumper/Trigger:** Increase/Decrease P

4. Record final tuned values
5. Implement in `Shooter.java` (ask for help with integration)

---

## 📊 Comparison

| Feature | Current System | Backup System |
|---------|----------------|---------------|
| **Ease of Use** | ✅ Simple (1 param) | ⚠️ Complex (3 params) |
| **Voltage Compensation** | ✅ Automatic | ⚠️ Manual tuning needed |
| **Low Speed Control** | ⚠️ Moderate | ✅ Better (kS helps) |
| **Overshoot Prevention** | ✅ Proactive (voltage-based) | ✅ Better steady-state |
| **Tuning Time** | ✅ 5-10 min | ⚠️ 20-30 min |

---

## 🎯 Recommendation

**KEEP CURRENT SYSTEM** unless you experience:
- Persistent overshoot by >50 RPM after voltage compensation
- Inconsistent behavior at different velocities
- Motors struggling at low RPM

**SWITCH TO BACKUP** if:
- Voltage compensation doesn't eliminate overshoot
- Need more precise control across velocity ranges
- Have time to re-tune before competition

---

## 📝 Files Created/Modified

### Current System:
- ✅ `Shooter.java` - Uses voltage compensation
- ✅ `ShooterVoltageHelper.java` - Handles voltage filtering & F calculation

### Backup System (Ready to Use):
- ✅ `FlywheelTuning.java` - Tuning OpMode with gamepad controls
- ✅ `PIDFController.java` - Manual PIDF controller for kV/kS

---

## 🚀 Next Steps

1. **Test current system** with different battery voltages
2. Monitor telemetry for "F Voltage-Comp" messages
3. If overshoot persists, enable `FlywheelTuning` and tune kV/kS
4. Report back results!

