# 🎛️ Virtual Target Lead Compensation - Calibration Guide

## Quick Reference: What to Change

### If Robot OVERSHOOTS (aims too far ahead):
```java
// Line 355: Reduce the lead time multiplier
double leadTime = 0.15 * leadFactor;  // Changed from 0.20
```

### If Robot UNDERSHOOTS (aims behind target):
```java
// Line 355: Increase the lead time multiplier
double leadTime = 0.25 * leadFactor;  // Changed from 0.20
```

---

## 📍 Exact Location in Code

Open `RobotAlignment.java` and find **line 355** (in the `CalculateHeadingCorrection()` method):

```java
// ✅ NEW STEP 3.5: CREATE VIRTUAL TARGET based on velocity
double virtualTargetX = targetX;
double virtualTargetY = targetY;

if (velocityMetersPerSec > 0.15) {  // Line 351
    double leadFactor = Math.min(velocityMetersPerSec / 1.5, 1.0);
    double leadTime = 0.20 * leadFactor;  // ← LINE 355: TUNE THIS!
    
    virtualTargetX = targetX - (velocityX * leadTime);
    virtualTargetY = targetY - (velocityY * leadTime);
```

---

## 🔧 Step-by-Step Tuning Process

### Step 1: Test Current Behavior

1. **Enable absolute heading lock** (ct1 Left Bumper)
2. **Drive at medium speed** (about 50-70% throttle)
3. **Strafe sideways** while watching the robot's aim
4. **Observe** where it's pointing:
   - ✅ **Perfect**: Stays aimed at target while moving
   - ❌ **Overshooting**: Aims too far ahead (leads too much)
   - ❌ **Undershooting**: Aims behind target (lags behind)

### Step 2: Identify the Problem

#### 🔴 OVERSHOOTING Symptoms:
- Robot aims **past** the target when strafing
- When moving right, aims too far left
- When moving left, aims too far right
- Feels like it's "over-compensating"

#### 🔵 UNDERSHOOTING Symptoms:
- Robot aims **behind** the target when strafing
- When moving right, still pointing right of target
- When moving left, still pointing left of target
- Feels like it's "lagging" or "following your motion"

### Step 3: Make Adjustments

#### For OVERSHOOTING:
```java
// Start with small reductions
double leadTime = 0.18 * leadFactor;  // Reduce by 10%

// If still overshooting:
double leadTime = 0.15 * leadFactor;  // Reduce by 25%

// If drastically overshooting:
double leadTime = 0.12 * leadFactor;  // Reduce by 40%
```

#### For UNDERSHOOTING:
```java
// Start with small increases
double leadTime = 0.22 * leadFactor;  // Increase by 10%

// If still undershooting:
double leadTime = 0.25 * leadFactor;  // Increase by 25%

// If drastically undershooting:
double leadTime = 0.30 * leadFactor;  // Increase by 50%
```

### Step 4: Test at Different Speeds

After each change, test at:
- **Low speed** (30% throttle)
- **Medium speed** (60% throttle)
- **High speed** (100% throttle)

The system should work well at **all speeds** because of the `leadFactor` scaling.

---

## 🎚️ Advanced Tuning Parameters

### Parameter 1: Minimum Activation Speed
**Location:** Line 351
```java
if (velocityMetersPerSec > 0.15) {  // ← Tune this
```

**What it does:** Only applies lead compensation above this speed

| Value | Effect |
|-------|--------|
| `0.10` | Activates earlier (more compensation at low speeds) |
| `0.15` | **Default** - good balance |
| `0.20` | Activates later (less compensation at low speeds) |

**When to change:**
- **Lower it (0.10)** if you get lag even at medium speeds
- **Raise it (0.20)** if you get jitter at low speeds

---

### Parameter 2: Lead Factor Scaling
**Location:** Line 354
```java
double leadFactor = Math.min(velocityMetersPerSec / 1.5, 1.0);  // ← Tune 1.5
```

**What it does:** Controls how quickly lead compensation ramps up with speed

| Value | Effect |
|-------|--------|
| `1.0` | Very aggressive - full compensation at 1.0 m/s |
| `1.5` | **Default** - full compensation at 1.5 m/s |
| `2.0` | Conservative - full compensation at 2.0 m/s |

**When to change:**
- **Lower it (1.0-1.2)** if you need more compensation at medium speeds
- **Raise it (1.8-2.0)** if compensation is too strong at medium speeds

---

### Parameter 3: Maximum Virtual Target Shift
**Location:** Line 362
```java
double maxShift = 24.0;  // ← Tune this (inches)
```

**What it does:** Safety limit on how far virtual target can move

| Value | Effect |
|-------|--------|
| `18.0"` | More conservative, prevents wild shifts |
| `24.0"` | **Default** - good for FTC speeds |
| `30.0"` | Allows larger shifts for very high speeds |

**When to change:**
- **Lower it (18")** if you see wild aim swings
- **Raise it (30")** if compensation gets "clipped" at max speed

---

## 🧪 Diagnostic Testing Procedure

### Test 1: Stationary Aim (Baseline)
1. Stand still
2. Enable heading lock
3. Robot should aim at target and hold steady
4. ✅ **If this doesn't work**, tune the main PID controller first (not virtual target)

### Test 2: Slow Strafe (Low Speed Compensation)
1. Strafe slowly (30% throttle) to the RIGHT
2. Watch where robot aims
3. Should aim **slightly left** of target (small lead)
4. When you stop, should settle back to center

**Problem?**
- Aims dead center → Lead time too low or activation threshold too high
- Aims far left → Lead time too high

### Test 3: Fast Strafe (High Speed Compensation)
1. Strafe fast (80-100% throttle) to the RIGHT
2. Watch where robot aims
3. Should aim **noticeably left** of target (large lead)
4. When you stop, should settle back to center

**Problem?**
- Still lags right → Need to increase `leadTime`
- Overshoots left → Need to decrease `leadTime`

### Test 4: Direction Change (Transition Smoothness)
1. Strafe right at high speed
2. Quickly switch to strafing left
3. Watch the aim transition
4. Should smoothly transition from aiming left to aiming right

**Problem?**
- Jerky transition → May need to adjust slew rate limiting (different parameter)
- Slow to respond → Check velocity filtering (different parameter)

---

## 📊 Example Tuning Session

### Initial Problem: Robot overshoots when strafing fast

**Test Results:**
- ✅ Stationary: Perfect aim, holds steady
- ⚠️ Slow strafe: Slight overshoot (not terrible)
- ❌ Fast strafe: Major overshoot, aims way past target

**Diagnosis:** Lead time too high

**Action 1:** Reduce from `0.20` to `0.16`
```java
double leadTime = 0.16 * leadFactor;
```

**Test Results After Change:**
- ✅ Stationary: Still perfect
- ✅ Slow strafe: Much better, minimal overshoot
- ⚠️ Fast strafe: Still slight overshoot

**Action 2:** Reduce from `0.16` to `0.14`
```java
double leadTime = 0.14 * leadFactor;
```

**Test Results After Change:**
- ✅ Stationary: Perfect
- ✅ Slow strafe: Perfect
- ✅ Fast strafe: Nearly perfect, very slight lag
- ✅ **ACCEPTED** - Good enough!

---

## 🎯 Optimal Values by Robot Characteristics

### For Heavy/Slow Robots:
```java
double leadTime = 0.15 * leadFactor;  // Less lead needed
if (velocityMetersPerSec > 0.12) {    // Activate earlier
```

### For Light/Fast Robots:
```java
double leadTime = 0.25 * leadFactor;  // More lead needed
if (velocityMetersPerSec > 0.18) {    // Activate later
```

### For Mecanum with Good Traction:
```java
double leadTime = 0.18 * leadFactor;  // Moderate lead
if (velocityMetersPerSec > 0.15) {    // Default threshold
```

### For Mecanum with Slippery Wheels:
```java
double leadTime = 0.22 * leadFactor;  // More lead for drift
if (velocityMetersPerSec > 0.12) {    // Activate earlier
double maxShift = 30.0;                // Allow larger shifts
```

---

## 🔍 Quick Troubleshooting

### Problem: Works at one speed but not others

**Cause:** Lead factor scaling is wrong

**Fix:** Adjust the divisor in line 354:
```java
// If high speed is good but low speed overshoots:
double leadFactor = Math.min(velocityMetersPerSec / 1.8, 1.0);

// If low speed is good but high speed undershoots:
double leadFactor = Math.min(velocityMetersPerSec / 1.2, 1.0);
```

### Problem: Jittery at low speeds

**Cause:** Virtual target activating too early

**Fix:** Raise activation threshold (line 351):
```java
if (velocityMetersPerSec > 0.20) {  // Raised from 0.15
```

### Problem: Wild swings in aim direction

**Cause:** Max shift too high or velocity readings noisy

**Fix 1:** Reduce max shift (line 362):
```java
double maxShift = 18.0;  // Reduced from 24.0
```

**Fix 2:** Or increase velocity buffer size for smoother filtering (line 106):
```java
private static final int VELOCITY_BUFFER_SIZE = 10;  // Increased from 7
```

---

## 📝 Recommended Tuning Order

1. **First:** Tune `leadTime` (line 355) - This is the main parameter
2. **Second:** Tune activation threshold (line 351) if needed
3. **Third:** Tune lead factor scaling (line 354) if behavior varies with speed
4. **Last:** Adjust max shift (line 362) only if you see clipping

---

## 💡 Pro Tips

1. **Make small changes** - Adjust by 0.02-0.05 at a time
2. **Test consistently** - Use same speeds and paths each time
3. **Log data** - Use `LogVelocityDiagnostics()` to see actual velocities
4. **Record values** - Write down what works for your robot
5. **Field conditions matter** - Retune if moving from carpet to tiles

---

## 🎬 Final Validation Test

Once tuned, perform this sequence:

1. ✅ Stand still → Perfect aim
2. ✅ Strafe right slowly → Slight left lead, smooth
3. ✅ Strafe right fast → Noticeable left lead, smooth
4. ✅ Stop → Returns to center aim
5. ✅ Strafe left fast → Noticeable right lead, smooth
6. ✅ Figure-8 pattern → Smooth aim transitions throughout

If all pass, you're calibrated! 🎯

---

## Quick Copy-Paste Values to Try

### Conservative (Less Lead):
```java
double leadTime = 0.12 * leadFactor;
```

### Default (Balanced):
```java
double leadTime = 0.20 * leadFactor;
```

### Aggressive (More Lead):
```java
double leadTime = 0.28 * leadFactor;
```

Start with default, then adjust based on testing!
