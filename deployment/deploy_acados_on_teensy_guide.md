# Deploying Your acados NMPC on the Teensy 4.1 — Step by Step

## The Big Picture

Here's what's happening at a high level. Right now your workflow is:

```
MATLAB  →  CasADi builds symbolic math  →  acados wraps it in an NLP solver  →  MATLAB calls the solver
```

What you need is:

```
MATLAB  →  CasADi builds symbolic math  →  acados generates pure C code  →  you compile that C on the Teensy
```

The good news: acados already does almost all of this for you. When you ran `AcadosOcpSolver(ocp)` in MATLAB, it **already generated C code** in a folder called `c_generated_code/`. You just didn't use it — MATLAB was calling it behind the scenes via MEX files.

Your job is to take that generated C code, add the acados runtime library (also C), cross-compile everything for the Teensy's ARM processor, and write a thin Arduino sketch that calls the solver.

---

## Step 0: Install PlatformIO (replaces Arduino IDE)

The Arduino IDE is painful for projects with many C files. Use **PlatformIO** instead — it handles compiler flags, include paths, and library linking properly.

```bash
# Install PlatformIO CLI (requires Python)
pip install platformio

# Or install the VS Code extension: search "PlatformIO IDE" in Extensions
```

If you strongly prefer the Arduino IDE, everything below still applies, but managing include paths and source files will be manual and error-prone. PlatformIO is strongly recommended.

---

## Step 1: Generate the C Code from MATLAB

### 1a. Modify your MATLAB script

You need to make two small changes to your `acadosNMPC_acadosSimulation.m`. After the OCP is built, add these lines to force code generation and make the solver lighter for embedded use:

```matlab
%% --- OCP SETUP --- (your existing code, with embedded-friendly changes)
N      = 10;
Tf     = 0.1;
dt_mpc = Tf / N;

ocp = AcadosOcp();
ocp.model.name        = 'maglev_nmpc';
ocp.model.x           = x;
ocp.model.u           = u;
ocp.model.xdot        = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 2;    % <-- REDUCED from 4 (saves ~50% compute)
ocp.solver_options.sim_method_num_steps  = 3;    % <-- REDUCED from 10 (saves ~70% compute)
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';

% ... (cost, constraints, etc. stay the same) ...

%% --- BUILD THE SOLVER (this generates C code) ---
ocp_solver = AcadosOcpSolver(ocp);
```

> **Important:** Before reducing `num_stages` and `num_steps`, verify in your MATLAB simulation that the solver still works well with the reduced settings. Run your simulation loop and check that the behavior is similar. If it degrades badly, try `num_stages=3, num_steps=5` as a middle ground.

### 1b. Run the script

```matlab
>> acadosNMPC_acadosSimulation   % (or whatever you named your modified version)
```

This creates the folder:

```
<your_working_directory>/c_generated_code/
```

### 1c. Verify the generated code exists

```bash
ls c_generated_code/
```

You should see something like:

```
c_generated_code/
├── acados_solver_maglev_nmpc.h        ← THE main header you'll #include
├── acados_solver_maglev_nmpc.c        ← solver wrapper
├── acados_sim_solver_maglev_sim.h     ← (sim solver — not needed on Teensy)
├── acados_sim_solver_maglev_sim.c
├── maglev_nmpc_model/                 ← CasADi-generated model functions
│   ├── maglev_nmpc_impl_dae_fun.c
│   ├── maglev_nmpc_impl_dae_fun_jac_x_xdot_u.c
│   └── ... (several more .c files)
├── maglev_nmpc_cost/                  ← cost function files
├── maglev_nmpc_constraints/           ← constraint function files
└── main_maglev_nmpc.c                 ← example standalone main() — very useful!
```

The file `main_maglev_nmpc.c` is a complete working example of how to call the solver from C. **Read this file** — it shows you exactly which functions to call.

---

## Step 2: Understand the acados Runtime Library

The generated code alone isn't enough. It calls functions from the **acados core library** (the QP solver HPIPM, the integrator, etc.). You need those too.

These live in your acados installation at:

```
/home/mariujf/acados/
├── include/                ← header files
│   ├── acados/
│   ├── acados_c/
│   ├── blasfeo/
│   └── hpipm/
└── lib/                    ← precompiled libraries (.a files, but for your PC's x86 arch)
    ├── libacados.a
    ├── libblasfeo.a
    └── libhpipm.a
```

**Problem:** those `.a` libraries are compiled for your Linux PC (x86_64). The Teensy has an ARM Cortex-M7 processor. You need to recompile them. This is the hardest step.

---

## Step 3: Cross-Compile acados for the Teensy 4.1

### Option A: Use the Teensy/ARM toolchain directly (recommended)

PlatformIO downloads the ARM compiler automatically. You can use it to compile acados from source.

#### 3a. Create the PlatformIO project first

```bash
mkdir ~/maglev_teensy_nmpc
cd ~/maglev_teensy_nmpc
platformio init --board teensy41
```

This creates:

```
maglev_teensy_nmpc/
├── platformio.ini
├── src/           ← your .ino/.cpp files go here
├── lib/           ← libraries go here
├── include/       ← header files go here
└── test/
```

#### 3b. Configure platformio.ini

Edit `platformio.ini`:

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

; Teensy 4.1 runs at 600 MHz by default
board_build.f_cpu = 600000000L

; Compiler flags for acados
build_flags =
    -std=gnu11          ; for C files
    -O2                 ; optimization (important for solver speed)
    -DACADOS_WITH_HPIPM
    -DBLASFEO_TARGET_ARMV7A_ARM_CORTEX_A7
    -I include/acados
    -I include/acados_c
    -I include/blasfeo/include
    -I include/hpipm/include

build_unflags =
    -std=gnu++14        ; avoid conflicts

; May need extra memory
board_build.ldscript = teensy41.ld
```

> **Note on the BLASFEO target:** The Teensy 4.1's Cortex-M7 is ARMv7E-M. BLASFEO's closest generic target is `ARMV7A_ARM_CORTEX_A7` or you may need `GENERIC`. If compilation fails with architecture errors, try `-DBLASFEO_TARGET_GENERIC` instead — it's slower but guaranteed to compile.

#### 3c. Copy acados source code into the project

Instead of pre-compiling `.a` files, the easiest approach for a Teensy project is to **compile the acados source files directly** as part of your project. This avoids the entire cross-compilation headache.

```bash
cd ~/maglev_teensy_nmpc

# Create directory structure
mkdir -p lib/acados_core/src
mkdir -p include/acados
mkdir -p include/acados_c
mkdir -p include/blasfeo/include
mkdir -p include/hpipm/include

# Copy the acados headers
cp -r /home/mariujf/acados/include/acados/* include/acados/
cp -r /home/mariujf/acados/include/acados_c/* include/acados_c/
cp -r /home/mariujf/acados/include/blasfeo/* include/blasfeo/include/
cp -r /home/mariujf/acados/include/hpipm/* include/hpipm/include/

# Copy the acados C source files you actually need (not everything!)
# The exact files depend on your solver configuration.
# Start with these core ones:
cp /home/mariujf/acados/acados/ocp_nlp/ocp_nlp_sqp_rti.c lib/acados_core/src/
cp /home/mariujf/acados/acados/ocp_nlp/ocp_nlp_common.c lib/acados_core/src/
# ... (more files will be needed — see troubleshooting below)
```

**This is where it gets messy.** You'll get undefined reference errors during compilation and need to add more `.c` files. The iterative process is:

1. Try to compile
2. Read the linker error (e.g., `undefined reference to 'ocp_nlp_sqp_rti_create'`)
3. Search the acados source tree for the file containing that function
4. Copy it in
5. Repeat

This is tedious but mechanical. There are typically 20–40 source files needed.

### Option B: Compile acados as a static library using CMake (cleaner but harder)

If you're comfortable with CMake and cross-compilation toolchains, acados provides CMake support:

```bash
cd /home/mariujf/acados
mkdir build_teensy && cd build_teensy

# Get the ARM toolchain path from PlatformIO
TOOLCHAIN=$(find ~/.platformio -name "arm-none-eabi-gcc" | head -1 | xargs dirname)

cmake .. \
    -DCMAKE_C_COMPILER=$TOOLCHAIN/arm-none-eabi-gcc \
    -DCMAKE_SYSTEM_NAME=Generic \
    -DCMAKE_SYSTEM_PROCESSOR=arm \
    -DACADOS_WITH_HPIPM=ON \
    -DBLASFEO_TARGET=GENERIC \
    -DCMAKE_C_FLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Os" \
    -DBUILD_SHARED_LIBS=OFF

make -j$(nproc)
```

This produces `libacados.a`, `libblasfeo.a`, `libhpipm.a` compiled for the Teensy. Copy these into your `lib/` folder.

**This may or may not work cleanly** — acados's CMake wasn't designed for bare-metal ARM targets. Option A (compiling source directly) is more reliable but more manual.

### Option C (Pragmatic): Start on a Raspberry Pi or PC first

If you get stuck on cross-compilation, **skip the Teensy for now**. Run the acados C code on a Raspberry Pi or even on your Linux PC as a standalone C program. This lets you validate the C code works before tackling the embedded compilation. You can communicate with the Teensy over serial (the Teensy handles sensors/actuators, the Pi runs the MPC).

---

## Step 4: Copy the Generated Code into the Project

```bash
cd ~/maglev_teensy_nmpc

# Copy the generated solver code
cp -r c_generated_code/* src/

# Or if you prefer to keep it organized:
mkdir -p lib/maglev_solver
cp c_generated_code/acados_solver_maglev_nmpc.c lib/maglev_solver/
cp c_generated_code/acados_solver_maglev_nmpc.h lib/maglev_solver/
cp -r c_generated_code/maglev_nmpc_model/ lib/maglev_solver/
cp -r c_generated_code/maglev_nmpc_cost/ lib/maglev_solver/
cp -r c_generated_code/maglev_nmpc_constraints/ lib/maglev_solver/
```

---

## Step 5: Write the Teensy Test Sketch

Create `src/main.cpp` (PlatformIO uses `.cpp` by default):

```cpp
#include <Arduino.h>

// The acados-generated solver header
extern "C" {
    #include "acados_solver_maglev_nmpc.h"
}

// Solver capsule (holds all internal memory)
static ocp_nlp_solver_capsule *capsule = nullptr;

// Your equilibrium values (from MATLAB — paste the actual numbers)
static const double xEq[12] = {0, 0, 0.0300, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//                                    ^^^^^^ replace with your actual zEq
static const double uEq[4]  = {0.0, 0.0, 0.0, 0.0};
//                              ^^^ replace with your actual uEq values

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }  // Wait for serial monitor

    Serial.println("=== acados NMPC Timing Test ===");

    // --- Create the solver ---
    Serial.print("Creating solver... ");
    capsule = maglev_nmpc_acados_create_capsule();
    int status = maglev_nmpc_acados_create(capsule);
    if (status != 0) {
        Serial.print("FAILED with status ");
        Serial.println(status);
        while (1) {}  // halt
    }
    Serial.println("OK");

    // --- Print memory usage ---
    Serial.print("Free RAM after solver creation: ");
    // Note: external_RAM_size is for Teensy 4.1 with PSRAM; internal is ~1MB
    // For rough check:
    extern unsigned long _heap_end;
    extern char *__brkval;
    Serial.print((char *)&_heap_end - __brkval);
    Serial.println(" bytes");

    // --- Warm-start at equilibrium ---
    int N = MAGLEV_NMPC_N;  // this macro is defined in the generated header
    for (int k = 0; k <= N; k++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims,
                         capsule->nlp_out, k, "x", (void *)xEq);
    }
    for (int k = 0; k < N; k++) {
        ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims,
                         capsule->nlp_out, k, "u", (void *)uEq);
    }

    // --- Set reference ---
    double yref[16];  // nx + nu = 12 + 4
    for (int i = 0; i < 12; i++) yref[i] = xEq[i];
    for (int i = 0; i < 4; i++)  yref[12 + i] = uEq[i];

    for (int k = 0; k < N; k++) {
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims,
                                capsule->nlp_in, k, "yref", (void *)yref);
    }
    // Terminal reference (only x, no u)
    ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims,
                            capsule->nlp_in, N, "yref", (void *)xEq);

    Serial.println("Solver initialized and warm-started.");
    Serial.println();

    // =========================================
    // TIMING TEST: solve 100 times, report stats
    // =========================================
    Serial.println("Running 100 RTI solves at equilibrium...");

    double x_current[12];
    memcpy(x_current, xEq, sizeof(xEq));

    unsigned long total_us = 0;
    unsigned long worst_us = 0;
    unsigned long best_us  = 999999;

    for (int i = 0; i < 100; i++) {
        // Set initial state constraint
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
                                       capsule->nlp_in, 0, "lbx", (void *)x_current);
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
                                       capsule->nlp_in, 0, "ubx", (void *)x_current);

        // --- SOLVE ---
        unsigned long t0 = micros();
        status = maglev_nmpc_acados_solve(capsule);
        unsigned long dt = micros() - t0;

        total_us += dt;
        if (dt > worst_us) worst_us = dt;
        if (dt < best_us)  best_us  = dt;

        // Read the first control action
        double u_opt[4];
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims,
                         capsule->nlp_out, 0, "u", u_opt);

        if (i < 5 || i == 99) {
            Serial.print("  Solve ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(dt);
            Serial.print(" us, status=");
            Serial.print(status);
            Serial.print(", u=[");
            for (int j = 0; j < 4; j++) {
                Serial.print(u_opt[j], 6);
                if (j < 3) Serial.print(", ");
            }
            Serial.println("]");
        }
    }

    Serial.println();
    Serial.println("=== TIMING RESULTS ===");
    Serial.print("  Average: ");
    Serial.print(total_us / 100);
    Serial.println(" us");
    Serial.print("  Best:    ");
    Serial.print(best_us);
    Serial.println(" us");
    Serial.print("  Worst:   ");
    Serial.print(worst_us);
    Serial.println(" us");
    Serial.print("  Budget:  ");
    Serial.print(10000);  // 10 ms = 100 Hz
    Serial.println(" us (for 100 Hz control)");
    Serial.println();

    if (worst_us < 10000) {
        Serial.println("PASS: Solver fits within 100 Hz budget.");
    } else if (worst_us < 25000) {
        Serial.println("MARGINAL: Solver fits 40 Hz but not 100 Hz.");
    } else {
        Serial.println("FAIL: Solver too slow. Reduce N, num_stages, or num_steps.");
    }
}

void loop() {
    // Nothing — this is just a timing test
}
```

---

## Step 6: Build and Upload

```bash
cd ~/maglev_teensy_nmpc
platformio run --target upload
platformio device monitor --baud 115200
```

You'll likely get compiler errors on the first attempt. See troubleshooting below.

---

## Step 7: Interpret the Results

The serial output tells you everything:

| Result | What to do |
|--------|-----------|
| Average < 2 ms | Excellent. You can run at 200+ Hz with room for sensors/estimation. |
| Average 2–10 ms | Good. 100 Hz is feasible. Proceed to Phase 2. |
| Average 10–25 ms | Workable at 40 Hz. Consider reducing `N` from 10 to 5. |
| Average > 25 ms | Too slow. Major reductions needed, or offload to companion computer. |
| `u ≈ uEq` | Correct — at equilibrium the optimal action is the equilibrium input. |
| `u` is NaN or huge | Solver is broken. Check that `xEq` and `uEq` match your MATLAB values. |

---

## Troubleshooting

### "undefined reference to ..." errors

This means a C source file is missing. Example:

```
undefined reference to `d_ocp_qp_ipm_arg_set_default'
```

Search for it:

```bash
grep -r "d_ocp_qp_ipm_arg_set_default" /home/mariujf/acados/ --include="*.c" -l
```

Copy the file it finds into your project.

### "not enough RAM" or hardfault on startup

The Teensy 4.1 has 1 MB internal RAM but your solver may need more if `N`, `nx`, and `num_steps` are large. Options:
- Add PSRAM chip to the Teensy 4.1 (gives you 8 or 16 MB)
- Reduce `N` from 10 to 5
- Reduce `num_steps` from 10 to 1–3

### Architecture-related errors in BLASFEO

If you see errors about NEON intrinsics or assembly instructions:

```
error: impossible constraint in 'asm'
```

Change the BLASFEO target to `GENERIC` in your build flags:

```
-DBLASFEO_TARGET_GENERIC
```

This uses plain C instead of ARM-specific assembly. It's ~2–3x slower than an optimized target but it will compile.

### "multiple definition of main"

The generated `main_maglev_nmpc.c` contains its own `main()` function. Either delete it from your project, or rename it:

```bash
rm src/main_maglev_nmpc.c
# or
mv src/main_maglev_nmpc.c src/main_maglev_nmpc.c.example
```

### Header not found errors

Make sure your `platformio.ini` `build_flags` include `-I` paths pointing to every directory that contains `.h` files. The generated code includes headers like `"acados/utils/math.h"` and `"blasfeo/include/blasfeo_d_aux.h"`.

---

## What Comes After This Works

Once you have timing numbers, the next concrete step is wiring the solver into your existing control loop. Your `.ino` changes minimally:

```
BEFORE:  pwmInputX = -Kp * magFieldX - Kd * dMagFieldX;
AFTER:   x_estimate = some_function_of(magFieldX, magFieldY, magFieldZ, ...);
         set solver initial state to x_estimate;
         solve;
         u_optimal = get solver output;
         pwmInputX = current_to_pwm(u_optimal[0] - u_optimal[1]);
```

But that's Phase 2 — don't worry about it until you have the timing test passing.

---

## Summary: What You're Actually Doing

| Step | What | Time estimate |
|------|------|---------------|
| 0 | Install PlatformIO | 10 min |
| 1 | Modify MATLAB script and generate C code | 30 min |
| 2 | Understand generated files (read `main_maglev_nmpc.c`) | 30 min |
| 3 | Set up project + copy acados source/headers | 2–4 hours |
| 4 | Copy generated code into project | 10 min |
| 5 | Write test sketch (provided above) | 10 min (copy-paste) |
| 6 | Iterate on build errors | 2–8 hours (most time spent here) |
| 7 | Upload and read timing | 5 min |

Step 6 is where 80% of the effort goes. It's not intellectually hard — it's just chasing missing files and fixing include paths. Having the acados source tree open in a file browser and using `grep` liberally is the way through it.
