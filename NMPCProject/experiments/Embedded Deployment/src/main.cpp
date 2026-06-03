#include <Arduino.h>

#undef B1

// ============================================
// MALLOC TRACKER - prints IMMEDIATELY on failure
// ============================================
static size_t total_allocated = 0;
static int alloc_count = 0;

extern "C" {
    void *__real_malloc(size_t size);
    void __real_free(void *ptr);
    void *__real_calloc(size_t nmemb, size_t size);

    void *__wrap_malloc(size_t size) {
        void *ptr = __real_malloc(size);
        alloc_count++;
        if (ptr) {
            total_allocated += size;
            // Print every 20th allocation to track progress
            if (alloc_count % 20 == 0) {
                Serial.print("  malloc #"); Serial.print(alloc_count);
                Serial.print(" size="); Serial.print(size);
                Serial.print(" total="); Serial.print(total_allocated/1024);
                Serial.println("KB");
                Serial.flush();
            }
        } else {
            // PRINT IMMEDIATELY before acados can dereference NULL
            Serial.print("*** MALLOC FAILED *** #"); Serial.print(alloc_count);
            Serial.print(" requested="); Serial.print(size);
            Serial.print(" bytes, total_so_far="); Serial.print(total_allocated/1024);
            Serial.println(" KB");
            Serial.flush();
            delay(100);  // ensure it transmits
        }
        return ptr;
    }

    void __wrap_free(void *ptr) {
        __real_free(ptr);
    }

    void *__wrap_calloc(size_t nmemb, size_t size) {
        size_t total_size = nmemb * size;
        void *ptr = __real_calloc(nmemb, size);
        alloc_count++;
        if (ptr) {
            total_allocated += total_size;
            if (alloc_count % 20 == 0) {
                Serial.print("  calloc #"); Serial.print(alloc_count);
                Serial.print(" size="); Serial.print(total_size);
                Serial.print(" total="); Serial.print(total_allocated/1024);
                Serial.println("KB");
                Serial.flush();
            }
        } else {
            Serial.print("*** CALLOC FAILED *** #"); Serial.print(alloc_count);
            Serial.print(" requested="); Serial.print(total_size);
            Serial.print(" bytes, total_so_far="); Serial.print(total_allocated/1024);
            Serial.println(" KB");
            Serial.flush();
            delay(100);
        }
        return ptr;
    }
}

#include "acados_solver_maglev_nmpc_reduced_mapped.h"

static maglev_nmpc_reduced_mapped_solver_capsule *capsule = nullptr;
static const double xEq[10] = {0, 0, 0.0301191833, 0, 0, 0, 0, 0, 0, 0};
static const double uEq[4]  = {0, 0, 0, 0};

void dbg(const char* msg) {
    Serial.println(msg);
    Serial.flush();
    delay(20);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    if (CrashReport) {
        Serial.println("=== CRASH REPORT FROM PREVIOUS RUN ===");
        Serial.println(CrashReport);
        Serial.println("======================================");
        Serial.flush();
    }

    dbg("=== acados Memory Diagnostic v2 (nx=10) ===");
    total_allocated = 0;
    alloc_count = 0;

    dbg("[1] Creating capsule...");
    capsule = maglev_nmpc_reduced_mapped_acados_create_capsule();
    dbg("    Capsule OK");

    dbg("[2] Calling acados_create (tracking every allocation)...");
    int status = maglev_nmpc_reduced_mapped_acados_create(capsule);

    // If we reach here, it worked!
    Serial.print("    acados_create returned: "); Serial.println(status);
    Serial.print("    Total allocations: "); Serial.println(alloc_count);
    Serial.print("    Total memory: "); Serial.print(total_allocated/1024); Serial.println(" KB");
    Serial.flush();

    if (status != 0) {
        Serial.print("    FAILED with status "); Serial.println(status);
        while (1) {}
    }

    dbg("\n=== SOLVER CREATED SUCCESSFULLY ===");

    ocp_nlp_config *config = maglev_nmpc_reduced_mapped_acados_get_nlp_config(capsule);
    ocp_nlp_dims *dims     = maglev_nmpc_reduced_mapped_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in     = maglev_nmpc_reduced_mapped_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out   = maglev_nmpc_reduced_mapped_acados_get_nlp_out(capsule);
    int N = MAGLEV_NMPC_REDUCED_MAPPED_N;

    for (int k = 0; k <= N; k++)
        ocp_nlp_out_set(config, dims, nlp_out, nlp_in, k, "x", (void *)xEq);
    for (int k = 0; k < N; k++)
        ocp_nlp_out_set(config, dims, nlp_out, nlp_in, k, "u", (void *)uEq);

    double yref[14];
    for (int i = 0; i < 10; i++) yref[i] = xEq[i];
    for (int i = 0; i < 4; i++)  yref[10 + i] = uEq[i];
    for (int k = 0; k < N; k++)
        ocp_nlp_cost_model_set(config, dims, nlp_in, k, "yref", (void *)yref);
    ocp_nlp_cost_model_set(config, dims, nlp_in, N, "yref", (void *)xEq);

    dbg("\n[3] Running 100 RTI solves at equilibrium...");
    double x_current[10];
    memcpy(x_current, xEq, sizeof(xEq));
    unsigned long total_us = 0, worst_us = 0, best_us = 999999;

    for (int i = 0; i < 100; i++) {
        ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, "lbx", (void *)x_current);
        ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, "ubx", (void *)x_current);
        unsigned long t0 = micros();
        status = maglev_nmpc_reduced_mapped_acados_solve(capsule);
        unsigned long dt = micros() - t0;
        total_us += dt;
        if (dt > worst_us) worst_us = dt;
        if (dt < best_us) best_us = dt;

        double u_opt[4];
        ocp_nlp_out_get(config, dims, nlp_out, 0, "u", u_opt);
        if (i < 5 || i == 99) {
            Serial.print("  Solve "); Serial.print(i);
            Serial.print(": "); Serial.print(dt);
            Serial.print(" us, status="); Serial.print(status);
            Serial.print(", u=[");
            for (int j = 0; j < 4; j++) {
                Serial.print(u_opt[j], 6);
                if (j < 3) Serial.print(", ");
            }
            Serial.println("]"); Serial.flush();
        }
    }

    Serial.println("\n=== TIMING RESULTS ===");
    Serial.print("  Average: "); Serial.print(total_us / 100); Serial.println(" us");
    Serial.print("  Best:    "); Serial.print(best_us); Serial.println(" us");
    Serial.print("  Worst:   "); Serial.print(worst_us); Serial.println(" us");
    Serial.println("  Budget:  10000 us (100 Hz)");
    if (worst_us < 10000) Serial.println("  VERDICT: PASS - fits 100 Hz");
    else if (worst_us < 25000) Serial.println("  VERDICT: MARGINAL - fits 40 Hz");
    else Serial.println("  VERDICT: FAIL - too slow");
    Serial.println("\n=== TEST COMPLETE ===");
    Serial.flush();
}

void loop() {}