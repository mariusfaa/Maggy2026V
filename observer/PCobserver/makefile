# =============================================================================
# Makefile
#
# Targets
#   make sim       – build simulator binary (original workflow, unchanged)
#   make lib       – build libobserver.so with static openblas/lapack/openmp
#   make all       – both
#   make clean
#
# =============================================================================

CXX      := g++
CXXFLAGS := -std=c++17 -O2

# ---------------------------------------------------------------------------
# Paths  –  edit these
# ---------------------------------------------------------------------------

# project headers and source files
INCLUDE_DIRS := -I./include

# Static libraries
#
# OpenBLAS static:
#   - often at /usr/lib/x86_64-linux-gnu/libopenblas.a
#   - or build from source: https://github.com/OpenMathLib/OpenBLAS
#     (make NO_SHARED=1; gives OpenBLAS/libopenblas.a)
#
# LAPACK static:
#   - often at /usr/lib/x86_64-linux-gnu/liblapack.a
#   - or bundled inside OpenBLAS (use -lopenblas only)
#
# libgomp static:
#   - at $(dirname $(gcc --print-libgcc-file-name))/libgomp.a
#     run: gcc --print-file-name=libgomp.a  to find it
OPENBLAS_STATIC := /usr/lib/x86_64-linux-gnu/libopenblas.a
LIBGOMP_STATIC  := $(shell $(CXX) --print-file-name=libgomp.a)

MODEL_LIB_DIR  := ./lib
MODEL_LIB      := -lmaglevModel

# Source files shared between sim and lib
COMMON_SRCS := matrices.cpp observer.cpp utilities.cpp

# ---------------------------------------------------------------------------
# Simulator target
# ---------------------------------------------------------------------------
.PHONY: sim
sim:
	$(CXX) $(CXXFLAGS) \
	    $(COMMON_SRCS) \
	    $(INCLUDE_DIRS) \
	    -L$(MODEL_LIB_DIR) $(MODEL_LIB) \
	    -larmadillo \
	    simulator.cpp \
	    -o target/sim

# ---------------------------------------------------------------------------
# Shared library target  –  embeds openblas / lapack / openmp statically
#
# Key flags explained:
#   -DARMA_DONT_USE_WRAPPER          bypass libarmadillo.so; link BLAS directly
#   -Wl,--whole-archive ... \
#   -Wl,--no-whole-archive           force all symbols from .a into the .so
#                                    (needed so MATLAB can find them at runtime)
#   -Wl,-Bstatic ... -Wl,-Bdynamic   bracket: link these libs static only
#   -static-libgcc -static-libstdc++ bundle GCC runtime (avoids version mismatch)
#   -Wl,--exclude-libs,ALL           hide internal symbols of bundled .a files
#                                    (prevents symbol clashes if MATLAB loads
#                                     its own openblas)
# ---------------------------------------------------------------------------
.PHONY: lib
lib: target/libobserver.so

target/libobserver.so: $(COMMON_SRCS) observer_capi.cpp
	@mkdir -p target
	$(CXX) $(CXXFLAGS) \
	    -shared -fPIC \
	    -DARMA_DONT_USE_WRAPPER \
	    $(INCLUDE_DIRS) \
	    $(COMMON_SRCS) observer_capi.cpp \
	    -L$(MODEL_LIB_DIR) $(MODEL_LIB) \
	    -Wl,--whole-archive \
	        $(OPENBLAS_STATIC) \
	    -Wl,--no-whole-archive \
	    -Wl,--exclude-libs,ALL \
	    -static-libgcc \
	    -static-libstdc++ \
	    -lpthread -lm -ldl \
	    -o $@


.PHONY: all
all: sim lib

.PHONY: clean
clean:
	rm -f target/sim target/libobserver.so

# ---------------------------------------------------------------------------
# Verify no unwanted dynamic deps after building
#   make check-deps
# ---------------------------------------------------------------------------
.PHONY: check-deps
check-deps: target/libobserver.so
	@echo "=== Dynamic dependencies of libobserver.so ==="
	@ldd target/libobserver.so
	@echo ""
	@echo "=== Should only see: linux-vdso, libpthread, libm, libdl, libc ==="
