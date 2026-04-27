gcc testgenfuncs.c -I./include -L./lib -Wl,-rpath,'$ORIGIN/lib' -lmaglevModel  -fopenmp -g -o run


# -g for debug symbols. -O2 for optimizations
g++ -o run -g -O2 run.cpp matrices.cpp observer.cpp utilities.cpp -larmadillo


g++ -g -O2 matrices.cpp observer.cpp utilities.cpp -larmadillo -I./include -L./lib -Wl,-rpath,'$ORIGIN/lib' -lmaglevModel  -fopenmp -o run run.cpp

# for static library maglevModel.a
g++ -g matrices.cpp observer.cpp utilities.cpp -I./include -L./lib -larmadillo -lmaglevModel -fopenmp -O2 simulator.cpp -o target/sim
