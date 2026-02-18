gcc testgenfuncs.c -I./include -L./lib -Wl,-rpath,'$ORIGIN/lib' -lmaglevModel  -fopenmp -g -o run


# -g for debug symbols. -O2 for optimizations
g++ -o run -g -O2 run.cpp matrices.cpp observer.cpp utilities.cpp -larmadillo
