tips for installation

```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON -DACADOS_WITH_OPENMP=ON ..
make install -j14
```
