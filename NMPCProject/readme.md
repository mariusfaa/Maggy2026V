tips for installation

Install gcc-13, default gcc for wsl is pretty old, gcc-13 might give a bit faster compiletime

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-13 g++-13
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13
```

```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON -DACADOS_WITH_OPENMP=ON ..
make install -j14
```
