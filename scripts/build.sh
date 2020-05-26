mkdir -p build/terminal
cd  build/terminal
cmake ../.. -GNinja -DCMAKE_BUILD_TYPE=Release -DYOCTO_EMBREE=ON
cmake --build . --parallel 8
