mkdir -p build/terminal
cd  build/terminal
cmake ../.. -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel 8
