CPPFLAGS= -O3 -Wall -fpic -march=native -Wno-narrowing -pthread -std=c++17 -w

all: clean dir xml serialarm executable

dir:
	mkdir build

xml:
	g++ src/tinyxml2.cpp -o build/tinyxml2.o -c $(CPPFLAGS)

serialarm:
	g++ src/SerialArm.cpp -o build/serialarm_o.o -c $(CPPFLAGS)
	ld -r build/serialarm_o.o build/tinyxml2.o -o build/serialarm.o

executable:
	g++ src/main.cpp -o build/main.o -c $(CPPFLAGS)
	g++ build/serialarm.o build/main.o -o build/main $(CPPFLAGS) -lcln -lginac -static-libstdc++

clean:
	rm -rf build
