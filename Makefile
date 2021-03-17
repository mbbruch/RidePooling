GUROBI = /opt/packages/gurobi/gurobi911/linux64
INC = /opt/packages/gurobi/gurobi911/linux64/include
CPPLIB = -L /opt/packages/gurobi/gurobi911/linux64/lib -l gurobi91 -L /ocean/projects/eng200002p/mbruchon/gurobi/gurobi911/linux64/src/build -l gurobi_c++17
IPATHS = -I/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/h -I/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/include
LPATHS = -L/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin -Wl,-rpath-link,/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin '-Wl,-rpath=/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin'

all:
	g++ -c globals.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c GPtree.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c Request.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c RTV.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) $(IPATHS) $(LPATHS) -lfusion64 -lmosek64 -m64 -std=c++17
	g++ -c RV.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c travel.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c util.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c Vehicle.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ -c Main.cpp -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
	g++ Main.o globals.o GPtree.o Request.o RTV.o RV.o travel.o util.o Vehicle.o -o main -O3 -fopenmp -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis -lstdc++fs -I $(INC) $(CPPLIB) -m64 -std=c++17
