GUROBI = -I/ocean/projects/eng200002p/mbruchon/gurobi/gurobi950/linux64/include -L /ocean/projects/eng200002p/mbruchon/gurobi/gurobi950/linux64/lib -l gurobi95 -L /ocean/projects/eng200002p/mbruchon/gurobi/gurobi950/linux64/lib -l gurobi_c++
BOOST = -I/opt/packages/boost/boost_1_75_0/include/boost/container -L/opt/packages/boost/boost_1_75_0/lib -l boost_container
TBB = -I/opt/packages/intel/tbb/include -L /opt/packages/intel/tbb/lib/intel64_lin/gcc4.8 -l tbb 
MOSEK = -I/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/h -I/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/include -L/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin -Wl,-rpath-link,/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin '-Wl,-rpath=/ocean/projects/eng200002p/mbruchon/mosek/9.2/tools/platform/linux64x86/bin'
METIS = -I/ocean/projects/eng200002p/mbruchon/metis/include -L/ocean/projects/eng200002p/mbruchon/metis/lib -l metis
OPTIONS = -O3 -std=c++17 -fopenmp -lstdc++fs
all:
	g++ -c globals.cpp $(OPTIONS) $(BOOST)
	g++ -c Graph.cpp $(OPTIONS) $(METIS)
	g++ -c GPtree.cpp $(OPTIONS) 
	g++ -c Request.cpp $(OPTIONS) 
	g++ -c RTV.cpp $(OPTIONS) $(GUROBI)
	g++ -c RV.cpp $(OPTIONS) 
	g++ -c travel.cpp $(OPTIONS)
	g++ -c util.cpp $(OPTIONS)
	g++ -c Vehicle.cpp $(OPTIONS) 
	g++ -c Main.cpp $(OPTIONS) $(GUROBI)
	g++ Main.o globals.o Graph.o GPtree.o Request.o RTV.o RV.o travel.o util.o Vehicle.o -o main $(OPTIONS) $(GUROBI) $(METIS)