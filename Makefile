GUROBI = C:\gurobi903\win64
INC = C:\gurobi903\win64\include
CPPLIB = -L C:\gurobi903\win64\lib -lgurobi_c++mdd2019 -lgurobi90
all:
	g++ -o Main Main.cpp -O3 -IC:\Program Files\Metis 5.1.0\include -LC:\Program Files\Metis 5.1.0\lib -l metis -l metis -I$(INC) $(CPPLIB) -m64
