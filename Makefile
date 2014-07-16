viewer:	viewer.cpp
	g++ main.cpp -lopencv_highgui -lopencv_core -o viewer
	./viewer 0.jpg
testd: test.cpp
	g++ test.cpp -g -o test
test: fmatrixmodel.h ransac.h test.cpp
	g++ -g test.cpp -lopencv_highgui -lopencv_core -o test
	./test
dtest: testd
	gdb test
3drestructure: 3drestructure.cpp
	g++ 3drestructure.cpp -lopencv_highgui -lopencv_core -lopencv_features2d -lopencv_nonfree -o 3drestructure	
3drestructure_mp: 3drestructure.cpp
	g++ 3drestructure.cpp -lopencv_highgui -lopencv_core -lopencv_features2d -lopencv_nonfree -o 3drestructure_mp -fopenmp
matrixtest:matrixtest.cpp
	g++ matrixtest.cpp -lopencv_highgui -lopencv_core -o matrixtest
dottest:dottest.cpp
	g++ dottest.cpp -lopencv_highgui -lopencv_core -o dottest

oclsurf:oclsurf.cpp
	 g++ oclsurf.cpp -lopencv_highgui -lopencv_core -lopencv_ocl -lopencv_features2d -lopencv_nonfree -o oclsurf
