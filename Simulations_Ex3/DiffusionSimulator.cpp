#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <format>
using namespace std;

#define dim3_2d 0	//refactor for 3d
#define staticTimeStep 0.001

Grid::Grid(int Dimension, bool is3d = false) {
	grid = vector<vector<vector<float>>>();
	dimension = Dimension;
	_3d = is3d;
	for (int i = 0; i < Dimension; i++) {
		vector<vector<float>> v = vector<vector<float>>();
		for (int j = 0; j < Dimension; j++) {
			v.push_back(vector<float>(Dimension, 0.0));
		}
		grid.push_back(v);
	}
}

float Grid::getTemperature(int i, int j, int k) {
	// edge case
	if (i < 0 || i >= grid.size() || j < 0 || j >= grid.size()) {
		if (_3d && (k >= 0 && k < grid.size()))
			;
		else {
			return 0;
		}
	}
	return grid[i][j][k];
}

void Grid::setTemperature(int i, int j, int k, float temperature) {
	// outside
	if (i < 0 || i >= grid.size() || j < 0 || j >= grid.size()) {
		if (_3d && (k >= 0 && k < grid.size()))
			;
		else {
			return;
		}
	}
	grid[i][j][k] = temperature;
}

int Grid::getDimension() {
	return dimension;
}

void Grid::set3D(bool b) {
	_3d = b;
}

bool Grid::is3D() {
	return _3d;
}

void setupA(SparseMatrix<Real>& A, Grid* T, double deltaTime, double alpha, double deltaX);
void setupA_3D(SparseMatrix<Real>& A, Grid* T, double deltaTime, double alpha, double deltaX);
void printA(SparseMatrix<Real>* A, Grid* T);

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	alpha = 1;
	alphaVar = alpha;
	precomputeA = true;
	minTemp = 0;
	maxTemp = 100;
	// to be implemented
	notifyCaseChanged(0);
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver, Explicit_solver_unstable!, Implicit_solver_stable!, Explicit_3D, Implicit_3D";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alphaVar, "");
	TwAddVarRW(DUC->g_pTweakBar, "precompute A", TW_TYPE_BOOL8, &precomputeA, "");

	TwAddVarRW(DUC->g_pTweakBar, "minTemp", TW_TYPE_FLOAT, &minTemp, "");
	TwAddVarRW(DUC->g_pTweakBar, "maxTemp", TW_TYPE_FLOAT, &maxTemp, "");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);

	int N = 0;

	switch (m_iTestCase)
	{

	case 0:
		cout << "Explicit solver!\n";
		T = new Grid(16);
		T->setTemperature(8, 8, dim3_2d, 10000);
		break;

	case 1:
		cout << "Implicit solver!\n";
		T = new Grid(16);
		N = T->getDimension() * T->getDimension();
		staticA = new SparseMatrix<Real>(N);
		
		setupA(*staticA, T, staticTimeStep, alpha, 1);
		T->setTemperature(8, 8, dim3_2d, 10000);
		break;
	case 2:
		alpha = 300;
		cout << "Explicit solver!\n";
		T = new Grid(16);
		T->setTemperature(8, 8, dim3_2d, 10000);
		break;
	case 3:
		alpha = 300;
		cout << "Implicit solver!\n";
		T = new Grid(16);
		N = T->getDimension() * T->getDimension();
		staticA = new SparseMatrix<Real>(N);
		setupA(*staticA, T, staticTimeStep, alpha, 1);
		T->setTemperature(8, 8, dim3_2d, 10000);
		break;
	case 4:
		cout << "Explicit solver! 3D\n";
		T = new Grid(16,true);
		T->setTemperature(8, 8, 8, 10000);
		break;
	case 5:
		cout << "Implicit solver!\n";
		T = new Grid(16,true);
		N = T->getDimension() * T->getDimension() * T->getDimension();
		staticA = new SparseMatrix<Real>(N);
		setupA_3D(*staticA, T, staticTimeStep, alpha, 1);
		T->setTemperature(8, 8, 8, 10000);
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
	alphaVar = alpha;
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float deltaTime) {//add your own parameters
	Grid* newT = new Grid(T->getDimension(),T->is3D());

	float deltaXSquared = 1;//pow(1.0 / T->getDimension(),2);
	float deltaYSquared = 1;//pow(1.0 / T->getDimension(), 2);
	float deltaZSquared = 1;//pow(1.0 / T->getDimension(), 2);

	if (T->is3D()) {
		for (int i = 1; i < T->getDimension() - 1; i++) {
			for (int j = 1; j < T->getDimension() - 1; j++) {
				for (int k = 1; k < T->getDimension() - 1; k++) {
					float Tijk = T->getTemperature(i, j, k);
					newT->setTemperature(i, j, k, Tijk + alpha * deltaTime *
						(((T->getTemperature(i + 1, j, k) - 2 * Tijk + T->getTemperature(i - 1, j, k)) / deltaXSquared) +	//dx
							((T->getTemperature(i, j + 1, k) - 2 * Tijk + T->getTemperature(i, j - 1, k)) / deltaYSquared) +	//dy
							((T->getTemperature(i, j, k + 1) - 2 * Tijk + T->getTemperature(i, j, k - 1)) / deltaZSquared))		//dz
					);
				}
			}
		}
	}
	else {
		for (int i = 1; i < T->getDimension() - 1; i++) {
			for (int j = 1; j < T->getDimension() - 1; j++) {
				float Tij = T->getTemperature(i, j, dim3_2d);
				newT->setTemperature(i, j, dim3_2d, Tij + alpha * deltaTime *
					(((T->getTemperature(i + 1, j, dim3_2d) - 2 * Tij + T->getTemperature(i - 1, j, dim3_2d)) / deltaXSquared) +	//dx
						((T->getTemperature(i, j + 1, dim3_2d) - 2 * Tij + T->getTemperature(i, j - 1, dim3_2d)) / deltaYSquared))	//dy
				);
			}
		}
	}

	
	return newT;
}

void setupB(std::vector<Real>& b, Grid* T) {//add your own parameters
	int z = 0;
	for (int i = 0; i < T->getDimension(); i++) {
		for (int j = 0; j < T->getDimension(); j++) {
			b.at(z) = T->getTemperature(i, j, dim3_2d);
			z++;
		}
	}
	
}

void fillT(vector<Real> x, Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	int z = 0;
	for (int i = 0; i < T->getDimension(); i++) {
		for (int j = 0; j < T->getDimension(); j++) {
			T->setTemperature(i, j, dim3_2d, x.at(z));
			z++;
		}
	}
}

void setupA(SparseMatrix<Real>& A, Grid* T, double deltaTime, double alpha, double deltaX) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	const int N = T->getDimension() * T->getDimension();

	double F = alpha * deltaTime;

	int t1 = 0;
	int t2 = 0;
	for (int i = 0; i < N; i++) {		//row
		bool rand = false;
		if (t2 == T->getDimension())
		{
			t1++;
			t2 = 0;
		}
		if (t1==0 || t2==0 || t1 == T->getDimension()-1 || t2 == T->getDimension()-1) {	//Rand
			rand = true;
		}
		int a1 = 0;
		int a2 = 0;
		for (int j = 0; j < N; j++) {
			if (a2 == T->getDimension())
			{
				a1++;
				a2 = 0;
			}
			if (a1 == t1 && a2 == t2) {				// i,j
				if(rand)
					A.set_element(i, j, 1);
				else
					A.set_element(i, j, 1. + 4. * F);
			}
							// i-1,j						// i+1,j					// i,j-1					// i,j+1
			else if (a1 == t1 - 1 && a2 == t2 || a1 == t1 + 1 && a2 == t2 || a1 == t1 && a2 == t2 - 1 || a1 == t1 && a2 == t2 + 1) {	
				if (rand)
					A.set_element(i, j, 0);
				else
					A.set_element(i, j, -F);
			}
			a2++;
		}
		t2++;
	}


	/*
	[1,0,0,0,0...
	 -F, 1+2F , -F   ,  0,...
	 0 , -F   , 1+2F , -F, 0...
	 0, 0,-F, 1+2F, -F,0,
	 ...
	0,0,0,0,0,0,0,0,1]
	*/

}

void printA(SparseMatrix<Real>* A, Grid* T){
	int a1 = 0;
	int a2 = 0;
	printf("%7s|","");
	for (int i = 0; i < T->getDimension() * T->getDimension(); i++) {
		if (a2 == T->getDimension())
		{
			a1++;
			a2 = 0;
		}
		printf("%3d,%-3d|",a1,a2);
		a2++;
	}
	printf("\n");
	printf("%7s|", "");
	for (int i = 0; i < T->getDimension() * T->getDimension(); i++) {
		printf("--------");
	}
	a1 = 0; a2 = 0;
	printf("\n");
	for (int i = 0; i < T->getDimension()*T->getDimension(); i++) {
		int j = 0;
		int z = 0;
		if (a2 == T->getDimension())
		{
			a1++;
			a2 = 0;
		}
		printf("%3d,%-3d|", a1, a2);
		for each(int index in A->index[i]) {
			for (j; j < index; j++) {
				printf("%6d |",0);
			}
			char b[100];
			printf("% 6.3f |",A->value[i][z++]);
			j++;
		}
		for (j; j < T->getDimension() * T->getDimension();j++) {
			printf("%6d |", 0);
		}
		printf("\n");
		a2++;
		
	}
	printf("%7s|", "");
	for (int i = 0; i < T->getDimension() * T->getDimension(); i++) {
		printf("--------");
	}
	printf("\n");
}

// 3D

void setupB_3D(std::vector<Real>& b, Grid* T) {//add your own parameters
	int z = 0;
	for (int i = 0; i < T->getDimension(); i++) {
		for (int j = 0; j < T->getDimension(); j++) {
			for (int k = 0; k < T->getDimension(); k++) {
				b.at(z) = T->getTemperature(i, j, k);
				z++;
			}
		}
	}

}

void fillT_3D(vector<Real> x, Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	int z = 0;
	for (int i = 0; i < T->getDimension(); i++) {
		for (int j = 0; j < T->getDimension(); j++) {
			for (int k = 0; k < T->getDimension(); k++) {
				T->setTemperature(i, j, k, x.at(z));
				z++;
			}
		}
	}
}

void setupA_3D(SparseMatrix<Real>& A, Grid* T, double deltaTime, double alpha, double deltaX) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	const int N = T->getDimension() * T->getDimension() * T->getDimension();

	double F = alpha * deltaTime;

	int t1 = 0;
	int t2 = 0;
	int t3 = 0;
	for (int i = 0; i < N; i++) {		//row
		bool rand = false;
		if (t3 == T->getDimension())
		{
			t2++;
			t3 = 0;
			if (t2 == T->getDimension()) {
				t1++;
				t2 = 0;
			}
		}
		if (t1 == 0 || t2 == 0 || t1 == T->getDimension() - 1 || t2 == T->getDimension() - 1 || t3==0 || t3 == T->getDimension()-1) {	//Rand
			rand = true;
		}
		int a1 = 0;
		int a2 = 0;
		int a3 = 0;
		for (int j = 0; j < N; j++) {
			if (a3 == T->getDimension())
			{
				a2++;
				a3 = 0;
				if (a2 == T->getDimension()) {
					a1++;
					a2 = 0;
				}
			}
			if (a1 == t1 && a2 == t2 && a3 == t3) {				// i,j,k
				if (rand)
					A.set_element(i, j, 1);
				else
					A.set_element(i, j, 1. + 6. * F);
			}
									// i-1,j,k							// i+1,j							// i,j-1,k									// i,j+1,k									// i,j,k-1							// i,j,k+1	
			else if (a1 == t1 - 1 && a2 == t2 && a3 == t3 || a1 == t1 + 1 && a2 == t2 && a3 == t3 || a1 == t1 && a2 == t2 - 1 && a3 == t3 || a1 == t1 && a2 == t2 + 1 && a3 == t3 || a1 == t1 && a2 == t2 && a3 == t3 - 1 || a1 == t1 && a2 == t2 && a3 == t3 + 1) {
				if (rand)
					A.set_element(i, j, 0);
				else
					A.set_element(i, j, -F);
			}
			a3++;
		}
		t3++;
	}


	/*
	[1,0,0,0,0...
	 -F, 1+2F , -F   ,  0,...
	 0 , -F   , 1+2F , -F, 0...
	 ...
	0,0,0,0,0,0,0,0,1]
	*/

}


void DiffusionSimulator::diffuseTemperatureImplicit(float deltaTime) {//add your own parameters
	// solve A T = b
	if (T->is3D()) {
		const int N = T->getDimension() * T->getDimension() * T->getDimension();
		SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
		std::vector<Real>* b = new std::vector<Real>(N);

		if (precomputeA) {
			A = staticA;
		}
		else {
			A = new SparseMatrix<Real>(N);
			setupA_3D(*A, T, alpha, deltaTime, 1);
		}
		setupB_3D(*b, T);

		// perform solve
		Real pcg_target_residual = 1e-05;
		Real pcg_max_iterations = 1000;
		Real ret_pcg_residual = 1e10;
		int  ret_pcg_iterations = -1;

		SparsePCGSolver<Real> solver;
		solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

		std::vector<Real> x(N);
		for (int j = 0; j < N; ++j) { x[j] = 0.; }

		// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
		solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
		// x contains the new temperature values
		fillT_3D(x, T);//copy x to T
	}
	else {
		const int N = T->getDimension() * T->getDimension();
		SparseMatrix<Real>* A;
		std::vector<Real>* b = new std::vector<Real>(N);

		if (precomputeA) {
			A = staticA;
		}else{
			A = new SparseMatrix<Real>(N);
			setupA(*A, T, alpha, deltaTime, 1);
		}
		setupB(*b, T);

		// perform solve
		Real pcg_target_residual = 1e-05;
		Real pcg_max_iterations = 1000;
		Real ret_pcg_residual = 1e10;
		int  ret_pcg_iterations = -1;

		SparsePCGSolver<Real> solver;
		solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

		std::vector<Real> x(N);
		for (int j = 0; j < N; ++j) { x[j] = 0.; }

		// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
		solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
		// x contains the new temperature values
		fillT(x, T);//copy x to T
	}
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	if (alphaVar != alpha) {
		alpha = alphaVar;
		if (T->is3D()) {
			setupA_3D(*staticA, T, staticTimeStep, alpha, 1);
		}
		else {
			setupA(*staticA, T, staticTimeStep, alpha, 1);
		}
	}
	switch (m_iTestCase)
	{
	case 0:
	case 2:
	case 4:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
	case 3:
	case 5:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	float maxT = 100;
	float minT = 0;
	int dim = T->getDimension();
	float size = 1.0 / dim;
	float offset = (size*dim) / 2.0;
	if (T->is3D()) {
		for (int i = 0; i < T->getDimension(); i++) {
			for (int j = 0; j < T->getDimension(); j++) {
				for (int k = 0; k < T->getDimension(); k++) {
					DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * mapFloatToColor(T->getTemperature(i, j, k)));
					DUC->drawSphere(Vec3(size * i - offset, size * j - offset, size * k - offset), Vec3(size/10));
				}
			}
		}
	}
	else {
		for (int i = 0; i < T->getDimension(); i++) {
			for (int j = 0; j < T->getDimension(); j++) {
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * mapFloatToColor(T->getTemperature(i, j, dim3_2d)));
				DUC->drawSphere(Vec3(size * i - offset, size * j - offset, 0), Vec3(size));
			}
		}
	}
}

Vec3 DiffusionSimulator::mapFloatToColor(float value) {
	/*float ratio = 2 * (value - minimum) / (maximum - minimum);
	float b = max(0.0, 255.0 * (1 - ratio));
	float r = max(0.0, 255.0 * (ratio - 1));
	float g = 255 - b - r;
	return Vec3(r, g, b);*/

	float r, g, b;
	r = ((value - minTemp)) / (maxTemp - minTemp);
	g = 0;//g = 1-((value - minimum)) / (maximum - minimum);
	b = 1-((value - minTemp)) / (maxTemp - minTemp);
	return Vec3(r, g, b);
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
