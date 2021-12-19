#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int dimension, bool is3d);
	float getTemperature(int i, int j,int k);
	void setTemperature(int i, int j, int k, float temperature);
	int getDimension();
	void set3D(bool d);
	bool is3D();

private:
	// Attributes
	bool _3d;
	int dimension;
	vector<vector<vector<float>>> grid;
};




class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float deltaTime);
	void diffuseTemperatureImplicit(float deltaTime);
	Vec3 mapFloatToColor(float value);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid* T; //save results of every time step
	float alpha;
	float alphaVar;
	bool precomputeA;// independent of timestep
	SparseMatrix<Real>* staticA;

	float minTemp;
	float maxTemp;
};

#endif