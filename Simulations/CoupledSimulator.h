#include "Simulator.h"
#include "rigidBodySystem.hpp"

#define TESTCASEUSEDTORUNTEST 2

#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2


struct Spring {
	RigidBody* p1;
	RigidBody* p2;
	float initialLength;
	float stiffness;
};

class CoupledSimulator :public Simulator {
public:
	// Construtors
	CoupledSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	// Specific Functions
	void addSpring(int rb1, int rb2, float initialLength, float stiffness);
	int getNumberOfSprings();
	void addDampeningAndGravityForces();

	Vec3 randVec3(float minDist, float maxDist, float allowNegative);

	float RandomFloat(float a, float b);
	int RandomInt(int a, int b);

private:
	// Attributes
	RigidBodySystem* m_pRigidBodySystem;
	Vec3 m_externalForce;
	int m_iIntegrator;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// extra 
// data
	vector<Spring> springs;
	float maxMass = 0;
	float gravity = 0;
	void calculateAndAddSpringForces();

	int numberRigidBodys;
	int numberSprings;
};