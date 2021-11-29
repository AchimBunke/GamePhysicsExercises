#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {

	Vec3 linearVelocity = Vec3();
	Vec3 angularVelocity = Vec3();
	Vec3 angularMomentum = Vec3();
	Vec3 torque = Vec3();
	Mat4 inverseInertiaTensor;
	Mat4 initInverseInertiaTensor;
	vector<tuple<Vec3,Vec3>> forces;
	Vec3 force;

	Quat orientation;
	Vec3 center;
	Mat4 scaleMat;
	Mat4 translatMat;
	Mat4 Obj2WorldMatrix;
	int mass;
	
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
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
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	vector<RigidBody> bodys;
	float clickForce;

	// Methods
	void clearForces();
	void calculateTorque();
	void integrateOrientation(float timeStep);
	void integrateAngularMomentum(float timeStep);
	void updateInverseInertiaTensor();
	void updateAngularVelocity();
	void calculateForces();
	void integrateVelocity(float timeStep);
	void integratePosition(float timeStep);
	Mat4 calculateInitInverseInertiaTensor(Vec3 size, float mass);

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif