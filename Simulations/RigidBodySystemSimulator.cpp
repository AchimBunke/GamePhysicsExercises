#include "RigidBodySystemSimulator.h"
#include <vector>
#include <math.h>

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_mouse = {};
	m_trackmouse = {};
	m_oldtrackmouse = {};
}

// Functions
const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "DEMO_1,DEMO_2,DEMO_3,DEMO_4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	reset();
}
void RigidBodySystemSimulator::reset()
{	
	bodys.clear();
	switch (m_iTestCase)
	{
	case 0:
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), 90));
		applyForceOnBody(0, Vec3(1, 1, 0), Vec3(0.3, 0.5, 0.25));

		break;
	case 1:
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), 90));
		applyForceOnBody(0, Vec3(1, 1, 0), Vec3(0.3, 0.5, 0.25));

		TwAddVarRW(DUC->g_pTweakBar, "Rotation", TW_TYPE_FLOAT, &clickForce, "");

		break;
	}
	
}


void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		RigidBody b = bodys[i];
		Mat4 rotMat = b.orientation.getRotMat();
		bodys[i].Obj2WorldMatrix = b.scaleMat * b.orientation.getRotMat() * b.translatMat;
		DUC->drawRigidBody(bodys[i].Obj2WorldMatrix);
	}

}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	cout << "Switch to testCase: " << m_iTestCase << endl;
	reset();
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		bodys[i].linearVelocity += m_externalForce * timeElapsed;
	}
	m_externalForce = Vec3();
}
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	calculateTorque();
	calculateForces();
	integratePosition(timeStep);
	integrateVelocity(timeStep);

	integrateOrientation(timeStep);
	integrateAngularMomentum(timeStep);
	updateInverseInertiaTensor();
	updateAngularVelocity();
	
	clearForces();
}

void RigidBodySystemSimulator::onClick(int x, int y) {

}
void RigidBodySystemSimulator::onMouse(int x, int y) {
	
}

// ExtraFunctions

int RigidBodySystemSimulator::getNumberOfRigidBodies() { return bodys.size(); }

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return bodys[i].translatMat.transformVector(bodys[i].center); }

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { return bodys[i].linearVelocity; }

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { return bodys[i].angularVelocity; }

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	bodys[i].forces.push_back({ loc,force });
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody b = RigidBody();
	b.center = Vec3();
	b.scaleMat = Mat4(size.x, 0, 0, 0,
		0, size.y, 0, 0,
		0, 0, size.z, 0,
		0, 0, 0, 1);
	b.translatMat = Mat4(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		position.x, position.y, position.z, 1);
	b.orientation = Quat();
	b.initInverseInertiaTensor = calculateInitInverseInertiaTensor(size, mass);
	b.mass = mass;
	bodys.push_back(b);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	bodys[i].orientation = orientation;
	bodys[i].orientation /= bodys[i].orientation.norm();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	bodys[i].linearVelocity = velocity;
}

void RigidBodySystemSimulator::clearForces()
{
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		bodys[i].forces.clear();
		bodys[i].torque = Vec3();
		bodys[i].force = Vec3();
	}
}

void RigidBodySystemSimulator::calculateTorque() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		bodys[i].torque = Vec3();
		Mat4 trans = Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -bodys[i].translatMat.value[3][0], -bodys[i].translatMat.value[3][1], -bodys[i].translatMat.value[3][2], 1);
		for (int z = 0; z < bodys[i].forces.size() ;z++) {
			bodys[i].torque += cross(trans.transformVector((Vec3)std::get<0>(bodys[i].forces[z])), (Vec3)std::get<1>(bodys[i].forces[z]));
		}
	}
}
void RigidBodySystemSimulator::integrateOrientation(float timeStep) {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		RigidBody body = bodys[i];
		Quat rot = Quat(body.angularVelocity, 1);
		bodys[i].orientation += Quat(((timeStep / 2.0) * rot) *  body.orientation);
		if (bodys[i].orientation.norm() != 0) {
			bodys[i].orientation /= bodys[i].orientation.norm();
		}
	}
}
void RigidBodySystemSimulator::integrateAngularMomentum(float timeStep) {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		RigidBody body = bodys[i];
		bodys[i].angularMomentum += timeStep * body.torque;
	}
}
void RigidBodySystemSimulator::updateInverseInertiaTensor() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		RigidBody body = bodys[i];
		Mat4 transposed = Mat4(body.orientation.getRotMat());
		transposed.transpose();
		bodys[i].inverseInertiaTensor =  body.orientation.getRotMat() * body.initInverseInertiaTensor *  transposed;
	}
}
void RigidBodySystemSimulator::updateAngularVelocity() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		RigidBody body = bodys[i];
		bodys[i].angularVelocity = bodys[i].inverseInertiaTensor.transformVector(bodys[i].angularMomentum);
	}
}

void RigidBodySystemSimulator::calculateForces() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		for (int z = 0; z < bodys[i].forces.size(); z++) {
			bodys[i].force += (Vec3)std::get<1>(bodys[i].forces[z]) / bodys[i].mass;
		}
	}
}
void RigidBodySystemSimulator::integrateVelocity(float timeStep) {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
			bodys[i].linearVelocity += bodys[i].force * timeStep;	
	}

}
void RigidBodySystemSimulator::integratePosition(float timeStep) {
	for (int i = 0; i < getNumberOfRigidBodies(); i++)
	{
		Vec3 v = bodys[i].linearVelocity * timeStep;
		bodys[i].translatMat += Mat4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, v.x, v.y, v.z, 0);
	}
}

Mat4 RigidBodySystemSimulator::calculateInitInverseInertiaTensor(Vec3 size, float mass) 
{
	
	Mat4 c = Mat4(((size.x / 2) * (size.x / 2))*mass , 0, 0, 0,
			   	  0, ((size.y / 2) * (size.y / 2)) * mass , 0, 0,
				  0, 0, ((size.z / 2) * (size.z / 2)) *mass, 0,
				  0, 0, 0, 0);
	float trace = c.value[0][0] + c.value[1][1]+c.value[2][2];
	Mat4 i = Mat4(trace, 0, 0, 0,
				  0, trace, 0, 0,
				  0, 0, trace, 0,
				  0, 0, 0, 1);
	return (i - c).inverse();
}
