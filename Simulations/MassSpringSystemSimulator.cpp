#include "MassSpringSystemSimulator.h"

#include <vector>
#include <math.h>

#define DEMO_1 0
#define DEMO_2 1
#define DEMO_3 2
#define DEMO_4 3





int demo = 0;



MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = demo;
	points = vector<Point>();
	springs = vector<Spring>();

	reset();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "DEMO_1,DEMO_2,DEMO_3,DEMO_4";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	points.clear();
	springs.clear();

	switch (m_iTestCase)
	{
	case 0:
		
		break;
	case 1:
		m_fMass = 10;
		m_fStiffness = 40.0;
		m_fDamping = 0.0;
		gravity = 0.0;
		m_iIntegrator = EULER;
		addSpring(
			addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
			addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
			1);
		break;
	case 2:
		m_fMass = 10;
		m_fStiffness = 40.0;
		m_fDamping = 0.0;
		gravity = 0.0;
		m_iIntegrator = MIDPOINT;
		addSpring(
			addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
			addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
			1);
		break;
	case 3:
		m_fMass = 10;
		m_fStiffness = 10.0;
		m_fDamping = 0.0;
		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(0, 2, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(5, 2, 0), Vec3(0, 0, 0), false);
		m_fMass = 30;
		addMassPoint(Vec3(1, 1, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, 0, 4), Vec3(0, 0.4, 0), true);
		addMassPoint(Vec3(2, 2, 2), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(5, 8, 1), Vec3(0, 0, -1), false);
		m_fMass = 15;
		addMassPoint(Vec3(1, 4, 1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(2, -14, 1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(20, 0, -20), Vec3(0, 0, 0), false);
		m_fMass = 50;
		addMassPoint(Vec3(-5, -5, 5), Vec3(0, 5, 0), false);
		m_fMass = 100;
		addMassPoint(Vec3(1, -10, -3), Vec3(0, 2, 2), false);
		m_fMass = 10;

		addSpring(0, 1, 1);
		addSpring(0, 5, 2);
		addSpring(3, 6, 4);
		m_fStiffness = 15.0;
		addSpring(0, 7, 2);
		addSpring(7, 2, 1);
		addSpring(4, 5, 3);
		m_fStiffness = 13.0;
		addSpring(6, 1, 6);
		addSpring(9, 8, 10);
		addSpring(8, 5, 1);
		m_fStiffness = 5.0;
		addSpring(1, 7, 6);
		addSpring(10, 2, 20);
		addSpring(11, 10, 6);
		m_fStiffness = 10.0;
		break;
	default:
		break;

		cout << "Reset!" << endl;
	}
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		points.clear();
		springs.clear();
		m_fMass = 10;
		m_fStiffness = 40.0;
		m_fDamping = 0.0;
		m_iIntegrator = EULER;
		gravity = 0.0;
		addSpring(
			addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
			addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
			1);
		simulateTimestep(0.1f);
		std::cout << "Euler: \n";
		std::cout << "Position of Point 1: " << getPositionOfMassPoint(0) << "\n";
		std::cout << "Velocity of Point 1: " << getVelocityOfMassPoint(0) << "\n\n";
		std::cout << "Position of Point 2: " << getPositionOfMassPoint(1) << "\n";
		std::cout << "Velocity of Point 2: " << getVelocityOfMassPoint(1) << "\n\n";
		points.clear();
		springs.clear();
		m_fMass = 10;
		m_fStiffness = 40.0;
		m_fDamping = 0.0;
		m_iIntegrator = MIDPOINT;
		gravity = 0.0;
		addSpring(
			addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
			addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
			1);
		simulateTimestep(0.1f);
		std::cout << "Midpoint: \n";
		std::cout << "Position of Point 1: " << getPositionOfMassPoint(0) << "\n";
		std::cout << "Velocity of Point 1: " << getVelocityOfMassPoint(0) << "\n\n";
		std::cout << "Position of Point 2: " << getPositionOfMassPoint(1) << "\n";
		std::cout << "Velocity of Point 2: " << getVelocityOfMassPoint(1) << "\n\n";

		break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INT8, &m_iIntegrator, "min=0 max=2");
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	cout << "Switch to testCase: " << m_iTestCase << endl;
	reset();
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		float m = points[i].mass;
		float rad =  log10(m) * 0.1;
		float red = min(1.0f,m * 2 / maxMass);
		float green = min(1.0f, 1.0f - ((2 * m) - maxMass ) / maxMass);
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(red,green, 0.0));
		DUC->drawSphere(points[i].position, Vec3(rad, rad, rad));
	}
	for (int i = 0; i < getNumberOfSprings(); i++) {
		float m1 = springs[i].p1->mass;
		float m2 = springs[i].p2->mass;
		/*float red1 = min(1.0f, m1 * 2 / maxMass);
		float green1 = min(1.0f, 1.0f - ((2 * m1) - maxMass) / maxMass);
		float red2 = min(1.0f, m2 * 2 / maxMass);
		float green2 = min(1.0f, 1.0f - ((2 * m2) - maxMass) / maxMass);*/
		float dif = sqrt(springs[i].p1->position.squaredDistanceTo(springs[i].p2->position)) - springs[i].initialLength;
		float red = min(1.0f, (float)pow(dif / 2, 2.0) );
		float green = min(1.0f, (float)-pow(dif / 2, 2.0)+1);

		DUC->beginLine();
		DUC->drawLine(springs[i].p1->position, Vec3(red, green,0),springs[i].p2->position,Vec3(red, green,0));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}


void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase) {
	case 0:
		break;
	default:
		// update current setup for each frame
		switch (m_iIntegrator)
		{// handling different cases
		case EULER:
		{

			// clearForces
			clearForces();
			// calculateAndAddSpringForces
			calculateAndAddSpringForces();
			// dampening
			addDampeningAndGravityForces();
			// integratePositions
			integratePositions(timeStep);
			// integrateVelocity
			integrateVelocity(timeStep);

		}
		break;
		case MIDPOINT:
		{
			// clear
			clearForces();
			// midstep
			vector<Point> oldPoints = vector<Point>(points);
			// calculateAndAddSpringForces
			calculateAndAddSpringForces();
			// dampening
			addDampeningAndGravityForces();
			// integratePositions
			integratePositions(timeStep / 2.0);
			// integrateVelocity
			integrateVelocity(timeStep / 2.0);

			// at midstep
			clearForces();
			// calculate midstep forces
			calculateAndAddSpringForces();
			// dampening add midstep
			addDampeningAndGravityForces();

			// reset to old position
			for (int i = 0; i < getNumberOfMassPoints(); i++) {
				points[i].position = oldPoints[i].position;
			}
			// integrate old positions with midstep velocity
			integratePositions(timeStep);
			// reset to old velocity
			for (int i = 0; i < getNumberOfMassPoints(); i++) {
				points[i].velocity = oldPoints[i].velocity;
			}
			// integrate old velocity with midstep forces
			integrateVelocity(timeStep);
		}
		break;
		default:
			break;
		}
		/*// print
		int c = 0;
		for (int i = 0; i < getNumberOfMassPoints(); i++) {
			cout << "P" << c << " - Pos: " << points[i].position << "   Vel: " << points[i].velocity << endl;
			c++;
		}*/
	}
}



int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	Point p;
	p.position = position;
	p.velocity = Velocity;
	p.mass = m_fMass;
	p.damping = m_fDamping;
	p.isFixed = isFixed;
	points.push_back(p);
	maxMass = max(maxMass, p.mass);
	return points.size()-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring s;
	s.p1 = &points[masspoint1];
	s.p2 = &points[masspoint2];
	s.initialLength = initialLength;
	s.stiffness = m_fStiffness;
	springs.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}
 
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return points[index].position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return points[index].velocity;
}


void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float dampingFactor) {
	m_fDamping = dampingFactor;
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::clearForces() {
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		points[i].force *= 0;
	}
}

void MassSpringSystemSimulator::calculateAndAddSpringForces() {
	for (int i = 0; i < getNumberOfSprings(); i++)
	{
		Vec3 dir = springs[i].p1->position - springs[i].p2->position;
		float dist = sqrt(springs[i].p1->position.squaredDistanceTo(springs[i].p2->position));
		Vec3 dir_norm = dir / dist;
		Vec3 force = -springs[i].stiffness * (dist - springs[i].initialLength) * dir_norm;
		springs[i].p1->force += force / springs[i].p1->mass;
		springs[i].p2->force += -force / springs[i].p2->mass;
	}
}


void MassSpringSystemSimulator::addDampeningAndGravityForces() {
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		points[i].force += Vec3(0,-gravity, 0) / points[i].mass;
		points[i].force += -points[i].velocity * points[i].damping;
	}
}

void MassSpringSystemSimulator::integratePositions(float timeStep) {
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (!points[i].isFixed) {
			points[i].position += timeStep * points[i].velocity;
		}
	}
}
void MassSpringSystemSimulator::integrateVelocity(float timeStep) {
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (!points[i].isFixed) {
			points[i].velocity += timeStep * points[i].force;
		}
	}
}