#include "CoupledSimulator.h"
#define DebugRigidBody

#include <math.h>


CoupledSimulator::CoupledSimulator()
{
	m_iTestCase = 0;
	m_iIntegrator = 0;
	m_pRigidBodySystem = new RigidBodySystem();
}


const char* CoupledSimulator::getTestCasesStr()
{
	return "BasicTest,Setup1,Setup2";
}

void CoupledSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void CoupledSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void CoupledSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_pRigidBodySystem->m_rigidBodies.clear();
	springs.clear();
	/*m_pRigidBodySystem->SceneSetup(m_iTestCase);*/

	/*switch (m_iTestCase)
	{
	case 3:
			
		addSpring(0, 1, 1);
		addSpring(0, 5, 2);
		addSpring(3, 6, 4);

		addSpring(0, 7, 2);
		addSpring(7, 2, 1);
		addSpring(4, 5, 3);

		addSpring(6, 1, 6);
		addSpring(9, 8, 10);
		addSpring(8, 5, 1);
		addSpring(1, 7, 6);
		addSpring(10, 2, 20);
		addSpring(11, 10, 6);
		m_fStiffness = 10.0;
		break;
	default:
		break;

		cout << "Reset!" << endl;
	}*/

	switch (m_iTestCase) {
	case 1:

		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 1000.0f);
		addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);

		addRigidBody(Vec3(-1.4f, -1.6f, 1.7f), Vec3(0.3f, 0.22f, 0.52f), 40.0f);
		addRigidBody(Vec3(-2.3f, 3.52f, 3.0f), Vec3(0.34f, 0.82f, 0.22f), 35.0f);

		addRigidBody(Vec3(-6.1f, -4.2f, 20.1f), Vec3(2.4f, 2.25f, 3.2f), 4.0f);
		addRigidBody(Vec3(22.0f, 5.2f, 3.0f), Vec3(0.4f, 0.23f, 0.2f), 1.0f);

		addRigidBody(Vec3(-27.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(4.0f, 0.2f, 0.0f), Vec3(0.2f, 0.2f, 0.2f), 4.0f);

		addRigidBody(Vec3(-0.661f, -0.42f, 1.1f), Vec3(0.4f, 0.2f, 0.2f), 2.0f);
		addRigidBody(Vec3(0.40f, 0.332f, 0.0f), Vec3(0.43f, 0.2f, 0.2f), 25.0f);

		addRigidBody(Vec3(-10.1f, -0.9f, 0.31f), Vec3(0.34f, 0.26f, 0.2f), 160.0f);
		addRigidBody(Vec3(0.9f, 0.9f, 0.20f), Vec3(0.4f, 0.2f, 0.26f), 200.0f);

		addRigidBody(Vec3(-53.1f, -3.2f, 0.1f), Vec3(0.33f, 0.2f, 0.2f), 10.0f);
		addRigidBody(Vec3(3.35f, 1.332f, 5.0f), Vec3(0.4f, 0.2f, 0.5f), 1.0f);

		addRigidBody(Vec3(-50.1f, -20.2f, -0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(0.50f, -0.42f, -3.0f), Vec3(0.4f, 0.2f, 0.2f), 30.0f);

		addRigidBody(Vec3(50.1f, 50.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 13.0f);
		addRigidBody(Vec3(10.0f, 0.52f, 0.0f), Vec3(0.34f, 0.2f, 0.2f), 4.7f);

		addRigidBody(Vec3(10.1f, -10.2f, -10.1f), Vec3(2.f, 2.2f, 2.2f), 13.0f);
		addRigidBody(Vec3(-10.0f, 6.52f, 5.0f), Vec3(3.34f, 1.2f, 2.2f), 40.7f);

		addRigidBody(Vec3(-23.1f, 13.2f, 10.1f), Vec3(1.6f, 2.2f, 1.2f), 101.0f);
		addRigidBody(Vec3(13.35f, 11.332f, 15.0f), Vec3(2.4f, 3.2f, 2.5f), 12.0f);

		addRigidBody(Vec3(-10.1f, -40.2f, 20.1f), Vec3(0.55f, 0.52f, 4.2f), 300.0f);
		addRigidBody(Vec3(1.50f, -5.42f, -8.0f), Vec3(0.756f, 0.25f, 4.2f), 1.0f);

		addRigidBody(Vec3(3.1f, 3.2f, 8.1f), Vec3(0.33f, 0.2f, 0.32f), 1.0f);
		addRigidBody(Vec3(-100.0f, -14.52f, 23.0f), Vec3(10.34f, 10.2f, 7.2f), 4.7f);

		addRigidBody(Vec3(18.1f, -4.2f, -7.1f), Vec3(2.0f, 2.23f, 2.2f), 5.0f);
		addRigidBody(Vec3(-5.0f, -4.52f, 1.6f), Vec3(3.34f, 1.32f, 2.2f), 14.7f);


		setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));

		addSpring(0, 1, 5, 100);
		addSpring(1, 2, 10, 10);
		addSpring(6, 3, 20, 150);
		addSpring(0, 1, 2, 30);
		addSpring(15, 1, 14, 14);
		addSpring(12, 15, 66, 10);
		addSpring(10, 17, 13, 90);
		addSpring(9, 17, 4, 3);
		addSpring(10, 5, 5, 72);
		addSpring(2, 4, 7, 47);
		addSpring(0, 16, 10, 28);
		addSpring(13, 11, 7, 43);

		addSpring(7, 8, 6, 5);
		addSpring(8, 0, 30, 230);

		addSpring(19, 18, 6, 5);
		addSpring(19, 0, 30, 230);

		addSpring(4, 18, 40, 100);
		addSpring(8, 19, 50, 79);


		addSpring(26, 27, 12, 12);
		addSpring(0, 26, 5, 45);


		addSpring(25, 23, 10, 322);
		addSpring(25, 24, 16, 5);


		addSpring(25, 16, 6, 23);
		addSpring(22, 21, 8, 67);


		addSpring(20, 19, 12, 22);
		addSpring(20, 21, 23, 33);

		addSpring(25, 16, 6, 23);
		addSpring(22, 21, 8, 67);


		addSpring(20, 15, 30, 65);
		addSpring(23, 8, 100, 30);
		break;
	case 2:
		int rb = 50;
		int springs = 50;
		float maxDist = 100;
		float maxSize = 5.0f;
		float minSize = 0.1;
		float maxMass = 300.0f;
		float maxSpringDist = 100.0f;
		float maxStiff = 300.0f;

		addRigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), 1000);

		for (int i = 0; i < rb; i++) {
			float s1 = RandomFloat(minSize, maxSize);
			addRigidBody(randVec3(0-maxDist/2, maxDist/2,true), randVec3(max(0.0f,(s1-s1/3)),s1,false), RandomFloat(1,maxMass));
		}
		for (int i = 0; i < springs; i++) {
			int a = RandomInt(0, rb );
			int b = RandomInt(0, rb );
			while (b == a) {
				b = RandomInt(0, rb );
			}
			addSpring(a, b, RandomFloat(1, maxSpringDist), RandomFloat(1, maxStiff));
		}

		break;
	}

}

Vec3 CoupledSimulator::randVec3(float minDist, float maxDist,float allowNegative = true) {
	float x, y, z;
	if (allowNegative) {
		x = RandomFloat(-1, 1);
		y = RandomFloat(-1, 1);
		z = RandomFloat(-1, 1);
	}
	else {
		x = RandomFloat(0, 1);
		y = RandomFloat(0, 1);
		z = RandomFloat(0, 1);
	}
	float dist = RandomFloat(minDist, maxDist);
	Vec3 v = Vec3(x, y, z);
	normalize(v);
	v *= dist;
	return v;
}

float CoupledSimulator::RandomFloat(float a, float b) {
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}
int CoupledSimulator::RandomInt(int a, int b) {
	int random = rand() % (b-a);
	return a + random;
}


void CoupledSimulator::externalForcesCalculations(float elapsedTime)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.2f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;

	m_externalForce = pullforce;

}

void CoupledSimulator::simulateTimestep(float timeStep)
{
	// one fixed time step
	switch (m_iTestCase) {
	default:

			
			calculateAndAddSpringForces();			
			addDampeningAndGravityForces();
			break;
	}

	switch (m_iTestCase)
	{
	case 0: // case 0 do nothing
		break;
	case 1:
	{
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);
	}
	break;
	case 2:
	{
		if (DXUTIsKeyDown(VK_LBUTTON))
			m_pRigidBodySystem->dragTogether();
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);

	}
	break;
	default:
		break;
	}
	
}

void CoupledSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 colors[3] = { Vec3(0.9,0.97,1), Vec3(0.5,0.5,1), Vec3(1,1,0) };
	int i = 0;
	for (RigidBody& rigidBody : m_pRigidBodySystem->m_rigidBodies)
	{
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, colors[i % 3]);
		DUC->drawRigidBody(rigidBody.getObj2World());
		++i;
#ifdef DebugRigidBody
		std::mt19937 eng;
		std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
		std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		//DUC->drawSphere(rigidBody.collisonPoint,Vec3(0.05f, 0.05f, 0.05f));
		DUC->beginLine();
		Vec3 velocity = rigidBody.totalVelocity;
		DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(), Colors::DeepPink, rigidBody.collisonPoint.toDirectXVector() + rigidBody.relVelocity.toDirectXVector(), Colors::DeepPink);

		if (i == 1) {
			DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(), Colors::DarkGreen, rigidBody.collisonPoint.toDirectXVector() + velocity.toDirectXVector(), Colors::DarkGreen);
			DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(), Colors::Yellow, rigidBody.collisonPoint.toDirectXVector() + rigidBody.collisioNormal.toDirectXVector(), Colors::Orange);
		}
		else {
			DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(), Colors::DarkRed, rigidBody.collisonPoint.toDirectXVector() + velocity.toDirectXVector(), Colors::DarkRed);
			DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(), Colors::DarkCyan, rigidBody.collisonPoint.toDirectXVector() + rigidBody.collisioNormal.toDirectXVector(), Colors::DarkViolet);

		}
		DUC->endLine();

		/*
					std::vector<XMVECTOR> corners = rigidBody.getCorners();
					Vec3 colors[8] = {Vec3(0,0,0),Vec3(0,0,1),Vec3(0,1,0),Vec3(0,1,1),Vec3(1,0,0),Vec3(1,0,1),Vec3(1,1,0),Vec3(1,1,1)};
					for(int j = 0; j< 8;j++)
					{
						DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,colors[j]);
						DUC->drawSphere(corners[j],Vec3(0.02f, 0.02f, 0.02f));

					}
					*/
#endif
	}	
	for (int i = 0; i < getNumberOfSprings(); i++) {
		float m1 = springs[i].p1->getMass();
		float m2 = springs[i].p2->getMass();
		/*float red1 = min(1.0f, m1 * 2 / maxMass);
		float green1 = min(1.0f, 1.0f - ((2 * m1) - maxMass) / maxMass);
		float red2 = min(1.0f, m2 * 2 / maxMass);
		float green2 = min(1.0f, 1.0f - ((2 * m2) - maxMass) / maxMass);*/
		float dif = sqrt(springs[i].p1->getCenter().squaredDistanceTo(springs[i].p2->getCenter())) - springs[i].initialLength;
		float red = min(1.0f, (float)pow(dif / 2, 2.0));
		float green = min(1.0f, (float)-pow(dif / 2, 2.0) + 1);

		DUC->beginLine();
		DUC->drawLine(springs[i].p1->getCenter(), Vec3(red, green, 0), springs[i].p2->getCenter(), Vec3(red, green, 0));
		DUC->endLine();
	}
}

int CoupledSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->m_rigidBodies.size();
}

Vec3 CoupledSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getCenter();
}

Vec3 CoupledSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getVelocity();
}

Vec3 CoupledSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getAngularV();
}

void CoupledSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_rigidBodies[i].addForceWorld(force, loc);
}

void CoupledSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->m_rigidBodies.emplace_back(position, size, mass);
	m_pRigidBodySystem->m_rigidBodies.back().update(0.0f);

}

void CoupledSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->m_rigidBodies[i].setRotation(orientation);

}

void CoupledSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->m_rigidBodies[i].setVelocity(velocity);
}

void CoupledSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void CoupledSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void CoupledSimulator::addSpring(int rb1, int rb2, float initialLength,float stiffness) {
	Spring s;
	s.p1 = &m_pRigidBodySystem->m_rigidBodies[rb1];
	s.p2 = &m_pRigidBodySystem->m_rigidBodies[rb2];
	s.initialLength = initialLength;
	s.stiffness = stiffness;
	springs.push_back(s);
}

int CoupledSimulator::getNumberOfSprings() {
	return springs.size();
}


void CoupledSimulator::calculateAndAddSpringForces() {
	for (int i = 0; i < getNumberOfSprings(); i++)
	{
		Vec3 dir = springs[i].p1->getCenter() - springs[i].p2->getCenter();
		float dist = sqrt(springs[i].p1->getCenter().squaredDistanceTo(springs[i].p2->getCenter()));
		Vec3 dir_norm = dir / dist;
		Vec3 force = -springs[i].stiffness * (dist - springs[i].initialLength) * dir_norm;
		springs[i].p1->addForceWorld(force / springs[i].p1->getMass(),springs[i].p1->getCenter());
		springs[i].p2->addForceWorld( -force / springs[i].p2->getMass(), springs[i].p2->getCenter());
	}
}


void CoupledSimulator::addDampeningAndGravityForces() {
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		//m_pRigidBodySystem->m_rigidBodies[i].addForceWorld(Vec3(0, -gravity, 0) / m_pRigidBodySystem->m_rigidBodies[i].getMass(), m_pRigidBodySystem->m_rigidBodies[i].getCenter());
	}
}