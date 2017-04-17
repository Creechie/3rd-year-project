#ifndef SENSOR_TUTORIAL_H
#define SENSOR_TUTORIAL_H

#include <vector>

class SensorTutorial : public Test {
	// Member variables go here (accessible to all class methods)
	bool gravityOn;

	b2Body* sensorBody;
	static const int e_count = 3;
	b2Body* boxes[e_count];
	bool boxTouching[e_count];

	b2Fixture* sensorFixture;

	std::vector<b2Body*> visibleBodies;

public:
	SensorTutorial() {
		// Gravity
		b2Vec2 gravity(0.0f, -9.81f);
		m_world->SetGravity(gravity);

		// Body definition
		b2BodyDef myBodyDef;

		
		

		//===============//
		// Sensor circle //
		//===============//
		{
			myBodyDef.position.Set(0, 20);

			// Shape definition
			b2CircleShape sensorShape;
			sensorShape.m_radius = 5;

			// Fixture definition
			b2FixtureDef myFixtureDef;
			myFixtureDef.shape = &sensorShape;
			myFixtureDef.isSensor = true;

			sensorBody = m_world->CreateBody(&myBodyDef);
			sensorBody->CreateFixture(&myFixtureDef);

		}
		//=============//
		// Ground edge //
		//=============//
		{
			myBodyDef.position.Set(0, 0);
			// Shape definition
			b2EdgeShape groundShape;
			groundShape.Set(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));

			b2Body* groundBody = m_world->CreateBody(&myBodyDef);
			groundBody->CreateFixture(&groundShape, 0.0);
		}
		//================//
		// Dynamic bodies //
		//================//
		{
			myBodyDef.type = b2_dynamicBody;
			// Shape definition
			b2PolygonShape boxShape;
			boxShape.SetAsBox(1, 1);

			for (int i = 0; i < e_count; i++) {
				boxTouching[i] = false;

				myBodyDef.position.Set(-15.0f + (15.0f * i), 30.0f);
				boxes[i] = m_world->CreateBody(&myBodyDef);
				boxes[i]->CreateFixture(&boxShape, 1.0);
			}
		}

	}

	// Implement contact listener.
	void BeginContact(b2Contact* contact) {

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == sensorBody->GetFixtureList()) {
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData) {
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}
		if (fixtureB == sensorBody->GetFixtureList()) {
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData) {
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}
	}




	// Tmplement contact listener
	void EndContact(b2Contact* contact) {

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == sensorBody->GetFixtureList()) {
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData) {
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}
		if (fixtureB == sensorBody->GetFixtureList()) {
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData) {
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}
	}

	void Step(Settings* settings) {
		// Run the default physics and rendering
		Test::Step(settings);

		for (int i = 0; i < e_count; i++) {

			if (!boxTouching[i]) continue;	// Skip this box if it isn't touching the sensor

			b2Body* box = boxes[i];
			b2Body* sensor = sensorBody;


			// Draw info on screen
			m_debugDraw.DrawString(5, m_textLine,
				"Box %i", i);
			m_textLine += 15;

			


		}



		
	}



	static Test* Create() {
		return new SensorTutorial;
	}
};


#endif
