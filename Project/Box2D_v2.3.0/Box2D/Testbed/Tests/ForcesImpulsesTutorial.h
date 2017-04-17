#ifndef FORCES_IMPULSES_TUTORIAL_H
#define FORCES_IMPULSES_TUTORIAL_H

class ForcesImpulsesTutorial : public Test {
	// Member variables go here (accessible to all class methods)
	b2Body* bodies[3];
	bool forceOn, torqueOn, gravityOn;

public:
	ForcesImpulsesTutorial() {
		forceOn  = false;
		torqueOn = false;


		// Body definition
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		// Shape definition
		b2PolygonShape polygonShape;
		polygonShape.SetAsBox(1, 1);

		// Fixture definition
		b2FixtureDef myFixtureDef;
		myFixtureDef.shape	 = &polygonShape;
		myFixtureDef.density = 1;

		// Create identical bodies in different positions
		for (int i = 0; i < 3; i++) {
			myBodyDef.position.Set(-10.0f + (10.0f * i), 20.0f);
			bodies[i] = m_world->CreateBody(&myBodyDef);
			bodies[i]->CreateFixture(&myFixtureDef);
		}

		// Create a floor
		myBodyDef.type = b2_staticBody;
		myBodyDef.position.Set(0, 0);
		b2Body* floor = m_world->CreateBody(&myBodyDef);
		b2EdgeShape floorShape;
		floorShape.Set(b2Vec2(-15.0, 0.0), b2Vec2(15.0, 0.0));
		floor->CreateFixture(&floorShape, 0.0);
	}

	void Step(Settings* settings) {
		// Run the default physics and rendering
		Test::Step(settings);

		if (forceOn)	  // Apply gradual force upwards
			//bodies[0]->ApplyForceToCenter(b2Vec2(0, 50), true);
			bodies[0]->ApplyForce(b2Vec2(0, 50), bodies[0]->GetWorldPoint(b2Vec2(1,1)), true);

		if (torqueOn)	  // Apply gradual torque counter-clockwise
			bodies[0]->ApplyTorque(20, true);

		if (!gravityOn) { // Apply force to cancel out gravity
			bodies[0]->ApplyForceToCenter(bodies[0]->GetMass() * -m_world->GetGravity(), false);
			bodies[1]->ApplyForceToCenter(bodies[1]->GetMass() * -m_world->GetGravity(), false);
		}
	}

	void Keyboard(unsigned char key) {
		switch (key) {
		case 'q':
			// Apply gradual force upwards
			forceOn = !forceOn; // Toggle
			break;
		case 'w':
			// Apply immediate force upwards
			//bodies[1]->ApplyLinearImpulse(b2Vec2(0, 50), bodies[1]->GetWorldCenter(), true);
			bodies[1]->ApplyLinearImpulse(b2Vec2(0, 50), bodies[1]->GetWorldPoint(b2Vec2(1, 1)), true);
			break;
		case 'e':
			// Teleport to new location
			bodies[2]->SetTransform(b2Vec2(10, 20), 0);
			break;
		case 'a':
			// Apply gradual torque counter-clockwise
			torqueOn = !torqueOn; // Toggle
			break;
		case 's':
			// Apply immediate spin counter-clockwise
			bodies[1]->ApplyAngularImpulse(20, true);
			break;
		case 'g':
			// Toggle gravity for bodies 0 and 1
			gravityOn = !gravityOn; // Toggle

		default:
			// Run default behaviour
			Test::Keyboard(key);
		}
	}

	static Test* Create() {
		return new ForcesImpulsesTutorial;
	}
};

#endif

