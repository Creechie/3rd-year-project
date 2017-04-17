#ifndef FOOTEST_H
	#define FOOTEST_H
	
	/// Radians to degrees and vice-versa ///
	#define DEGTORAD 0.01745329251994329576923690768489f
	#define RADTODEG 57.295779513082320876798154814105f

	class FooTest : public Test {
		b2Body* dynamicBody;	// This must be defined as a class variable, instead of just in the constructor, as we use it in the Step() function

	public:
		FooTest() {

			b2Vec2 gravity(0.0f, -9.81f);
			m_world->SetGravity(gravity);

			// Create body
			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody;	// This will be a 'dynamic' body
			myBodyDef.position.Set(0, 20);		// Set the initial position
			myBodyDef.angle = 0;				// Set the initial angle

			// Send body to world
			dynamicBody = m_world -> CreateBody(&myBodyDef);

			// Create shape (for fixture)
			b2PolygonShape boxShape;
			boxShape.SetAsBox(1, 1);

			// Create fixture
			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;				// Assign shape to the fixture
			boxFixtureDef.density = 1;						// (Area of fixture * density = mass)
			dynamicBody -> CreateFixture(&boxFixtureDef);

			dynamicBody -> SetTransform( b2Vec2(10, 20), 45 * DEGTORAD );	// Move the box and rotate 45 degrees (counter-clockwise)
			dynamicBody -> SetLinearVelocity( b2Vec2(-5, 5) );				// Give the box a velocity
			dynamicBody -> SetAngularVelocity( -90 * DEGTORAD );			// Give the box an angular velocity


			// Create a 'static' body using the same body + fixture
			myBodyDef.type = b2_staticBody;
			myBodyDef.position.Set(0, 10);
			// Send to world
			b2Body* staticBody = m_world->CreateBody(&myBodyDef);
			// Add fixture to body
			staticBody->CreateFixture(&boxFixtureDef);

			// Create a 'kinematic' body
			myBodyDef.type = b2_kinematicBody;
			myBodyDef.position.Set(-18, 11);
			b2Body* kinematicBody = m_world->CreateBody(&myBodyDef);
			kinematicBody->CreateFixture(&boxFixtureDef);

			kinematicBody->SetLinearVelocity(b2Vec2(3, 0));				// Give the box a velocity
			kinematicBody->SetAngularVelocity(360 * DEGTORAD);			// Give the box an angular 



		}

		

		void Step(Settings* settings) {
			// Run the default physics and rendering
			Test::Step(settings);

			// Show some text in the main screen
			m_debugDraw.DrawString(5, m_textLine, "Now we have a Foo test");
			m_textLine += 15;

			// Get info on the dynamic box
			b2Vec2 pos = dynamicBody->GetPosition();
			float angle = dynamicBody->GetAngle();
			b2Vec2 vel = dynamicBody->GetLinearVelocity();
			float angularVel = dynamicBody->GetAngularVelocity();

			// Draw info on screen
			m_debugDraw.DrawString(5, m_textLine,
				"Position: %.2f,%.2f Angle: %.2f", pos.x, pos.y, angle * RADTODEG);
			m_textLine += 15;

			m_debugDraw.DrawString(5, m_textLine,
				"Velocity: %.2f,%.2f Angular velocity: %.2f", vel.x, vel.y, angularVel * RADTODEG);
			m_textLine += 15;



			// For each body in the world
			for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
				// Do something with body 'b'?
			}

		}

		static Test* Create() {
			return new FooTest;
		}
	};

#endif
