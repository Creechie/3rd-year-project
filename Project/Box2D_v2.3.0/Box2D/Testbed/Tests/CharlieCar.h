#ifndef CHARLIE_CAR_H
#define CHARLIE_CAR_H

// Radians to degrees and vice-versa
#define DEGTORAD 0.01745329251994329576923690768489f
#define RADTODEG 57.295779513082320876798154814105f

#include <vector>
#include <set>

// Enumerations for keyboard presses
enum {
	LEFT  = 0x1,		// == 0001
	RIGHT = 0x2,		// == 0010
	UP	  = 0x4,		// == 0100
	DOWN  = 0x8			// == 1000
};


// Enumerations for types of fixture user data	(for collision detection)
enum fixtureUserDataType {
	FUD_CAR_TIRE = 0x1,
	FUD_WALL	 = 0x2
};

class FixtureUserData {
	fixtureUserDataType m_type;
protected:
	FixtureUserData(fixtureUserDataType type) : m_type(type) { }
public:
	virtual fixtureUserDataType getType() { return m_type; }
	virtual ~FixtureUserData() { }
};

// Allows a fixture to be marked as a car tire	(for collision detection)
class CarTireFUD : public FixtureUserData {
public:
	CarTireFUD() : FixtureUserData(FUD_CAR_TIRE) { }
};

// Allows a fixture to be marked as a wall		(for collision detection)
class WallFUD : public FixtureUserData {
public:
	bool hasCrashed;

	WallFUD(bool hc) : FixtureUserData(FUD_WALL) {
		hasCrashed = hc;
	}
};


class MyDestructionListener : public b2DestructionListener {

	void SayGoodbye(b2Fixture* fixture) {
		if (FixtureUserData* fud = (FixtureUserData*)fixture->GetUserData())
			delete fud;
	}

	void SayGoodbye(b2Joint* joint) { }
};

//bool b2Fixture::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex);

class CSensor {
public:


};

class CTire {
public:
	b2Body* m_body;

	float m_maxForwardSpeed  = 250.0f;
	float m_maxBackwardSpeed = -40.0f;
	float m_maxDriveForce    = 300.0f;

	std::set<WallFUD*> m_wall;
	bool m_hasCrashed;


	/// Constructor
	CTire(b2World* world) {
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		m_body = world->CreateBody(&bd);

		b2PolygonShape shape;
		shape.SetAsBox(0.5f, 1.25f);
		m_body->CreateFixture(&shape, 1.0f);

		m_body->SetUserData(this);

		m_hasCrashed = false;
	}

	/// Destructor
	~CTire() {
		m_body->GetWorld()->DestroyBody(m_body);
	}

	/// Gets the lateral (sideways) velocity of the tire
	b2Vec2 getLateralVelocity() {
		b2Vec2 currentRightNormal = m_body->GetWorldVector(b2Vec2(1, 0));
		return b2Dot(currentRightNormal, m_body->GetLinearVelocity()) * currentRightNormal;              
	}

	/// Gets the forwards velocity of the tire
	b2Vec2 getForwardVelocity() {
		b2Vec2 currentForwardNormal = m_body->GetWorldVector(b2Vec2(0, 1));
		return b2Dot(currentForwardNormal, m_body->GetLinearVelocity()) * currentForwardNormal;
	}

	/// Make the tire move like a real rolling tire would
	void updateFriction() {
		// Apply an impulse opposing the tire's lateral velocity 
		// This is to prevent the tire from moving sideways, aka skidding.
		b2Vec2 impulse = m_body->GetMass() * -getLateralVelocity();

		// This statement allows for /some/ skidding to occur. Higher maxLateralImpulse -> less skidding
		float maxLateralImpulse = 4.5f;
		if (impulse.Length() > maxLateralImpulse)
			impulse *= maxLateralImpulse / impulse.Length();

		m_body->ApplyLinearImpulse(impulse, m_body->GetWorldCenter(), true);

		// Do the same with the tire's angular velocity (not completely though, hence the '0.1f')
		m_body->ApplyAngularImpulse(0.1f * m_body->GetInertia() * -m_body->GetAngularVelocity(), true);

		// Apply a drag force to slow down the tire
		b2Vec2 currentForwardNormal = getForwardVelocity();
		float currentForwardSpeed = currentForwardNormal.Normalize();
		float dragMagnitude;
		if(!m_hasCrashed)
			dragMagnitude = -2 * currentForwardSpeed;
		else
			// If the tire has crashed - apply huge drag to stop the car
			dragMagnitude = -50 * currentForwardSpeed;

		m_body->ApplyForce(dragMagnitude * currentForwardNormal, m_body->GetWorldCenter(), false);

	} 

	void updateDrive(int keyboardState) {

		// Get desired speed based on key press
		float desiredSpeed = 0.0f;

		switch (keyboardState & (UP | DOWN)) {
		case UP:
			desiredSpeed = m_maxForwardSpeed;  break;
		case DOWN:
			desiredSpeed = m_maxBackwardSpeed; break;
		default:
			return;	// Do nothing
		}

		// Find current speed
		b2Vec2 currentForwardNormal = m_body->GetWorldVector(b2Vec2(0, 1));
		float currentSpeed = b2Dot(getForwardVelocity(), currentForwardNormal);

		// Apply necessary force
		float force = 0.0f;
		if (!m_hasCrashed)
			if (desiredSpeed > currentSpeed)
				force = m_maxDriveForce;
			else if (desiredSpeed < currentSpeed)
				force = -m_maxDriveForce;
			else
				return; // Do nothing (desired speed is met)
		

		m_body->ApplyForce(force * currentForwardNormal, m_body->GetWorldCenter(), true);
	}

	void updateTurn(int keyboardState) {

		float desiredTorque = 0;

		switch (keyboardState & (LEFT | RIGHT)) {
		case LEFT:
			desiredTorque = 15;  break;
		case RIGHT:
			desiredTorque = -15; break;
		default:
			return;	// Do nothing
		}
		m_body->ApplyTorque(desiredTorque, true);
	}


	void addWall(WallFUD* wall) { m_wall.insert(wall);   m_hasCrashed = true; }
	void removeWall(WallFUD* wall) { m_wall.erase(wall); m_hasCrashed = false; }
	

};


class CCar {
	b2Body* m_body;

	std::vector<CTire*> m_tires;
	b2RevoluteJoint *flJoint, *frJoint;
public:
	CCar(b2World* world) {
		// Create the car body
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		m_body = world->CreateBody(&bd);

		b2Vec2 vertices[8];
		vertices[0].Set(1.5, 0.0);
		vertices[1].Set(3.0, 2.5);
		vertices[2].Set(2.8, 5.5);
		vertices[3].Set(1.0, 10.0);
		vertices[4].Set(-1.0, 10.0);
		vertices[5].Set(-2.8, 5.5);
		vertices[6].Set(-3.0, 2.5);
		vertices[7].Set(-1.5, 0.0);

		b2PolygonShape ps;
		ps.Set(vertices, 8);
		b2Fixture* fixture = m_body->CreateFixture(&ps, 0.1f);

		// Set up joints for tires
		b2RevoluteJointDef jd;
		jd.bodyA = m_body;
		jd.enableLimit = true;
		// Make it so the joints won't move
		jd.lowerAngle = 0;
		jd.upperAngle = 0;
		// Anchor tire joints to the center of the tires
		jd.localAnchorB.SetZero();

		// Front left tire
		CTire* tire = new CTire(world);
		jd.bodyB = tire->m_body;
		jd.localAnchorA.Set(-3.0f, 8.5f);
		flJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
		m_tires.push_back(tire);

		// Front right tire
		tire = new CTire(world);
		jd.bodyB = tire->m_body;
		jd.localAnchorA.Set(3.0f, 8.5f);
		frJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
		m_tires.push_back(tire);

		// Back left tire
		tire = new CTire(world);
		jd.bodyB = tire->m_body;
		jd.localAnchorA.Set(-3.0f, 0.5f);
		world->CreateJoint(&jd);
		m_tires.push_back(tire);

		// Back right tire
		tire = new CTire(world);
		jd.bodyB = tire->m_body;
		jd.localAnchorA.Set(3.0f, 0.5f);
		world->CreateJoint(&jd);
		m_tires.push_back(tire);

	}

	void update(int keyboardState) {

		// Front and back movement control
		for (int i = 0; i < m_tires.size(); i++) {
			m_tires[i]->updateFriction();
			m_tires[i]->updateDrive(keyboardState);
		}

		/*for (int i = 0; i < m_tires.size(); i++)
			m_tires[i]->updateDrive(keyboardState);*/

		float lockAngle = 40 * DEGTORAD;
		float turnSpeedPerSec = 320 * DEGTORAD;
		float turnPerTimeStep = turnSpeedPerSec / 60.0f;
		float desiredAngle = 0.0f;

		switch (keyboardState & (LEFT | RIGHT)) {
		case LEFT : desiredAngle =  lockAngle; break;
		case RIGHT: desiredAngle = -lockAngle; break;
		default: break;	
		}

		float angleNow = flJoint->GetJointAngle();
		float angleToTurn = desiredAngle - angleNow;
		angleToTurn = b2Clamp(angleToTurn, -turnPerTimeStep, turnPerTimeStep);
		float newAngle = angleNow + angleToTurn;

		flJoint->SetLimits(newAngle, newAngle);
		frJoint->SetLimits(newAngle, newAngle);


	}
};


/// Test bed class
class CharlieCar : public Test {
	CTire *m_tire;
	CCar  *m_car;

	int m_keyboardState;

	MyDestructionListener m_destructionListener;

public:
	CharlieCar() {
		m_world->SetDestructionListener(&m_destructionListener);

		// Set the world gravity to zero (as the simultaion is top-down)
		m_world->SetGravity(b2Vec2(0, 0));

		m_keyboardState = 0;

		m_tire = new CTire(m_world);
		m_car  = new  CCar(m_world);


		b2BodyDef bodyDef;
		m_groundBody = m_world->CreateBody(&bodyDef);

		b2PolygonShape polygonShape;
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &polygonShape;
		fixtureDef.isSensor = true;

		polygonShape.SetAsBox(9, 7, b2Vec2(-10, 15), 20 * DEGTORAD);
		b2Fixture* wallFixture = m_groundBody->CreateFixture(&fixtureDef);
		wallFixture->SetUserData(new WallFUD(true));

		polygonShape.SetAsBox(9, 5, b2Vec2(5, 20), -40 * DEGTORAD);
		wallFixture = m_groundBody->CreateFixture(&fixtureDef);
		wallFixture->SetUserData(new WallFUD(true));

	}

	void Keyboard(unsigned char key) {
		switch (key) {
		case 'w': m_keyboardState |= UP;    break;
		case 'a': m_keyboardState |= LEFT;  break;
		case 's': m_keyboardState |= DOWN;  break;
		case 'd': m_keyboardState |= RIGHT; break;
		default : Test::Keyboard(key);
		}
	}

	void KeyboardUp(unsigned char key) {
		switch (key) {
		case 'w': m_keyboardState &= ~UP;    break;
		case 'a': m_keyboardState &= ~LEFT;  break;
		case 's': m_keyboardState &= ~DOWN;  break;
		case 'd': m_keyboardState &= ~RIGHT; break;
		default : Test::Keyboard(key);
		}
	}


	void handleContact(b2Contact* contact, bool began) {

		b2Fixture* a = contact->GetFixtureA();
		b2Fixture* b = contact->GetFixtureB();
		FixtureUserData* fudA = (FixtureUserData*)a->GetBody()->GetUserData();
		FixtureUserData* fudB = (FixtureUserData*)b->GetUserData();

		if (!fudA || !fudB)
			return;

		tire_vs_wall(a, b, began);

		/*if		(fudA->getType() == FUD_CAR_TIRE && fudB->getType() == FUD_WALL)
			tire_vs_wall(a, b, began);
		else if (fudA->getType() == FUD_WALL && fudB->getType() == FUD_CAR_TIRE)
			tire_vs_wall(b, a, began);*/

	}

	void BeginContact(b2Contact* contact) { handleContact(contact, true ); }
	void   EndContact(b2Contact* contact) { handleContact(contact, false); }

	void tire_vs_wall(b2Fixture* tireFixture, b2Fixture* wallFixture, bool began) {
		CTire* tire = (CTire*)tireFixture->GetBody()->GetUserData();
		WallFUD* wallFud = (WallFUD*)wallFixture->GetUserData();

		if (began)
			tire->addWall(wallFud);
		else
			tire->removeWall(wallFud);

	}


	void Step(Settings* settings) {

		m_car->update(m_keyboardState);
		
		//m_tire->updateFriction();
		//m_tire->updateDrive(m_keyboardState);
		//m_tire->updateTurn(m_keyboardState);

		if (m_tire->m_hasCrashed)
			m_debugDraw.DrawString(5, m_textLine, "You crashed - you're shit");

		// Run the default physics and rendering
		Test::Step(settings);

	}

	static Test* Create() {
		return new CharlieCar;
	}

	~CharlieCar() {
		delete m_tire;
		delete m_car;

		m_world->DestroyBody(m_groundBody);
	}
};


#endif

