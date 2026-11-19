#include "contacts.h"
#include <cassert>

using namespace cyclone;

// contact stuff:

// swaps the two rigidbodies and updates the normal to match
void Contact::swapBodies() {
	mNormal *= -1.f;
	Rigidbody* temp = mBodies[0];
	mBodies[0] = mBodies[1];
	mBodies[1] = temp;
}

//  calculates the internals 
void Contact::calculateInternals(real deltaTime) {
	// swap the rigidbodies so there will always be something in the first slot if there is one body
	if (!mBodies[0]) swapBodies();
	// we assume at least one of the collisions will have a rigidbody if they collided
	assert(mBodies[0]);

	// calculate a set of axes at the contact point
	calculateContactBasis();

	// store the relative position of the contact relative to each body
	relativeContactPositions[0] = mPoint - mBodies[0]->getPos();
	if (mBodies[1]) {
		relativeContactPositions[1] = mPoint - mBodies[1]->getPos();
	}

	// find and store the relative velocities of the bodies at the contact point
	contactVelocity = calculateLocalVelocity(0, deltaTime);
	if (mBodies[1])
		contactVelocity -= calculateLocalVelocity(1, deltaTime);

	// calculate the desired change in velocity for the resolution
	calculateDesiredDeltaVelocity(deltaTime);
}

// finds and sets the internal value for the desired delta velocities
void Contact::calculateDesiredDeltaVelocity(real deltaTime) {
	const static real velocityLimit = (real)0.25f;

	// calculate the acceleration induced velocity accumulated from this frame
	real velFromAcc = 0;

	// if we ever do the get awake stuff, we add an if here
	velFromAcc += mBodies[0]->getLastFrameAcceleration() * deltaTime * mNormal;
	if (mBodies[1]) velFromAcc -= mBodies[1]->getLastFrameAcceleration() * deltaTime * mNormal;

	// if the velocity is slow, limit the restitution
	real thisRestitution = mCoeffRest;
	if (real_abs(contactVelocity.x) < velocityLimit) thisRestitution = (real)0.f;

	// combine the bounce vel with the removed accel vel
	desiredDeltaVel = -contactVelocity.x - thisRestitution * (contactVelocity.x - velFromAcc);
}

Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real deltaTime) {
	Rigidbody* thisBody = mBodies[bodyIndex];

	Vector3 vel = thisBody->getAngularVelocity() % relativeContactPositions[bodyIndex];
	vel += thisBody->getLinearVelocity();

	// turn the velocity into contact coords
	Vector3 contactVel = contactToWorld.transformTranspose(vel);

	// calculate the amount of velocity that is due to forces without rections
	Vector3 accVelocity = thisBody->getLastFrameAcceleration() * deltaTime;

	// calculate the velocity in the contact coordinates
	accVelocity = contactToWorld.transformTranspose(accVelocity);

	// ignore any component of acceleration in the contact normal direction, since we only want planar acceleration
	accVelocity.x = 0;
	
	// add planar velocities, and if there is enough friction they will be removed during the resolution
	// though we probably aren't calculating friction in space lmao
	contactVel += accVelocity;

	// return it
	return contactVel;
}

// makes an arbitrary orthonormal basis for the contact. This is stored as a 3x3 matrix, where each vector is a column. 
void Contact::calculateContactBasis() {
	Vector3 contactTangent[2];

	// check whether the Z-axis is nearer to the X or Y axis. 
	if (real_abs(mNormal.x) > real_abs(mNormal.y)) {
		// scaling factor to make the results normalized
		const real scalingFactor = (real)1.f / real_sqrt(mNormal.z * mNormal.z + mNormal.x * mNormal.x);
		
		// The new X axis is at right angles to the world Y axis
		contactTangent[0].x = mNormal.z * scalingFactor;
		contactTangent[0].y = 0;
		contactTangent[0].z = -mNormal.x * scalingFactor;

		// the new Y axis is at right angles to the new x and z axes
		contactTangent[1].x = mNormal.y * contactTangent[0].x;
		contactTangent[1].y = mNormal.z * contactTangent[0].x - mNormal.x * contactTangent[0].z;
		contactTangent[1].z = -mNormal.y * contactTangent[0].x;
	}
	else {
		// scaling factor to make the results normalized
		const real scalingFactor = (real)1.f / real_sqrt(mNormal.z * mNormal.z + mNormal.y * mNormal.y);

		// the new x axis is at right angles to the world x axis
		contactTangent[0].x = 0;
		contactTangent[0].y = -mNormal.z * scalingFactor;
		contactTangent[0].z = mNormal.y * scalingFactor;

		// the new Y axis is at right angles to the new x and z axes
		contactTangent[1].x = mNormal.y * contactTangent[0].z - mNormal.z * contactTangent[0].y;
		contactTangent[1].y = -mNormal.x * contactTangent[0].z;
		contactTangent[1].z = mNormal.x * contactTangent[0].y;
	}

	contactToWorld.setComponents(mNormal, contactTangent[0], contactTangent[1]);
}

Vector3 Contact::calculateFrictionlessImpulse(Matrix3* inverseInertiaTensors) {
	Vector3 impulseContact; // this will be the final impulse returned

	// build a vector that shows the change in velocity in world space for a unit based impulse in the direction of the contact normal.
	Vector3 deltaVelWorld = relativeContactPositions[];
}


// Functions for each type of collision
std::map<std::pair<cyclone::ColliderType, cyclone::ColliderType>, std::function<bool(Collider*, Collider*, CollisionData*)>> CollisionTests::mMap =
{
	{	std::make_pair(ColliderType::Sphere, ColliderType::Sphere), [](Collider* a, Collider* b, CollisionData* data) { return SphereSphereCollide(*static_cast<SphereCollider*>(a), *static_cast<SphereCollider*>(b), data); }},
	{	std::make_pair(ColliderType::Sphere, ColliderType::Plane), [](Collider* a, Collider* b, CollisionData* data) { return SpherePlaneCollide(*static_cast<SphereCollider*>(a), *static_cast<PlaneCollider*>(b), data); }},
	{	std::make_pair(ColliderType::Plane, ColliderType::Plane), [](Collider* a, Collider* b, CollisionData* data) { return false; } },
};

bool CollisionTests::checkCollide(Collider* c1, Collider* c2, CollisionData* data)
{
	// get both types
	auto c1Type = c1->getType();
	auto c2Type = c2->getType();

	// try and find if its in the first type
	auto it1 = mMap.find({ c1Type, c2Type });
	if (it1 != mMap.end())
		return it1->second(c1, c2, data);
	// if it doesn't swap them and check the opposite
	else {
		auto it2 = mMap.find({ c2Type, c1Type });
		if (it2 != mMap.end())
			return it2->second(c2, c1, data);
		// if it still doesn't output and error 
		else {
			std::cout << "Error: Collision between types is not set up" << '\n';
			return false;
		}
	}
}
int CollisionTests::SphereSphereCollide(const SphereCollider& a, const SphereCollider& b, CollisionData* data)
{
	// get off set and normal
	Vector3 offset = a.getAxis(3) - b.getAxis(3);
	real dist = offset.magnitude();
	real radiiSum = a.getRadius() + b.getRadius();

	//// debug stuff:
	//Vector3 posA = a.getAxis(3);
	//Vector3 posB = b.getAxis(3);
	//Vector3 pointTest = posA - (offset / 2);
	//// output
	//std::cout << "Sphere A: " << posA << "\n";
	//std::cout << "Sphere B: " << posB << "\n";
	//std::cout << "Contact point: " << pointTest << "\n";

	//std::cout << "Contact Difference A: " << (pointTest - a.getRigidbody()->getPos()) << "\n";
	//std::cout << "Contact Difference B: " << (pointTest - b.getRigidbody()->getPos()) << "\n";


	// early out if there is no penetration
	if (dist >= radiiSum) return 0;

	// get penetration
	real penetration = radiiSum - dist;

	Vector3 normal = offset.unit();

	// get intersection point
	Vector3 point = a.getAxis(3) - (offset / 2);

	// generate contact
	Contact* contact = data->getNextContact();

	contact->mPoint = point;
	contact->mNormal = normal;
	contact->mPenetration = penetration;
	contact->mBodies[0] = a.getRigidbody();
	contact->mBodies[1] = b.getRigidbody();
	contact->mCoeffRest = data->getCoeffRestitution();

	data->incrementToNext();
	return 1;
}
int CollisionTests::SpherePlaneCollide(const SphereCollider& a, const PlaneCollider& b, CollisionData* data)
{
	real distance = b.getNormal() * a.getAxis(3) - a.getRadius() - b.getOffset();

	// exit if we are far away in a galaxy far away or nearby ig
	if (distance >= 0) return 0;

	// Get Penetration and intersection point
	real penetration = -distance;
	Vector3 point = a.getAxis(3) - (b.getNormal() * distance);

	// generate contact
	Contact* contact = data->getNextContact();

	contact->mPoint = point;
	contact->mNormal = b.getNormal();
	contact->mPenetration = penetration;
	contact->mBodies[0] = a.getRigidbody();
	contact->mBodies[1] = b.getRigidbody();
	contact->mCoeffRest = data->getCoeffRestitution();

	data->incrementToNext();
	return 1;
}

void CollisionData::init(unsigned int maxContacts)
{
	if (isInit) cleanup();

	mMaxContacts = maxContacts;
	mContactArray = new Contact[maxContacts];

	mContactCount = 0;
	isInit = true;
}
void CollisionData::cleanup()
{
	if (!isInit) return;
	free(mContactArray);
}
void CollisionData::incrementToNext()
{
	if (mContactCount > mMaxContacts)
	{
		std::cerr << "ERR// no more contacts left, contacts will be overridded\n";
		return;
	}
	mContactCount++;
}