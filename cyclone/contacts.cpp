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
	Vector3 deltaVelWorld = relativeContactPositions[0] % mNormal;
	deltaVelWorld = inverseInertiaTensors[0].transform(deltaVelWorld);
	deltaVelWorld = deltaVelWorld % relativeContactPositions[0];

	// work out the change in velocity in contact coords
	real deltaVel = deltaVelWorld * mNormal;

	// add the linear component of the velocity change
	deltaVel += mBodies[0]->getInverseMass();

	// do the second body's data too if it exists
	if (mBodies[1]) {
		// build a vector that shows the change in velocity in world space for a unit based impulse in the direction of the contact normal.
		Vector3 deltaVelWorld2 = relativeContactPositions[1] % mNormal;
		deltaVelWorld2 = inverseInertiaTensors[1].transform(deltaVelWorld2);
		deltaVelWorld2 = deltaVelWorld2 % relativeContactPositions[1];

		// work out the change in velocity in contact coords
		deltaVel += deltaVelWorld2 * mNormal;

		// add the linear component of the velocity change
		deltaVel += mBodies[1]->getInverseMass();
	}

	// calculate the requried size of the impulse
	impulseContact.x = desiredDeltaVel / deltaVel;
	impulseContact.y = 0;
	impulseContact.z = 0;
	return impulseContact;
}

Vector3 Contact::calculateFrictionImpulse(Matrix3* inverseInertiaTensors) {
	Vector3 impulseContact;
	real inverseMass = mBodies[0]->getInverseMass();

	// the equivalent of a cross product in matricies is multiplication by a skew symmetric matrix, so we build the matrix
	// for converting between linear and angular quantities
	Matrix3 impulseToTorque;
	impulseToTorque.setSkewSymmetric(relativeContactPositions[0]);

	// Build the matrix to convert contact impulse to a change in velocity in world coords.
	Matrix3 deltaVelWorld = impulseToTorque;
	deltaVelWorld *= inverseInertiaTensors[0];
	deltaVelWorld *= impulseToTorque;
	deltaVelWorld *= -1;

	// check if we need to add the second body's data
	if (mBodies[1]) {
		// set the cross product matrix
		impulseToTorque.setSkewSymmetric(relativeContactPositions[1]);

		// calculate the velocity change matrix
		Matrix3 deltaVelWorld2 = impulseToTorque;
		deltaVelWorld2 *= inverseInertiaTensors[1];
		deltaVelWorld2 *= impulseToTorque;
		deltaVelWorld2 *= -1;

		// add to the total delta velocity
		deltaVelWorld += deltaVelWorld2;

		// add to the inverse mass
		inverseMass += mBodies[1]->getInverseMass();
	}

	// change the basis to convert to contact coordinates
	Matrix3 deltaVel = contactToWorld.transpose();
	deltaVel *= deltaVelWorld;
	deltaVel *= contactToWorld;
	
	// add the linear velocity change
	deltaVel.data[0] += inverseMass;
	deltaVel.data[4] += inverseMass;
	deltaVel.data[8] += inverseMass;

	// Invert to get the impulse per unit
	Matrix3 impulseMatrix = deltaVel.inverse();

	// find the target velocities to completely stop the movement of
	Vector3 velToStop(desiredDeltaVel, -contactVelocity.y, -contactVelocity.z);

	// find the impulse to stop the target velocities
	impulseContact = impulseMatrix.transform(velToStop);

	// check for extra friction
	real planarImpulse = real_sqrt(impulseContact.y * impulseContact.y * impulseContact.z * impulseContact.z);
	if (planarImpulse > impulseContact.x * mFriction) {
		 // use dynamic friction
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;

		impulseContact.x = deltaVel.data[0] +
			deltaVel.data[1] * mFriction * impulseContact.y +
			deltaVel.data[2] * mFriction * impulseContact.z;
		impulseContact.x = desiredDeltaVel / impulseContact.x;
		impulseContact.y *= mFriction * impulseContact.x;
		impulseContact.z *= mFriction * impulseContact.x;
	}
	return impulseContact;
}

void Contact::applyVelocityChange(Vector3 velChange[2], Vector3 rotChange[2]) {
	// get the world coords inverse mass and world coords inverse inertia tensor
	Matrix3 inverseInertiaTensor[2];
	mBodies[0]->getWorldInvInerTensor(&inverseInertiaTensor[0]); // doing it this way is easier and saves memory
	if(mBodies[1]) mBodies[1]->getWorldInvInerTensor(&inverseInertiaTensor[1]);

	// we will calculate the impulse for each contact axis
	Vector3 impulseContact;

	// If we have friction, then do a friction calculation. otherwise, frictionless
	if (mFriction <= 0.f) impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
	else impulseContact = calculateFrictionImpulse(inverseInertiaTensor);

	// convert the impulse calculated to world coordinates
	Vector3 impulse = contactToWorld.transform(impulseContact);

	// split the impulse into linear and rotation parts
	Vector3 impulsiveTorque = relativeContactPositions[0] % impulse;
	rotChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
	velChange[0].clear();
	velChange[0].addScaledVector(impulse, mBodies[0]->getInverseMass());

	// add the changes
	mBodies[0]->setLinearVelocity(mBodies[0]->getLinearVelocity() + velChange[0]);
	mBodies[0]->setAngularVelocity(mBodies[0]->getAngularVelocity() + rotChange[0]);

	//mBodies[0]->addForce(velChange[0]); these are wrong
	//mBodies[0]->addTorque(rotChange[0]);

	// do the same to the second body if it exists
	if (mBodies[1]) {
		// split the impulse into linear and rotation parts
		Vector3 impulsiveTorque2 = impulse % relativeContactPositions[1]; // flipped for 2nd body
		rotChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque2);
		velChange[1].clear();
		velChange[1].addScaledVector(impulse, -mBodies[1]->getInverseMass());

		// add the changes
		mBodies[1]->setLinearVelocity(mBodies[1]->getLinearVelocity() + velChange[1]);
		mBodies[1]->setAngularVelocity(mBodies[1]->getAngularVelocity() + rotChange[1]);
	}
}

void Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration) {
	const real angularLimit = (real)0.2f;
	real angularMove[2];
	real linearMove[2];

	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];

	// we need to work out the inertia of each object in the direction of the contact normal, in respect to angular inertia only
	// we are looping this to save from copying and pasting it twice
	for (unsigned i = 0; i < 2; i++) {
		if (mBodies[i]) {
			Matrix3 inverseInertiaTensor;
			mBodies[i]->getWorldInvInerTensor(&inverseInertiaTensor);

			// use the same method for calculating frictionless velocity change to find the angular inertia
			Vector3 angularInertiaWorld = relativeContactPositions[i] % mNormal;
			angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
			angularInertiaWorld = angularInertiaWorld % relativeContactPositions[i];

			angularInertia[i] = angularInertiaWorld * mNormal;

			// the linear component of this is just the inverse mass
			linearInertia[i] = mBodies[i]->getInverseMass();

			totalInertia += linearInertia[i] + angularInertia[i];
		}
	}

	// next loop through again to calculate and apply the changes
	for (unsigned i = 0; i < 2; i++) {
		if (mBodies[i]) {
			// the linear and angular movements required are in proportion to the two inverse intertias
			real isOtherDir = (i == 0) ? 1 : -1; // negative if it is the other body
			angularMove[i] = isOtherDir * penetration * (angularInertia[i] / totalInertia);
			linearMove[i] = isOtherDir * penetration * (linearInertia[i] / totalInertia);

			// limit the angular movement to avoid angular projections that are too great
			Vector3 projection = relativeContactPositions[i];
			projection.addScaledVector(mNormal, -relativeContactPositions[i].scalarProduct(mNormal));

			// use a small angle approximation for the sine of the angle, the magnitude would be sine(angularLimit) * projection.magnitude but we approximate the sine of angular limit to just angularLimit save memory
			real maxMagnitude = angularLimit * projection.magnitude();
			
			if (angularMove[i] < -maxMagnitude) {
				real totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude) {
				real totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			// we have the linear amount of movement required by turning the body in angularMove[i], so we now need to calculate the desired rotation to achieve that
			if (angularMove[i] == 0) {
				// no movement is no rotation
				angularChange[i].clear();
			}
			else {
				// find the directiion we would want to rotate in
				Vector3 targetAngularDirection = relativeContactPositions[i].vectorProduct(mNormal);
				Matrix3 inverseInertiaTensor;
				mBodies[i]->getWorldInvInerTensor(&inverseInertiaTensor);
				// find the direction we need to rotate that
				angularChange[i] = inverseInertiaTensor.transform(targetAngularDirection) * (angularMove[i] / angularInertia[i]); 
			}

			// velocity change is just the linear movement along the contact normal
			linearChange[i] = mNormal * linearMove[i];

			// now apply the values
			// linear applying
			Vector3 pos;
			mBodies[i]->getPos(&pos);
			pos.addScaledVector(mNormal, linearMove[i]);
			mBodies[i]->setPosition(pos);

			// angular applying
			Quaternion q; 
			mBodies[i]->getOrientation(&q);
			q.addScaledVector(angularChange[i], ((real)1.0f));
			mBodies[i]->setOrientation(q);

			// update the derived data
			mBodies[i]->calculateDerivedData();
		}
	}
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
	Vector3 point = a.getAxis(3) - normal * a.getRadius();

	// generate contact
	Contact* contact = data->getNextContact();

	contact->mPoint = point;
	contact->mNormal = normal;
	contact->mPenetration = penetration;
	contact->mBodies[0] = a.getRigidbody();
	contact->mBodies[1] = b.getRigidbody();
	contact->mCoeffRest = data->getCoeffRestitution();
	contact->mFriction = data->getFriction();

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
	Vector3 point = a.getAxis(3) - b.getNormal() * (distance + a.getRadius());

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
	//free(mContactArray); // this is causing crashes
	delete[] mContactArray;
	mContactArray = nullptr;
	mContactCount = 0;
	mMaxContacts = 0;
	isInit = false;
}
void CollisionData::incrementToNext()
{
	if (mContactCount >= mMaxContacts)
	{
		std::cerr << "ERR// no more contacts left, contacts will be overridded\n";
		return;
	}
	mContactCount++;
}