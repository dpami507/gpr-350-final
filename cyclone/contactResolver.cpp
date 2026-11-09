#include "contactResolver.h"

using namespace cyclone;

real ContactResolver::coefOfRestitution = 0.5;

void ContactResolver::resolveContacts(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt)
{
	if (numContacts == 0) return;

	adjustPositions(contactArray, numContacts, dt);
	adjustVelocities(contactArray, numContacts, dt);
}
void ContactResolver::adjustPositions(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt)
{
	for (int i = 0; i < numContacts; i++)
	{
		// helpers
		Contact* currentContact = contactArray + i;
		Rigidbody* rbA = currentContact->mBodies[0];
		Rigidbody* rbB = currentContact->mBodies[1];

		real invMaA = (rbA != nullptr) ? rbA->getInverseMass() : 0;
		real invMaB = (rbB != nullptr) ? rbB->getInverseMass() : 0;

		real totalInverseMass = invMaA + invMaB;
		if (totalInverseMass <= 0) continue; // skip if both are infinite mass

		Vector3 dn = currentContact->mNormal * currentContact->mPenetration;

		if (rbA)
		{
			Vector3 deltaPosA = dn * (invMaA / totalInverseMass);
			Vector3 newA = rbA->getPos();
			newA += deltaPosA;
			rbA->setPosition(newA);
		}
		if (rbB)
		{
			Vector3 deltaPosB = dn * -(invMaB / totalInverseMass);
			Vector3 newB = rbB->getPos();
			newB += deltaPosB;
			rbB->setPosition(newB);
		}
	}
}
void ContactResolver::adjustVelocities(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt)
{
	for (int i = 0; i < numContacts; i++)
	{
		// saftey helpers
		Contact* currentContact = contactArray + i;
		Rigidbody* rbA = currentContact->mBodies[0];
		Rigidbody* rbB = currentContact->mBodies[1];

		Vector3 velA = (rbA != nullptr) ? rbA->getLinearVelocity() : Vector3::ZERO;
		Vector3 velB = (rbB != nullptr) ? rbB->getLinearVelocity() : Vector3::ZERO;

		real invMaA = (rbA != nullptr) ? rbA->getInverseMass() : 0;
		real invMaB = (rbB != nullptr) ? rbB->getInverseMass() : 0;

		Vector3 relativeVelocity = velA - velB;

		real totalInverseMass = invMaA + invMaB;
		if (totalInverseMass <= 0) continue; // skip if both are infinite mass

		// calculate seperating velocity (erm why is this working correctly)
		real seperatingVelocity = relativeVelocity * currentContact->mNormal;
		if (seperatingVelocity > 0) continue; // skip if we are going away from eachother
		real seperatingVelocityRestitution = -seperatingVelocity * coefOfRestitution;

		real deltaVelocity = seperatingVelocityRestitution - seperatingVelocity;

		// calculate impulse
		real impulseMagnitude = deltaVelocity / totalInverseMass;
		Vector3 impulse = currentContact->mNormal * impulseMagnitude;

		// apply
		if(rbA)
			rbA->setLinearVelocity(velA + impulse * invMaA);
		if (rbB)
			rbB->setLinearVelocity(velB - impulse * invMaB);
	}
}
