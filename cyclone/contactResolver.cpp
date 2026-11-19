#include "contactResolver.h"

using namespace cyclone;

//real ContactResolver::coefOfRestitution = 0.5;

void ContactResolver::resolveContacts(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt)
{
	if (numContacts == 0) return;

	adjustVelocities(contactArray, numContacts, dt);
	adjustPositions(contactArray, numContacts, dt);
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

// Angular Calculation upgrade derived from here: https://physics.stackexchange.com/questions/783524/angular-motion-in-collisions
void ContactResolver::adjustVelocities(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt)
{
	for (int i = 0; i < numContacts; i++)
	{
		// saftey helpers
		Contact* currentContact = contactArray + i;
		Rigidbody* rbA = currentContact->mBodies[0];
		Rigidbody* rbB = currentContact->mBodies[1];

		// adjust how the rest of velocity is calculated in order to take into account the rotation forces.
		Vector3 velA = (rbA != nullptr) ? rbA->getLinearVelocity() : Vector3::ZERO;
		Vector3 velB = (rbB != nullptr) ? rbB->getLinearVelocity() : Vector3::ZERO;

		real invMaA = (rbA != nullptr) ? rbA->getInverseMass() : 0;
		real invMaB = (rbB != nullptr) ? rbB->getInverseMass() : 0;

		// calculate the vectors from the center of each object's mass to the contact point for angular collisions
		Vector3 contactDiffA = (currentContact->mPoint - ((rbA != nullptr) ? rbA->getPos() : Vector3::ZERO));
		Vector3 contactDiffB = (currentContact->mPoint - ((rbB != nullptr) ? rbB->getPos() : Vector3::ZERO));

		// contact velocity at the point of impact for taking into account angular velocity (3 on website):
		Vector3 contactVelA = velA + ((rbA != nullptr) ? rbA->getAngularVelocity() : Vector3::ZERO) % contactDiffA;
		Vector3 contactVelB = velB + ((rbB != nullptr) ? rbB->getAngularVelocity() : Vector3::ZERO) % contactDiffB;

		Vector3 relativeVelocity = contactVelA - contactVelB;

		// to account for angular collision, we need to get the effective mass of the contact, aka the reduced mass of the system. This used to be total inv mass. (9 on website)
		real effectiveInverseMass = invMaA + invMaB +
			((contactDiffA % currentContact->mNormal) * (((rbA != nullptr) ? rbA->getWorldInvInerTensor() : Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0)) * (contactDiffA % currentContact->mNormal))) +
			((contactDiffB % currentContact->mNormal) * (((rbB != nullptr) ? rbB->getWorldInvInerTensor() : Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0)) * (contactDiffB % currentContact->mNormal)));
		if (effectiveInverseMass <= 0) continue; // skip if both are infinite mass

		// calculate seperating velocity now with relation to angular velocity (6 on website)
		real seperatingVelocity = currentContact->mNormal * relativeVelocity;
		if (seperatingVelocity > 0) continue; // skip if the rigidbodies are already going away from each other

		// calculate for both velocity and angular impulses
		real impulseMagnitude = (-1.0f * (1.0f + currentContact->mCoeffRest) * seperatingVelocity) / effectiveInverseMass; // THIS is J throughout the website (8 specifically)
		Vector3 vectorImpulse = currentContact->mNormal * impulseMagnitude;

		// apply to linear velocity
		if(rbA)
			rbA->setLinearVelocity(velA + vectorImpulse * invMaA);
		if (rbB)
			rbB->setLinearVelocity(velB - vectorImpulse * invMaB);

		// apply to angular velocity
		if (rbA)
			rbA->setAngularVelocity(rbA->getAngularVelocity() + (rbA->getWorldInvInerTensor() * (contactDiffA % vectorImpulse)));
		if (rbB)
			rbB->setAngularVelocity(rbB->getAngularVelocity() + (rbB->getWorldInvInerTensor() * (contactDiffB % (-vectorImpulse))));

		if (rbA)
			rbA->calculateDerivedData();
		if (rbB)
			rbB->calculateDerivedData();
	}
}
