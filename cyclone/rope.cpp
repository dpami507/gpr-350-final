#include "rope.h"

namespace cyclone
{
	Rope::Rope(int segmentCount, real segmentLength, real thickness, real damping, real mass, Vector3 position, Vector3 accel)
	{
		mParticles.reserve(segmentCount);
		mSegmentLength = segmentLength;
		mThickness = thickness;

		for (int i = 0; i < segmentCount; i++)
		{
			VerletParticle currentParticle;

			Vector3 newPos = position;
			newPos.x += (mSegmentLength * i) - ((segmentLength * segmentCount / 2));
			currentParticle.mPosition = newPos;
			currentParticle.mPrevPosition = newPos;

			currentParticle.mDamping = damping;
			currentParticle.mInverseMass = 1.0 / mass;
			currentParticle.mAcceleration = accel;

			mParticles.emplace_back(currentParticle);
		}
	}
	Rope::~Rope()
	{
		mParticles.clear();
	}
	void Rope::integrate(int constraintIterations, cyclone::real dt)
	{
		for (int j = 0; j < mParticles.size(); j++)
		{
			mParticles[j].integrate(dt);
		}

		// Relaxation for the distance
		for (int i = 0; i < constraintIterations; i++)
		{
			for (int j = 0; j < mParticles.size() - 1; j++)
			{
				Constraints::distanceConstraint(mParticles[j], mParticles[j + 1], mSegmentLength);
			}
		}
	}
	void Rope::setPosition(const Vector3& pos)
	{
		for (int i = 0; i < mParticles.size(); i++)
		{
			Vector3 newPos = pos;
			newPos.x += (mSegmentLength * i) - ((mSegmentLength * mParticles.size() / 2));
			mParticles[i].mPosition = newPos;
			mParticles[i].mPrevPosition = newPos;
		}
	}
	void Rope::checkCollision(Collider* collider, real frictionCoef)
	{
		if (collider->getType() == ColliderType::Sphere)
		{
			SphereCollider* sphereColldier = dynamic_cast<SphereCollider*>(collider);

			for (VerletParticle& p : mParticles)
			{
				Vector3 offset = p.mPosition - sphereColldier->getAxis(3);
				real dist = offset.magnitude();
				real radiiSum = mThickness + sphereColldier->getRadius();

				// early out if there is no penetration
				if (dist >= radiiSum) continue;

				real penetration = radiiSum - dist;
				handleCollision(collider, p, penetration, offset.unit(), frictionCoef);
			}
		}
		else if (collider->getType() == ColliderType::Plane)
		{
			PlaneCollider* planeColldier = dynamic_cast<PlaneCollider*>(collider);

			for (VerletParticle& p : mParticles)
			{
				real distance = planeColldier->getNormal() * p.mPosition - mThickness - planeColldier->getOffset();

				// exit if we are far away in a galaxy far away or nearby ig
				if (distance >= 0) continue;

				real penetration = -distance;
				handleCollision(collider, p, penetration, planeColldier->getNormal(), frictionCoef);
			}
		}
	}
	void Rope::setDamping(real damping)
	{
		for (VerletParticle& p : mParticles)
		{
			p.mDamping = damping;
		}
	}
	void Rope::handleCollision(Collider* collider, VerletParticle& particle, real penetration, const Vector3& normal, real frictionCoef)
	{
		// ----------------------------- Position ---------------------------------- \\

		// Calc collisions
		Rigidbody* colliderRb = collider->getRigidbody();
		real colliderInvMa = (colliderRb != nullptr) ? colliderRb->getInverseMass() : 0;

		real totalInverseMass = colliderInvMa + particle.mInverseMass;
		if (totalInverseMass <= 0) return; // skip if both are infinite mass

		Vector3 dn = normal * penetration;

		if (colliderRb)
		{
			Vector3 deltaPosA = dn * -(colliderInvMa / totalInverseMass);
			Vector3 newPos = colliderRb->getPos() + deltaPosA;
			colliderRb->setPosition(newPos);
		}
		Vector3 deltaPosB = dn * (particle.mInverseMass / totalInverseMass);
		Vector3 newB = particle.mPosition + deltaPosB;
		particle.mPosition = newB;

		// ----------------------------- Velcoity ---------------------------------- \\

		Vector3 particleVel = particle.mPosition - particle.mPrevPosition;
		Vector3 tangentVelocity = particleVel - normal * (particleVel * normal);
		Vector3 frictionImpulse = -tangentVelocity.unit() * std::min(frictionCoef * penetration, tangentVelocity.magnitude());
		Vector3 newTangentVelocity = tangentVelocity + frictionImpulse;

		particle.mPrevPosition = particle.mPosition - newTangentVelocity;
	}
}