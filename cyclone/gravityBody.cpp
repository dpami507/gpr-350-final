#include "gravityBody.h"

namespace cyclone
{
	GravityBody::GravityBody()
	{
		Rigidbody* rb = new Rigidbody();
		mInvMass = rb->getInverseMass();
		mCol = new SphereCollider(rb, Matrix4(), 0.5);
	}
	GravityBody::GravityBody(SphereCollider* sphereCol) :mCol(sphereCol)
	{
		mInvMass = sphereCol->getRigidbody()->getInverseMass();
	}
	GravityBody::GravityBody(Vector3 pos, real radius)
	{
		Rigidbody* rb = new Rigidbody(pos);
		mInvMass = rb->getInverseMass();
		mCol = new SphereCollider(rb, Matrix4(), radius);
	}

	void GravityBody::update()
	{
		mCol->updateInternals();
	}

	void GravityBody::calcGravForce(GravityBody* a, GravityBody* b)
	{
		Vector3 diff = a->mCol->getAxis(3) - b->mCol->getAxis(3);
		real dist = diff.magnitude();
		Vector3 dir = diff.unit();

		if (dist == 0) return;

		real aMagnitude = a->getMass() * (GRAVCONST * b->getMass()) / (dist*dist);
		real bMagnitude = b->getMass() * (GRAVCONST * a->getMass()) / (dist*dist);

		a->mCol->getRigidbody()->setAcceleration(-dir * aMagnitude);
		b->mCol->getRigidbody()->setAcceleration(dir * bMagnitude);
	}

	void GravityBody::setInvMass(real invmass) 
	{
		mInvMass = invmass;
		updateColMass();
	}
	void GravityBody::setMass(real mass)
	{
		mInvMass = (mass != 0) ? 1 / mass : 0;
		updateColMass();
	}
	real GravityBody::getInvMass()
	{
		return mInvMass;
	}
	real GravityBody::getMass()
	{
		return (mInvMass != 0) ? 1 / mInvMass : 0;
	}
	void GravityBody::updateColMass()
	{
		mCol->getRigidbody()->setInvMass(mInvMass);
	}
}