#include "gravityBody.h"

namespace cyclone
{
	GravityBody::GravityBody()
	{
		Rigidbody* rb = new Rigidbody();
		mCol = new SphereCollider(rb, Matrix4(), 0.5);
	}
	GravityBody::GravityBody(SphereCollider* sphereCol) :mCol(sphereCol)
	{
	}
	GravityBody::GravityBody(Vector3 pos, real radius)
	{
		Rigidbody* rb = new Rigidbody(pos);
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

		a->mCol->getRigidbody()->addForce(-dir * aMagnitude);
		b->mCol->getRigidbody()->addForce(dir * bMagnitude);
	}

	void GravityBody::setInvMass(real invmass) 
	{
		mCol->getRigidbody()->setInvMass(invmass);
	}
	void GravityBody::setMass(real mass)
	{
		mCol->getRigidbody()->setInvMass((mass != 0) ? 1 / mass : 0);
	}
	real GravityBody::getInvMass()
	{
		return mCol->getRigidbody()->getInverseMass();
	}
	real GravityBody::getMass()
	{
		real invmass = mCol->getRigidbody()->getInverseMass();
		return (invmass != 0) ? 1 / invmass : 0;
	}
}