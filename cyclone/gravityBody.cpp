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

	void GravityBody::update()
	{
		mCol->updateInternals();
	}

	void GravityBody::calcGravForce(GravityBody* a, GravityBody* b)
	{
		
	}
}