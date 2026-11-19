#pragma once

#include "collider.h"
#include "core.h"

namespace cyclone
{

	//reduced by a factor of 10^6
	const static real GRAVCONST = 6.67 * pow(10, -5);

	class GravityBody
	{
	public:

		GravityBody();
		GravityBody(SphereCollider* sphereCol);
		GravityBody(Vector3 pos, real radius);
		~GravityBody() {}

		void update();
		void integrate(real dt) { mCol->getRigidbody()->integrate(dt); };

		static void calcGravForce(GravityBody* a, GravityBody* b);

		Vector3 getAxis(const uint8_t index) { return mCol->getAxis(index); }
		void setOffset(const Vector3& position) { mCol->setOffset(position); }
		void setOrientation(const Quaternion& orientation) { mCol->setOrientation(orientation); }

		void setInvMass(real invmass);
		void setMass(real mass);
		real getInvMass();
		real getMass();

		Matrix4 getTransform() { return mCol->getTransform(); }
		Collider* getCollider() { return mCol; }

	private:

		SphereCollider* mCol;

	};

	
}