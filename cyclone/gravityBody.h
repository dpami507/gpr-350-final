#pragma once

#include "collider.h"
#include "core.h"

namespace cyclone
{
	class GravityBody
	{
	public:

		GravityBody();
		GravityBody(SphereCollider* sphereCol);
		~GravityBody() {}

		void update();

		static void calcGravForce(GravityBody* a, GravityBody* b);

		Vector3 getAxis(const uint8_t index) { return mCol->getAxis(index); }
		void setOffset(const Vector3& position) { mCol->setOffset(position); }
		void setOrientation(const Quaternion& orientation) { mCol->setOrientation(orientation); }

	private:

		SphereCollider* mCol;
		real mInvMass;

	};

	
}