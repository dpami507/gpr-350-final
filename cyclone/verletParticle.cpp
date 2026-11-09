#include "verletParticle.h"

namespace cyclone
{
	void Constraints::distanceConstraint(VerletParticle& p1, VerletParticle& p2, real restLength)
	{
		Vector3 delta = p2.mPosition - p1.mPosition;
		real deltaLength = delta.magnitude();
		if (deltaLength == 0) return;
		real diff = (deltaLength - restLength) / (deltaLength * (p1.mInverseMass + p2.mInverseMass));

		p1.mPosition += delta * 0.5 * diff;
		p2.mPosition -= delta * 0.5 * diff;
	}
}