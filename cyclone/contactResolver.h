#pragma once
#include "contacts.h"

namespace cyclone 
{
	class ContactResolver
	{
	public:
		ContactResolver() = default; // shouldn't be called now
		ContactResolver(unsigned iterations = 10, real velocityEpilson = 0.01, real positionEpilson = 0.01) {
			mVelocityIterations = iterations;
			mPositionIterations = iterations;
			mVelocityEpilson = velocityEpilson;
			mPositionEpilson = positionEpilson;
		}
		~ContactResolver() = default;

		void resolveContacts(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);
		inline void setCollisionIterations(int numIterations) { mVelocityIterations = numIterations; mPositionIterations = numIterations; };

	private:
		// sets up the contacts to be processed. this makes the internal data configure correctly. 
		// will make the correct bodies alive if we implement that from the book too.
		void prepareContacts(Contact* contactArray, unsigned numContacts, real deltaTime);

		void adjustPositions(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);
		void adjustVelocities(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);

		unsigned mVelocityIterations;
		unsigned mVelIterationsUsed = 0;
		unsigned mPositionIterations;
		unsigned mPosIterationUsed = 0;

		real mVelocityEpilson;
		real mPositionEpilson;
	};
}