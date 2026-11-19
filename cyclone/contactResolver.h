#pragma once
#include "contacts.h"

namespace cyclone 
{
	class ContactResolver
	{
	public:
		ContactResolver() = default;
		~ContactResolver() = default;

		static void resolveContacts(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);
		//static inline void setCOR(real cor) { coefOfRestitution = cor; };
	protected:
		// sets up the contacts to be processed. this makes the internal data configure correctly. 
		// will make the correct bodies alive if we implement that from the book too.
		void prepareContacts(Contact* contactArray, unsigned numContacts, real deltaTime);

	private:
		static void adjustPositions(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);
		static void adjustVelocities(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);

		//static real coefOfRestitution;
	};
}