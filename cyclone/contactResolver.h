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
		static inline void setCOR(real cor) { coefOfRestitution = cor; };
	private:
		static void adjustPositions(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);
		static void adjustVelocities(cyclone::Contact* contactArray, unsigned int numContacts, cyclone::real dt);

		static real coefOfRestitution;
	};
}