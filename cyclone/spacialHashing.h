#pragma once
#include <unordered_map>
#include <iostream>
#include "core.h"
#include "collider.h"

namespace cyclone
{
	struct Vector3IntHash {
		size_t operator()(const cyclone::Vector3Int& v) const {
			return (size_t(v.x) * 73856093) ^ (size_t(v.y) * 19349663) ^ (size_t(v.z) * 83492791);
		}
	};

	class SpacialHashing
	{
	public:
		SpacialHashing(int spacing);
		~SpacialHashing() = default;

		void insert(Collider* collider);
		void clear();
		void queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs);
	private:
		std::unordered_map<Vector3Int, std::vector<Collider*>, Vector3IntHash> mMap;
		int mSpacing;
		real mInverseSpacing;
	};
}