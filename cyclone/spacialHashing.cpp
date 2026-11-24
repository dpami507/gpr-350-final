#include "spacialHashing.h"

namespace cyclone
{
	SpacialHashing::SpacialHashing(int spacing)
	{
		mSpacing = spacing;
		mInverseSpacing = 1.0 / mSpacing;
	}

	void SpacialHashing::insert(Collider* collider)
	{
		const Vector3 colliderPos = collider->getAxis(3);
		Vector3Int hashVec(
			(int)ceil(colliderPos.x * mInverseSpacing),
			(int)ceil(colliderPos.y * mInverseSpacing),
			(int)ceil(colliderPos.z * mInverseSpacing)
		);
		mMap[hashVec].push_back(collider);

		const real boundingLength = collider->getBoundingLength();

		// Check for overlap between neighboring grids
		Vector3 axis[3] = {
			{1,0,0}, {0,1,0}, {0,0,1}
		};
		for (Vector3& a : axis)
		{
			Vector3Int tempHashVec;
			// check negative axis
			Vector3 min = colliderPos + (a * -boundingLength);
			tempHashVec = {
				(int)ceil(min.x * mInverseSpacing),
				(int)ceil(min.y * mInverseSpacing),
				(int)ceil(min.z * mInverseSpacing)
			};
			if (tempHashVec != hashVec)
				mMap[tempHashVec].push_back(collider);
			// check positive axis
			Vector3 max = colliderPos + (a * boundingLength);
			tempHashVec = {
				(int)ceil(max.x * mInverseSpacing),
				(int)ceil(max.y * mInverseSpacing),
				(int)ceil(max.z * mInverseSpacing)
			};
			if (tempHashVec != hashVec)
				mMap[tempHashVec].push_back(collider);
		}
	}
	void SpacialHashing::clear()
	{
		for (auto& c : mMap)
		{
			c.second.clear();
		}
	}
	void SpacialHashing::queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs)
	{
		for (std::pair<Vector3Int, std::vector<Collider*>> partition : mMap)
		{
			std::vector<Collider*>& vec = partition.second;
			if (vec.size() <= 1) continue;

			for (int i = 0; i < vec.size(); i++)
			{
				for (int j = i + 1; j < vec.size(); j++)
				{
					if (pow(vec[i]->getBoundingLength() + vec[j]->getBoundingLength(), 2) >
						(vec[i]->getAxis(3) - vec[j]->getAxis(3)).squareMagnitude()
						)
					{
						pairs.push_back({ vec[i], vec[j] });
					}
				}
			}
		}
		// does slash the fps by a bit but keeps it at 
		// that fps when they span out and saves a lot of memory
		for (auto it = mMap.begin(); it != mMap.end();)
		{
			// remove if only one inside
			if (it->second.size() <= 1)
				it = mMap.erase(it);
			else 
				++it;
		}
	}
}