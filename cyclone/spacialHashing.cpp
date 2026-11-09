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
		Vector3 colliderPos = collider->getAxis(3);
		Vector3Int hashVec(
			(int)ceil(colliderPos.x * mInverseSpacing),
			(int)ceil(colliderPos.y * mInverseSpacing),
			(int)ceil(colliderPos.z * mInverseSpacing)
		);
		mMap[hashVec].push_back(collider);

		// Check for overlap between neighboring grids
		Vector3 axis[3] = {
			{1,0,0}, {0,1,0}, {0,0,1}
		};
		for (auto a : axis)
		{
			Vector3Int tempHashVec;
			// check negative axis
			Vector3 min = colliderPos + (a * -collider->getBoundingLength());
			tempHashVec = {
				(int)ceil(min.x * mInverseSpacing),
				(int)ceil(min.y * mInverseSpacing),
				(int)ceil(min.z * mInverseSpacing)
			};
			if (tempHashVec != hashVec)
				mMap[tempHashVec].push_back(collider);
			// check positive axis
			Vector3 max = colliderPos + (a * collider->getBoundingLength());
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
		mMap.clear();
	}
	void SpacialHashing::queryPossiblePairs(std::vector<std::pair<Collider*, Collider*>>& pairs)
	{
		for (std::pair<Vector3Int, std::vector<Collider*>> partition : mMap)
		{
			std::vector<Collider*>& vec = partition.second;
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
	}
}