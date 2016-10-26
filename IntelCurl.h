#pragma once

#include "vmath.hpp" // PORTME: vmath can be replaced with IG's internal math equivalents
#include <vector> // PORTME: replace with array / non-STL based implementation if need be
#include <memory> // PORTME: replace with non-STL unique_ptr implementation if need be
#define USING_UNITY 1

#if USING_UNITY
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT 
#endif

namespace IntelCurlNoise
{
	using namespace Vectormath::Aos;

	// This class encapsulates different volume collider primitives for the purpose of distance field estimation
	// It does NOT handle irregular scaling for curved primitives (for ex, ellipsoids, ellipsoid cylinders..) at the moment.
	// PORTME: IG Core already defines different Volume types. It seems to have a distance field function GetSignedDistancePoint(..)
	//         It handles all the primitive shapes defined below, but approximates some of them 
	//		   (for ex, box distance is Manhattan distance to the corner point)

	class Volume
	{
		enum eShape
		{
			kSphere,
			kCylinder,
			kBox,

			kCount
		};

	public:
		Volume(const Matrix4& worldToObjectNoScale, float r); // sphere
		Volume(const Matrix4& worldToObjectNoScale, const Vector3& extents); // box
		Volume(const Matrix4& worldToObjectNoScale, float r, float h); // cylinder

		float DistanceToSurface(const Vector3& wsPos) const;
		inline void SetWorldToObjectTransform(const Matrix4& m);

	protected:
		eShape m_Shape;
		Vector3 m_Extents; // world space units; interpreted based on the shape
		Matrix4 m_WorldToObject;
	};

	// API for visual effect and gameplay systems to use
	extern Vector3 ComputeCurl(Vector3 wsPos, const std::vector<std::unique_ptr<Volume>>& colliders);	

#if USING_UNITY
	extern "C"
	{
		struct float3
		{
			float val[3];
		};

		DLLEXPORT float3 ComputeCurl(Vector3 wsPos, Volume *pColliders, unsigned int length);

		DLLEXPORT float3 ComputeCurlAnalytical(Vector3 wsPos);
	}
#endif
};