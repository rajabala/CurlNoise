#pragma once

#include "vmath.hpp" // PORTME: vmath can be replaced with IG's internal math equivalents
#include <vector> 	 // PORTME: replace with array / non-STL based implementation if need be
#include <memory> 	 // PORTME: replace with non-STL unique_ptr implementation if need be
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
	//		   I'd guess that being tied to SceneObject, that would end up being shared with the vfx system.
	//		   This is just a place holder for proof of concept and testing.	
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
		// constructor overloads for various primitive types
		Volume(const Matrix4& worldToObjectNoScale, float r); // sphere
		Volume(const Matrix4& worldToObjectNoScale, const Vector3& extents); // box
		Volume(const Matrix4& worldToObjectNoScale, float r, float h); // cylinder

		float DistanceToSurface(const Vector3& wsPos) const;
		inline void SetWorldToObjectTransform(const Matrix4& m);

	protected:
		eShape m_Shape;
		Vector3 m_Extents; // world space units; interpreted based on the shape
		Matrix4 m_WorldToObject; // inverse of the object to world matrix consisting of only translation and rotation
	};

	// API for visual effect system to use
	Vector3 ComputeCurl(Vector3 wsPos, const Volume *pColliders, unsigned int length);
	Vector3 ComputeCurlWithoutObstacles(Vector3 wsPos);

#if USING_UNITY
	extern "C"
	{		
		struct float3
		{
			float val[3];		
		};
		
		DLLEXPORT float3 ComputeCurlBruteForce(Vector3 wsPos, Volume *pColliders, unsigned int length);

		DLLEXPORT float3 ComputeCurlNoBoundaries(Vector3 wsPos);

		DLLEXPORT float3 ComputeCurlNonBruteForce(Vector3 wsPos, Volume *pColliders, unsigned int length);
	}
#endif
};