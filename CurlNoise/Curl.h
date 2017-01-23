#pragma once

#include "vmath.hpp" // PORTME: vmath can be replaced with IG's internal math equivalents
#define USING_UNITY 1

#if USING_UNITY
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT 
#endif

namespace CurlNoise
{
	using namespace Vectormath::Aos;

	// This class encapsulates different volume collider primitives for the purpose of distance field estimation
	// It does NOT handle irregular scaling for curved primitives (for ex, ellipsoids, ellipsoid cylinders..) at the moment.	
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
		inline Vector3 GetWorldPos() const;

	protected:
		eShape m_Shape;
		Vector3 m_Extents; // world space units; interpreted based on the shape
		Matrix4 m_WorldToObject; // inverse of the object to world matrix consisting of only translation and rotation
	};


	struct CurlSettings
	{
    CurlSettings(unsigned int numOctaves, 
                 float lacunarity, 
                 float persistence, 
                 float lattingSpacing, 
                 bool cheapGradient = true)
    {
      m_bCheapGradient = cheapGradient;
      m_LatticeSpacing = lattingSpacing;
      m_NumOctaves     = numOctaves;
      m_Lacunarity     = lacunarity;
      m_Persistence    = persistence;
    }

		// true => the gradient is calculated as vector from center of nearest collider to the particle
		// false => the gradient is calculated based on the change of the distance field wrt xyz
		bool			    m_bCheapGradient = true; 
		
		//--- See PerlinNoise3::Noise for how the following fields are used ---
		// Controls the spatial granularity at which noise gradients are assigned (in world space units)
    // Larger the value, the more space is covered by the same trail
		float					m_LatticeSpacing = 1.f; // 1 world unit

		// number of octaves to sum up. each successive octave scales the previous octave's frequency by the lacunarity
		unsigned int	m_NumOctaves = 2;

		// factor by which frequency changes in successive octaves
		float			    m_Lacunarity = 2.0f;

		// factor by which amplitude changes in successive octaves
		float			    m_Persistence = 0.5f;		
	};

#if USING_UNITY
	extern "C"
	{		
		struct float3
		{
			float val[3];		
		};
		
    DLLEXPORT float3 ComputeCurl_Unity(Vector3 wsPos, 
                                       const Volume* const *pColliders, 
                                       unsigned int length);

		DLLEXPORT float3 ComputeCurlNoBoundaries_Unity(Vector3 wsPos);

    DLLEXPORT void   UpdateCurlSettings_Unity(bool          bCheapGradient,
                                              float	        latticeSpacing,
                                              unsigned int	numOctaves,
                                              float			    lacunarity,
                                              float			    persistence);
	}
#else
	// API for visual effect system to use
  Vector3 ComputeCurl(const CurlSettings& settings, Vector3 wsPos, const Volume* const *pColliders, unsigned int length); 
#endif

};