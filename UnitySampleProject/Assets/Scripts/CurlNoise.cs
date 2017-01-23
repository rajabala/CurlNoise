using UnityEngine;
using System.Runtime.InteropServices;
using System;

// This script when attached to a game object containing a Particle System component,
// updates the velocities of each of the particles based on the curl of a potential (force)
// field and obstacles in the immediate vicinity of the particle.
// 
[RequireComponent(typeof(ParticleSystem))]
public class CurlNoise : MonoBehaviour
{ 
    public GameObject m_CurlForceFieldRegion;    
    public bool m_NoCurl;
    public bool m_CurlWithoutBoundaries;
    public Transform[] m_SphereColliders;

	[Range(0f, 1f)]
	public float strength = 1f;

    [Range(0.1f, 10f)]
    public float latticeSpacing = 1f;

    [Range(1, 8)]
    public int octaves = 1;

    [Range(0f, 5f)]
    public float lacunarity = 2f;

    [Range(0f, 1f)]
    public float persistence = 0.5f;

    private ParticleSystem system;
	private ParticleSystem.Particle[] particles;

    IntPtr ppColliders; // pointer to an array of pointers to Volume

    [Serializable]
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct Volume
    {
        public enum eShape { kSphere, kBox, kCylinder };

        [MarshalAs(UnmanagedType.U4, SizeConst = 4)]
        public eShape mShape;
        public Vector3 mExtent;
        public Matrix4x4 mWorldToObject;
    }

    [Serializable]
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    struct CurlSettings
    {       
        [MarshalAs(UnmanagedType.Bool)]
        public bool     mbCheapGradient;
        public float    mLatticeSpacing;
        public UInt32   mNumOctaves;
        public float    mLacunarity;
        public float    mPersistence;        
    }  

    void Awake()
    {
        // TODO extend to other primitives
        Int32 numColliders = m_SphereColliders.Length;
        ppColliders = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(IntPtr)) * numColliders);
    }

    private void LateUpdate ()
    {
		if (system == null) {
			system = GetComponent<ParticleSystem>();
            system.simulationSpace = ParticleSystemSimulationSpace.World;

        }
		if (particles == null || particles.Length < system.maxParticles) {
			particles = new ParticleSystem.Particle[system.maxParticles];
		}
		int particleCount = system.GetParticles(particles);

        if (!m_NoCurl)
        {
            Bounds b = system.GetComponent<ParticleSystemRenderer>().bounds;

            if (b.Intersects(m_CurlForceFieldRegion.GetComponent<Renderer>().bounds))
            {
                NoisePluginWrapper.UpdateCurlSettings(false, latticeSpacing, octaves, lacunarity, persistence);

                UpdateParticleVelocity();

                system.SetParticles(particles, particleCount);
            }            
        }        		
    }


    private void UpdateParticleVelocity()
    {
        Int32 numColliders = m_SphereColliders.Length;
        IntPtr ptr;
        uint c_id = 0;
        IntPtr tmp = ppColliders;

        foreach (Transform t in m_SphereColliders)
        {
            Volume obstacle = new Volume();
            
            obstacle.mShape = Volume.eShape.kSphere;
            obstacle.mExtent = t.lossyScale * 0.5f;
            obstacle.mWorldToObject = Matrix4x4.TRS(t.position, t.rotation, Vector3.one);

            Marshal.StructureToPtr(obstacle, tmp, false);
            tmp = (IntPtr)(tmp.ToInt64() + Marshal.SizeOf(tmp)); // advance pointer            
        }


        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 ppos = particles[i].position;

            Vector3 p_to_c = ppos - m_CurlForceFieldRegion.transform.position;
            float r = m_CurlForceFieldRegion.transform.lossyScale.x * 0.5f;
            if (p_to_c.sqrMagnitude <  r * r)
            {
                Vector3 v = new Vector3();

                if (m_CurlWithoutBoundaries)
                {
                    Profiler.BeginSample("CC-NoBoundaries");
                    Vector3 curl = NoisePluginWrapper.ComputeCurlNB(ppos);
                    v = strength * curl;
                    Profiler.EndSample();
                }
                else
                {
                    Profiler.BeginSample("CC-YesBoundaries");
                    Vector3 curl = NoisePluginWrapper.ComputeCurlB(ppos, ppColliders, numColliders);
                    v = strength * curl;
                    Profiler.EndSample();
                }

                particles[i].velocity = new Vector3(0f, 0.04f, 0f) + v;
            }
        }
    }  

    void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(m_CurlForceFieldRegion.transform.position, m_CurlForceFieldRegion.transform.lossyScale.x * 0.5f);
    }
}