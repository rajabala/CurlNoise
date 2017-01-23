using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System; // IntPtr


// Wrapper around the native DLL that acquires function pointers to exported functions and wraps
// calls to them.
// Uses Native.cs to handle loading and unloading of the DLL manually, so we can update the DLL
// when the Unity Editor is open.
public class NoisePluginWrapper : MonoBehaviour
{
    #region Debug Hooks
    // The block of code below is a neat trick to allow for calling into the debug console from C++
    delegate void DebugLog(string log);
    delegate void LinkDebug([MarshalAs(UnmanagedType.FunctionPtr)]IntPtr debugCal);

    private static readonly DebugLog debugLog = DebugWrapper;
    private static readonly IntPtr functionPointer = Marshal.GetFunctionPointerForDelegate(debugLog);
    private static void DebugWrapper(string log) { Debug.Log(log); }
    private static void HookDebugWrapperWithNativeCode()
    {
        Delegate fpLinkDebug = Native.GetFuncPtr<LinkDebug>(nativeLibraryPtr);
        if (fpLinkDebug != null)
            Native.InvokeFast(fpLinkDebug, functionPointer); // Let's the native plugin write into Unity console
    }
    #endregion

    #region Function Imports
    static string dllName = "CurlNoise";
    static IntPtr nativeLibraryPtr;
    static Delegate fpComputeCurlNoBoundaries;
    static Delegate fpComputeCurl;
    static Delegate fpNonBruteForceCurl;
    static Delegate fpUpdateCurlSettings;

    delegate Vector3 ComputeCurl_Unity(Vector3 p, IntPtr colliders, Int32 length);
    delegate Vector3 ComputeCurlNoBoundaries_Unity(Vector3 wsPos);
    delegate void    UpdateCurlSettings_Unity(  bool  bCheapGradient,
                                                float latticeSpacing,
                                                UInt32 numOctaves,
                                                float lacunarity,
                                                float persistence);
    #endregion

    void Awake()
    {
        if (nativeLibraryPtr != IntPtr.Zero) return;

        string dllPath = Application.dataPath + "/" + dllName;
        nativeLibraryPtr = Native.LoadLibrary(dllPath);
        if (nativeLibraryPtr == IntPtr.Zero)
        {
            Debug.LogError("Failed to load native library at " + dllPath);
        }

        HookDebugWrapperWithNativeCode();

        GetFunctionPointersToExportedNativeMethods();
    }


    private static void GetFunctionPointersToExportedNativeMethods()
    {
        fpComputeCurlNoBoundaries = Native.GetFuncPtr<Vector3 /*return type*/, ComputeCurlNoBoundaries_Unity>(nativeLibraryPtr);
        fpComputeCurl             = Native.GetFuncPtr<Vector3 /*return type*/, ComputeCurl_Unity>(nativeLibraryPtr);
        fpUpdateCurlSettings      = Native.GetFuncPtr<UpdateCurlSettings_Unity>(nativeLibraryPtr);
    }

    // Curl evaluated over 3D noise. Obstacles not considered.
    public static Vector3 ComputeCurlNB(Vector3 p)
    {
        if (fpComputeCurlNoBoundaries != null)
            return Native.InvokeFast<Vector3>(fpComputeCurlNoBoundaries, p);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
            return Vector3.zero;
        }
    }


    // Curl evaluated over 3D noise and obstacles.
    public static Vector3 ComputeCurlB(Vector3 p, IntPtr colliders, Int32 length)
    {
        if (fpComputeCurl != null)
            return Native.InvokeFast<Vector3>(fpComputeCurl, p, colliders, length);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
            return Vector3.zero;
        }
    }


    public static void UpdateCurlSettings(bool bCheapGradient,
                                          float latticeSpacing,
                                          int numOctaves,
                                          float lacunarity,
                                          float persistence)
    {
        if (fpUpdateCurlSettings != null)
            Native.InvokeFast(fpUpdateCurlSettings, bCheapGradient, latticeSpacing, (UInt32)numOctaves, lacunarity, persistence);
        else
        {
            Debug.LogError("Couldnt get fn ptr");
        }
    }


    void OnApplicationQuit()
    {
        if (nativeLibraryPtr == IntPtr.Zero) return;

        Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                      ? "Native library successfully unloaded."
                      : "Native library could not be unloaded.");
    }
}
