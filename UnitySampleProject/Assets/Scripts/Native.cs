/* 
 * Native dll invocation helper by Francis R. Griffiths-Keam
 * www.runningdimensions.com/blog
 * 
 * Modified to get function pointers and invoke an acquired fn ptr,
 * rather than acquire each time
 * 
 * This class handles loading and unloading of native (non-rendering)
 * plugins for Unity, so the dll can be rebuilt with the editor open.
 * 
 * Note: Place the dll in the Assets/ directory and NOT Assets/Plugins
 */

using System;
using System.Runtime.InteropServices;
using UnityEngine;

public static class Native
{
    
    public static T Invoke<T, T2>(IntPtr library, params object[] pars)
    {
        IntPtr funcPtr = GetProcAddress(library, typeof(T2).Name);
        if (funcPtr == IntPtr.Zero)
        {
            Debug.LogWarning("Could not gain reference to method address.");
            return default(T);
        }

        var func = Marshal.GetDelegateForFunctionPointer(GetProcAddress(library, typeof(T2).Name), typeof(T2));
        return (T)func.DynamicInvoke(pars);
    }

    public static void Invoke<T>(IntPtr library, params object[] pars)
    {
        IntPtr funcPtr = GetProcAddress(library, typeof(T).Name);
        if (funcPtr == IntPtr.Zero)
        {
            Debug.LogWarning("Could not gain reference to method address.");
            return;
        }

        var func = Marshal.GetDelegateForFunctionPointer(funcPtr, typeof(T));
        func.DynamicInvoke(pars);
    }

    public static Delegate GetFuncPtr<T, T2>(IntPtr library)
    {
        IntPtr funcPtr = GetProcAddress(library, typeof(T2).Name);
        if (funcPtr == IntPtr.Zero)
        {
            Debug.LogWarning("Could not gain reference to method address.");
            return null;
        }

        var func = Marshal.GetDelegateForFunctionPointer(GetProcAddress(library, typeof(T2).Name), typeof(T2));
        return func;
    }

    public static Delegate GetFuncPtr<T>(IntPtr library)
    {
        IntPtr funcPtr = GetProcAddress(library, typeof(T).Name);
        if (funcPtr == IntPtr.Zero)
        {
            Debug.LogWarning("Could not gain reference to method address.");
            return null;
        }

        var func = Marshal.GetDelegateForFunctionPointer(GetProcAddress(library, typeof(T).Name), typeof(T));
        return func;
    }


    public static T InvokeFast<T>(Delegate d, params object[] pars)
    {
        return (T)d.DynamicInvoke(pars);
    }

    public static void InvokeFast(Delegate d, params object[] pars)
    {
        d.DynamicInvoke(pars);
    }


    [DllImport("kernel32", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool FreeLibrary(IntPtr hModule);

    [DllImport("kernel32", SetLastError = true, CharSet = CharSet.Unicode)]
    public static extern IntPtr LoadLibrary(string lpFileName);

    [DllImport("kernel32")]
    public static extern IntPtr GetProcAddress(IntPtr hModule, string procedureName);
}