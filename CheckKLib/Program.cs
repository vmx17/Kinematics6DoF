using System;
using System.Globalization;
using KinematicsGPT5; // Reference to your kinematics library

public class Program
{
    public static void Main(string[] args)
    {
        // Set culture for consistent decimal formatting
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        var kin = new KinematicsSolver();

        Console.WriteLine("=============================================");
        Console.WriteLine("    KinematicsGPT5 Library Test");
        Console.WriteLine("=============================================");

        // Initial pose: all joint angles zero
        double[] q_basic = { 0, 0, 0, 0, 0, 0 };
        double[] tool = { 0, 0, 0, 0, 0, 0 };
        double[] mount = { 0, 0, 0, 0, 0, 0 };

        // --- 1. Forward Kinematics (FK) Test ---
        Console.WriteLine("--- 1. Forward Kinematics (FK) Test ---");
        // verbose=2 to print matrices for inspection
        double[,] T_fk = kin.Forward(q_basic, tool, mount, verbose: 1);

        Console.WriteLine("\n  Input Angles:");
        Console.WriteLine($"  {FormatAngles(q_basic)}");
        Console.WriteLine("\n  FK Result (TCP):");
        Console.WriteLine($"  {FormatPose(T_fk)}");

        // Do not compare TCP to (0,0,0) here — that check is meaningless for FK.
        // If you want to inspect intermediate origins (O1..O6) call FK with verbose=1:
        Console.WriteLine("\n  (Intermediate origins printed above by verbose=2; verbose=1 would print origins only.)");

        // --- 2. Inverse Kinematics (IK) Test ---
        Console.WriteLine("\n--- 2. Inverse Kinematics (IK) Test ---");
        Console.WriteLine("[2.1] Target pose from FK:");
        Console.WriteLine($"  {FormatPose(T_fk)}");

        // IK1: Geometric approach
        Console.WriteLine("\n[2.2] InverseGeometric IK...");
        double[][] ik_geo_solutions = kin.InverseGeometric(T_fk, mount, tool, verbose: 1);

        if (ik_geo_solutions == null || ik_geo_solutions.Length == 0)
        {
            Console.WriteLine("  [Geo IK] No solutions returned.");
        }
        else
        {
            Console.WriteLine($"  [Geo IK] {ik_geo_solutions.Length} solution(s) returned. Verifying by FK:");
            for (int i = 0; i < ik_geo_solutions.Length; i++)
            {
                double[] qg = ik_geo_solutions[i];
                double[,] T_from_geo = kin.Forward(qg, tool, mount, verbose: 0);
                bool match = IsPoseEqual(T_from_geo, T_fk, tolPos: 1e-3, tolOri: 1e-3);
                Console.WriteLine($"    Solution #{i + 1}: {FormatAngles(qg)} -> FK matches target: {(match ? "OK" : "NG")}");
            }
        }

        bool foundJacobianInGeo = false;
        double[] ik_jac_solution = null;
        bool jacobianSuccess = false;

        // IK2: Jacobian approach
        Console.WriteLine("\n[2.3] InverseJacobian IK...");
        ik_jac_solution = kin.InverseJacobian(T_fk, q_basic, mount, tool, out jacobianSuccess, verbose: 1);

        if (!jacobianSuccess || ik_jac_solution == null)
        {
            Console.WriteLine("  [Jacobian IK] Solver did not converge or returned no solution.");
        }
        else
        {
            Console.WriteLine("  [Jacobian IK] Solution (verified by FK):");
            Console.WriteLine($"  {FormatAngles(ik_jac_solution)}");
            double[,] T_from_jac = kin.Forward(ik_jac_solution, tool, mount, verbose: 0);
            bool jacMatch = IsPoseEqual(T_from_jac, T_fk, tolPos: 1e-3, tolOri: 1e-3);
            Console.WriteLine($"  FK( JacobianSolution ) matches target: {(jacMatch ? "OK" : "NG")}");
        }

        // Check if Jacobian solution is contained in geometric solutions
        if (ik_geo_solutions != null && jacobianSuccess)
        {
            foreach (var geo in ik_geo_solutions)
            {
                if (IsAnglesEqual(geo, ik_jac_solution))
                {
                    foundJacobianInGeo = true;
                    break;
                }
            }
        }

        Console.WriteLine($"\n  [Check] Jacobian IK solution is in Geometric IK solutions: {(foundJacobianInGeo ? "OK" : "NG")}");
        Console.WriteLine();
    }

    // --- Helper functions ---

    static string FormatAngles(double[] angles)
    {
        if (angles == null) return "(null)";
        return string.Join(", ", Array.ConvertAll(angles, x => $"{x:F4}"));
    }

    static string FormatPose(double[,] T)
    {
        if (T == null) return "(null)";
        return $"[{T[0, 3]:F3}, {T[1, 3]:F3}, {T[2, 3]:F3}]";
    }

    static bool IsPositionEqual(double[] a, double[] b, double tol = 1e-3)
    {
        if (a == null || b == null || a.Length != b.Length) return false;
        for (int i = 0; i < a.Length; i++)
            if (Math.Abs(a[i] - b[i]) > tol) return false;
        return true;
    }

    static bool IsAnglesEqual(double[] a, double[] b, double tol = 1e-4)
    {
        if (a == null || b == null || a.Length != b.Length) return false;
        for (int i = 0; i < a.Length; i++)
            if (Math.Abs(a[i] - b[i]) > tol) return false;
        return true;
    }

    // Compare full pose (position + simple orientation using z-axis) within tolerances
    static bool IsPoseEqual(double[,] A, double[,] B, double tolPos = 1e-3, double tolOri = 1e-3)
    {
        if (A == null || B == null) return false;
        // position
        for (int i = 0; i < 3; i++)
            if (Math.Abs(A[i, 3] - B[i, 3]) > tolPos) return false;
        // simple orientation check: compare z-axis column
        for (int i = 0; i < 3; i++)
            if (Math.Abs(A[i, 2] - B[i, 2]) > tolOri) return false;
        return true;
    }
}