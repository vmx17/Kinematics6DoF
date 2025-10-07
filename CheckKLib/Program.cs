using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using KinematicsRM2; // Use Manipulator6DoF (namespace changed version)

internal class Program
{
    // Forward() verbosity: 0 none, 1 origins, 2 origins + axes
    private const int ForwardVerbose = 2;

    // Always call InverseGeometric with verbose=2 to show target pose / Pw etc.
    private const int GeometricVerbose = 2;

    private const bool DumpEachGeometricSolutionFrames = true;
    private const bool DumpJacobianFinalFrames = true;
    private const bool CompareGeometricWithJacobian = true;

    // Match tolerances (joint RMS deg / position mm / orientation deg) for asserting that
    // at least one geometric solution matches the Jacobian solution.
    private const double JointMatchRmsDegTol = 0.05;    // very tight: adjust if needed
    private const double PosMatchTol = 1e-3;            // same unit as FK (mm if inputs are mm)
    private const double OriMatchDegTol = 0.05;         // orientation difference
    internal static readonly double[] item = new double[]{0.1,0.2,0.3,0.4,0.5,0.6};

    public static void Main()
    {
        Console.WriteLine("--- KinematicsRM2 Test (Forward / Geometric IK / Jacobian IK) ---");

        var testCases = new List<(string name, double[] q)>
        {
            ("ZeroPose",    new double[]{0,0,0,0,0,0}),
            ("GeneralPose", item)
        };

        foreach (var (label, qRef) in testCases)
            RunCase(label, qRef);

        Console.WriteLine("\n--- Test End ---");
    }

    private static void RunCase(string label, double[] qRef)
    {
        Console.WriteLine($"\n=== {label} ===");
        Console.WriteLine($"Reference joints (deg): [{FormatJointAngles(qRef)}]");

        var robot = ManipulatorFactory.CreateManipulator("MiRobot", "null");

        // Forward target pose
        var T_target = robot.Forward(qRef, ForwardVerbose);
        Console.WriteLine("Target Pose (Forward result):");
        PrintMatrix(T_target);
        var (axis, angle) = GetAxisAngle(T_target);
        Console.WriteLine($"Axis-Angle: axis=({axis.X:F4},{axis.Y:F4},{axis.Z:F4}) angle={angle * 180 / Math.PI:F4} deg");

        // Geometric IK
        Console.WriteLine("\n-- Geometric IK (InverseGeometric) --");
        bool[] geoFlags;
        var geoSols = robot.InverseGeometric(T_target, out geoFlags, verbose: GeometricVerbose);

        var valid = geoFlags.Count(f => f);
        Console.WriteLine($"Geometric IK valid solutions: {valid}/{geoSols.Length}");
        for (var i = 0; i < geoSols.Length; i++)
        {
            if (!geoFlags[i]) continue;
            var qSol = geoSols[i];
            Console.WriteLine($"  GeoSol {i + 1}: [{FormatJointAngles(qSol)}]");
            var T_sol = robot.Forward(qSol, DumpEachGeometricSolutionFrames ? ForwardVerbose : 0);
            var ok = CompareMatrices(T_target, T_sol);
            Console.WriteLine(ok ? "    Pose Verification: SUCCESS" : "    Pose Verification: FAILED");
            if (!ok)
            {
                Console.WriteLine("    Recomputed Pose:");
                PrintMatrix(T_sol);
            }
        }

        // Jacobian IK
        Console.WriteLine("\n-- Jacobian IK (InverseJacobian) --");
        var qInit = (double[])qRef.Clone(); // seed with reference
        bool jacSuccess;
        var qJac = robot.InverseJacobian(
            T_target,
            qInit,
            out jacSuccess,
            maxIterations: 800,
            tolerance: 1e-8,
            alpha: 0.1,
            verbose: GeometricVerbose);

        if (jacSuccess)
        {
            Console.WriteLine($"Jacobian converged: [{FormatJointAngles(qJac)}]");
            var T_jac = robot.Forward(qJac, DumpJacobianFinalFrames ? ForwardVerbose : 0);
            Console.WriteLine("Jacobian Re-Forward:");
            PrintMatrix(T_jac);
            Console.WriteLine(CompareMatrices(T_target, T_jac)
                ? "    Pose Verification: SUCCESS"
                : "    Pose Verification: FAILED");
        }
        else
        {
            Console.WriteLine("Jacobian IK did NOT converge.");
        }

        // Geometric vs Jacobian Comparison + Match Assertion
        if (CompareGeometricWithJacobian && jacSuccess && valid > 0)
        {
            Console.WriteLine("\n-- Geometric vs Jacobian Comparison --");
            var anyMatch = false;
            var bestRms = double.MaxValue;
            var bestIndex = -1;

            for (var i = 0; i < geoSols.Length; i++)
            {
                if (!geoFlags[i]) continue;
                var (jointRmsDeg, posErr, oriErrDeg) = EvaluateDiff(geoSols[i], qJac, robot);
                Console.WriteLine(
                    $"  GeoSol {i + 1}: JointRMS={jointRmsDeg:F4} deg  PosErr={posErr:F6}  OriErr={oriErrDeg:F4} deg");

                if (jointRmsDeg < bestRms)
                {
                    bestRms = jointRmsDeg;
                    bestIndex = i;
                }

                if (jointRmsDeg <= JointMatchRmsDegTol &&
                    posErr <= PosMatchTol &&
                    oriErrDeg <= OriMatchDegTol)
                {
                    anyMatch = true;
                }
            }

            Console.WriteLine(anyMatch
                ? $"[MATCH] A geometric solution matches the Jacobian result within thresholds (JointRMS<={JointMatchRmsDegTol} deg, Pos<={PosMatchTol}, Ori<={OriMatchDegTol} deg)."
                : "[NO MATCH] No geometric solution met all joint/pose/orientation thresholds against Jacobian result.");

            if (!anyMatch && bestIndex >= 0)
            {
                Console.WriteLine($"Closest geometric solution index: {bestIndex + 1} (JointRMS={bestRms:F4} deg)");
            }
        }
        else if (jacSuccess && valid == 0)
        {
            Console.WriteLine("[INFO] No geometric solutions to compare with Jacobian result.");
        }
    }

    // Difference metrics between two joint solutions
    private static (double jointRmsDeg, double posErr, double oriErrDeg) EvaluateDiff(
        double[] qA, double[] qB, Manipulator6DoF robot)
    {
        double sumSq = 0;
        for (var i = 0; i < 6; i++)
        {
            var d = NormalizeAngle(qA[i] - qB[i]);
            sumSq += d * d;
        }
        var jointRmsDeg = Math.Sqrt(sumSq / 6.0) * 180.0 / Math.PI;

        var TA = robot.Forward(qA, 0);
        var TB = robot.Forward(qB, 0);

        var dx = TA[0, 3] - TB[0, 3];
        var dy = TA[1, 3] - TB[1, 3];
        var dz = TA[2, 3] - TB[2, 3];
        var posErr = Math.Sqrt(dx * dx + dy * dy + dz * dz);

        var tr =
            TA[0, 0] * TB[0, 0] + TA[1, 0] * TB[1, 0] + TA[2, 0] * TB[2, 0] +
            TA[0, 1] * TB[0, 1] + TA[1, 1] * TB[1, 1] + TA[2, 1] * TB[2, 1] +
            TA[0, 2] * TB[0, 2] + TA[1, 2] * TB[1, 2] + TA[2, 2] * TB[2, 2];

        var cosAng = (tr - 1.0) * 0.5;
        cosAng = Math.Clamp(cosAng, -1.0, 1.0);
        var oriErrDeg = Math.Acos(cosAng) * 180.0 / Math.PI;

        return (jointRmsDeg, posErr, oriErrDeg);
    }

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // Utilities
    public static void PrintMatrix(double[,] m)
    {
        for (var r = 0; r < 4; r++)
        {
            Console.Write("[");
            for (var c = 0; c < 4; c++)
            {
                Console.Write(c < 3 ? $"{m[r, c]:F6}" : $"{m[r, c]:F3}");
                if (c < 3) Console.Write(", ");
            }
            Console.WriteLine("]");
        }
    }

    public static bool CompareMatrices(double[,] A, double[,] B, double tolerance = 1e-5)
    {
        if (A.GetLength(0) != 4 || A.GetLength(1) != 4 || B.GetLength(0) != 4 || B.GetLength(1) != 4)
            return false;
        for (var r = 0; r < 4; r++)
            for (var c = 0; c < 4; c++)
                if (Math.Abs(A[r, c] - B[r, c]) > tolerance) return false;
        return true;
    }

    public static string FormatJointAngles(double[] anglesRad) =>
        string.Join(", ", anglesRad.Select(a => (a * 180.0 / Math.PI).ToString("F4")));

    private static (Vector3 axis, double angle) GetAxisAngle(double[,] T)
    {
        double r00 = T[0, 0], r01 = T[0, 1], r02 = T[0, 2];
        double r10 = T[1, 0], r11 = T[1, 1], r12 = T[1, 2];
        double r20 = T[2, 0], r21 = T[2, 1], r22 = T[2, 2];
        var trace = r00 + r11 + r22;
        var cosAng = Math.Clamp((trace - 1.0) * 0.5, -1.0, 1.0);
        var angle = Math.Acos(cosAng);
        if (angle < 1e-9)
            return (new Vector3(1, 0, 0), 0);
        var denom = 2 * Math.Sin(angle);
        var ax = (r21 - r12) / denom;
        var ay = (r02 - r20) / denom;
        var az = (r10 - r01) / denom;
        return (new Vector3((float)ax, (float)ay, (float)az), angle);
    }
}
