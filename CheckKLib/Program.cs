using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using KinematicsRM2; // Manipulator6DoF

internal class Program
{
    // Verbosity
    private const int ForwardVerbose = 0;        // Set to 2 for full axes dump
    private const int GeometricVerbose = 0;
    private const bool DumpJacobianFinalFrames = false;

    // Match tolerances
    private const double JointMatchRmsDegTol = 0.05;
    private const double PosMatchTol = 1e-3;
    private const double OriMatchDegTol = 0.05;

    // Test joints
    internal static readonly double[] GeneralPose = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

    public static void Main()
    {
        Console.WriteLine("--- KinematicsRM2 Tool Integration Test (Flange vs TCP) ---");

        var testCases = new List<(string Label, double[] q)>
        {
            ("ZeroPose", new double[]{0,0,0,0,0,0}),
            ("GeneralPose", GeneralPose)
        };

        // Tools to test (null = no offset, tool0 = offset + tilt)
        var tools = new[] { "null", "tool0" };
        double?[] rollOverrides = { null, Math.PI / 6 }; // test without & with roll override (for stored tool only)

        foreach (var tool in tools)
        {
            Console.WriteLine($"\n=== Tool: {tool} ===");
            foreach (var (label, q) in testCases)
            {
                Console.WriteLine($"\n--- Case: {label} ---");
                RunFlangeOnlyCycle(q, tool);
                RunTcpCycles(q, tool, rollOverrides);
            }
        }

        Console.WriteLine("\n--- Test End ---");
    }

    // 1. Original (flange) FK + Jacobian IK (baseline, tool not in target)
    private static void RunFlangeOnlyCycle(double[] qRef, string toolName)
    {
        var robot = ManipulatorFactory.CreateManipulator("MiRobot", toolName);

        // Flange pose target
        var T_flange_target = robot.Forward(qRef, ForwardVerbose);
        Console.WriteLine("Flange Target (Forward):");
        PrintMatrix(T_flange_target);

        // Jacobian IK (flange)
        bool jacSuccess;
        var qInit = (double[])qRef.Clone();
        var qJac = robot.InverseJacobian(
            T_flange_target,
            qInit,
            out jacSuccess,
            maxIterations: 600,
            tolerance: 1e-8,
            alpha: 0.1,
            verbose: GeometricVerbose);

        if (!jacSuccess)
        {
            Console.WriteLine("  [Flange IK] Jacobian failed to converge.");
            return;
        }

        var T_flange_re = robot.Forward(qJac, 0);
        var ok = CompareMatrices(T_flange_target, T_flange_re);
        Console.WriteLine(ok ? "  [Flange IK] SUCCESS (pose match)" : "  [Flange IK] FAILED (pose mismatch)");

        if (!ok)
        {
            Console.WriteLine("  Recomputed:");
            PrintMatrix(T_flange_re);
        }

        var (jointRms, posErr, oriErr) = EvaluateJointAndPoseDifferences(qRef, qJac, robot, flangeOnly: true);
        Console.WriteLine($"  Diff Flange IK vs Ref: JointRMS={jointRms:F4}deg PosErr={posErr:E3} OriErr={oriErr:F4}deg");
    }

    // 2. TCP (tool applied) FK + Jacobian IK using TCP wrappers (requires patched methods)
    private static void RunTcpCycles(double[] qRef, string toolName, double?[] rollOverrides)
    {
        var robot = ManipulatorFactory.CreateManipulator("MiRobot", toolName);

        foreach (var roll in rollOverrides)
        {
            Console.WriteLine(roll.HasValue
                ? $"\n  --- TCP Cycle (roll override = {roll.Value * 180.0 / Math.PI:F2} deg) ---"
                : "\n  --- TCP Cycle (no roll override) ---");

            // TCP target pose
            var T_tcp_target = robot.ForwardWithTool(qRef, roll, ForwardVerbose);
            Console.WriteLine("  TCP Target (ForwardWithTool):");
            PrintMatrix(T_tcp_target);

            // Verify manual composition: Forward * Tool == ForwardWithTool
            var T_flange = robot.Forward(qRef, 0);
            var T_tool = robot.GetToolTransform(roll);
            var T_composed = MatMul4x4(T_flange, T_tool);
            if (!CompareMatrices(T_tcp_target, T_composed))
                Console.WriteLine("  [WARN] ForwardWithTool != Forward*ToolTransform (check tool integration)");

            // Jacobian TCP IK (requires InverseJacobianTcp)
            bool jacSuccessTcp;
            var qInit = (double[])qRef.Clone();
            double[] qJacTcp;
            try
            {
                qJacTcp = robot.InverseJacobianTcp(
                    T_tcp_target,
                    qInit,
                    out jacSuccessTcp,
                    rollOverride: roll,
                    maxIterations: 800,
                    tolerance: 1e-8,
                    alpha: 0.1,
                    verbose: GeometricVerbose);
            }
            catch (MissingMethodException)
            {
                Console.WriteLine("  [SKIP] InverseJacobianTcp not implemented in current code.");
                return;
            }

            if (!jacSuccessTcp)
            {
                Console.WriteLine("  [TCP IK] Jacobian failed to converge.");
                continue;
            }

            var T_tcp_re = robot.ForwardWithTool(qJacTcp, roll, 0);
            var tcpOk = CompareMatrices(T_tcp_target, T_tcp_re);
            Console.WriteLine(tcpOk ? "  [TCP IK] SUCCESS (pose match)" : "  [TCP IK] FAILED (pose mismatch)");

            if (!tcpOk)
            {
                Console.WriteLine("  Recomputed TCP:");
                PrintMatrix(T_tcp_re);
            }

            var (jointRms, posErr, oriErr) = EvaluateJointAndPoseDifferences(qRef, qJacTcp, robot, flangeOnly: false, rollOverride: roll);
            Console.WriteLine($"  Diff TCP IK vs Ref: JointRMS={jointRms:F4}deg PosErr={posErr:E3} OriErr={oriErr:F4}deg");
        }
    }

    private static (double jointRmsDeg, double posErr, double oriErrDeg) EvaluateJointAndPoseDifferences(
        double[] qRef, double[] qSol, Manipulator6DoF robot, bool flangeOnly, double? rollOverride = null)
    {
        double sumSq = 0;
        for (var i = 0; i < 6; i++)
        {
            var d = NormalizeAngle(qRef[i] - qSol[i]);
            sumSq += d * d;
        }
        var jointRmsDeg = Math.Sqrt(sumSq / 6.0) * 180.0 / Math.PI;

        var Tref = flangeOnly ? robot.Forward(qRef, 0) : robot.ForwardWithTool(qRef, rollOverride, 0);
        var Tsol = flangeOnly ? robot.Forward(qSol, 0) : robot.ForwardWithTool(qSol, rollOverride, 0);

        var dx = Tref[0, 3] - Tsol[0, 3];
        var dy = Tref[1, 3] - Tsol[1, 3];
        var dz = Tref[2, 3] - Tsol[2, 3];
        var posErr = Math.Sqrt(dx * dx + dy * dy + dz * dz);

        var tr =
            Tref[0, 0] * Tsol[0, 0] + Tref[1, 0] * Tsol[1, 0] + Tref[2, 0] * Tsol[2, 0] +
            Tref[0, 1] * Tsol[0, 1] + Tref[1, 1] * Tsol[1, 1] + Tref[2, 1] * Tsol[2, 1] +
            Tref[0, 2] * Tsol[0, 2] + Tref[1, 2] * Tsol[1, 2] + Tref[2, 2] * Tsol[2, 2];

        var cosAng = (tr - 1.0) * 0.5;
        cosAng = Math.Clamp(cosAng, -1.0, 1.0);
        var oriErrDeg = Math.Acos(cosAng) * 180.0 / Math.PI;
        return (jointRmsDeg, posErr, oriErrDeg);
    }

    // --- Helpers reused from original ---

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

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // Local 4x4 mul (since RMLib is internal)
    private static double[,] MatMul4x4(double[,] A, double[,] B)
    {
        var C = new double[4, 4];
        for (var i = 0; i < 4; i++)
            for (var j = 0; j < 4; j++)
                for (var k = 0; k < 4; k++)
                    C[i, j] += A[i, k] * B[k, j];
        return C;
    }
}
