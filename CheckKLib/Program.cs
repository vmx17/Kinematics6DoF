using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KinematicsRM2; // Manipulator6DoF
using RowMeasureUtility; // RMLib (必要なら既に存在していれば重複可)

internal class Program
{
    // Robot under test (MiRobot2 only)
    private const string RobotName = "MiRobot2";

    // Verbosity (Forward: 0 none, 1 origins, 2 origins+axes)
    private const int ForwardVerbose = 0;
    private const int IKVerbose = 0;

    // Tolerances
    private const double PosTol = 1e-4;          // positional match tolerance (m)
    private const double OriTolDeg = 0.01;       // orientation match tolerance (deg)
    private const double JointRmsDegTol = 0.05;  // joint RMS (deg) tolerance for convergence validation

    // Test joint sets
    private static readonly double[] ZeroPose = { 0, 0, 0, 0, 0, 0 };
    private static readonly double[] GeneralPose = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

    // Optional more samples (enable if needed)
    private static IEnumerable<double[]> AdditionalSamples()
    {
        yield return new[] { 0.0, -0.3, 0.5, -0.2, 0.3, 0.7 };
        yield return new[] { -0.5, 0.4, -0.2, 0.8, -0.6, 1.0 };
    }

    public static void Main()
    {
        Console.WriteLine($"=== MiRobot2 FK / IK Consistency Check ===");

        var toolNames = new[] { "null", "tool0" };
        double?[] rollOverrides = { null, Math.PI / 6 };

        var testCases = new List<(string Label, double[] q)>
        {
            ("ZeroPose", ZeroPose),
            ("GeneralPose", GeneralPose)
        };
        testCases.AddRange(AdditionalSamples().Select((q,i) => ($"Extra{i+1}", q)));

        foreach (var tool in toolNames)
        {
            Console.WriteLine($"\n--- Tool: {tool} ---");
            foreach (var (label, qRef) in testCases)
            {
                Console.WriteLine($"\n[{label}]  q(deg)= {FormatJointAngles(qRef)}");
                RunFlangeConsistency(qRef, tool);
                RunTcpConsistency(qRef, tool, rollOverrides);
            }
        }

        Console.WriteLine("\n=== Test End ===");
    }

    // ========== Flange (no tool) consistency: Forward -> InverseGeometric & InverseJacobian ==========
    private static void RunFlangeConsistency(double[] qRef, string toolName)
    {
        var robot = ManipulatorFactory.CreateManipulator(RobotName, toolName);

        // Forward target
        var T_target = robot.Forward(qRef, ForwardVerbose);
        Console.WriteLine("  Flange Forward Pose:");
        PrintMatrix(T_target);

        // Geometric IK (all solutions)
        bool[] geoFlags;
        double[][] geoSolutions;
        try
        {
            geoSolutions = robot.InverseGeometric(T_target, out geoFlags, verbose: IKVerbose);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"  [Geometric IK ERROR] {ex.Message}");
            geoSolutions = Array.Empty<double[]>();
        }

        if (geoSolutions.Length == 0)
        {
            Console.WriteLine("  Geometric IK: NO solutions.");
        }
        else
        {
            Console.WriteLine($"  Geometric IK: {geoSolutions.Length} solution(s).");
            // Pick best (closest in joint RMS to reference) just for reporting
            var best = geoSolutions
                .Select(s => (sol: s, rms: JointRmsDeg(qRef, s)))
                .OrderBy(t => t.rms)
                .First();

            foreach (var (sol, idx) in geoSolutions.Select((s,i)=>(s,i)))
            {
                var (posErr, oriErrDeg) = PoseErrors(robot.Forward(sol, 0), T_target);
                var rms = JointRmsDeg(qRef, sol);
                Console.WriteLine($"    [{idx}] JointRMS={rms:F4}deg  PosErr={posErr:E3}  OriErr={oriErrDeg:F4}deg");
            }
            Console.WriteLine($"  -> Best Geometric solution RMS={best.rms:F4}deg");
            CheckTolerance("  Geometric IK pose match", robot.Forward(best.sol,0), T_target);
        }

        // Jacobian IK
        bool jacSuccess;
        var qJac = robot.InverseJacobian(
            T_target,
            q_initial: (double[])qRef.Clone(),
            out jacSuccess,
            maxIterations: 800,
            tolerance: 1e-10,
            alpha: 0.1,
            verbose: IKVerbose);

        if (!jacSuccess)
        {
            Console.WriteLine("  Jacobian IK: NOT converged.");
        }
        else
        {
            var T_re = robot.Forward(qJac, 0);
            var (posErr, oriErrDeg) = PoseErrors(T_re, T_target);
            var jointRms = JointRmsDeg(qRef, qJac);
            Console.WriteLine($"  Jacobian IK: JointRMS={jointRms:F4}deg PosErr={posErr:E3} OriErr={oriErrDeg:F4}deg");
            CheckTolerance("  Jacobian IK pose match", T_re, T_target);
        }

        if (ShowInternalIKDebug)
            DebugGeometricInternals(robot, T_target, geoSolutions);
    }

    // ========== TCP consistency (with tool and optional roll override) ==========
    private static void RunTcpConsistency(double[] qRef, string toolName, double?[] rollOverrides)
    {
        var robot = ManipulatorFactory.CreateManipulator(RobotName, toolName);

        foreach (var roll in rollOverrides)
        {
            Console.WriteLine(roll.HasValue
                ? $"  [TCP] roll override = {roll.Value * 180.0 / Math.PI:F2} deg"
                : "  [TCP] no roll override");

            var T_tcp_target = robot.ForwardWithTool(qRef, roll, ForwardVerbose);
            Console.WriteLine("    TCP Forward Pose:");
            PrintMatrix(T_tcp_target);

            // Geometric TCP IK
            bool[] flagsTcp;
            double[][] geoTcp;
            try
            {
                geoTcp = robot.InverseGeometricTcp(T_tcp_target, out flagsTcp, roll, verbose: IKVerbose);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"    GeometricTCP IK ERROR: {ex.Message}");
                geoTcp = Array.Empty<double[]>();
            }

            if (geoTcp.Length == 0)
            {
                Console.WriteLine("    GeometricTCP IK: NO solutions.");
            }
            else
            {
                Console.WriteLine($"    GeometricTCP IK: {geoTcp.Length} solution(s).");
                var best = geoTcp
                    .Select(s => (sol: s, rms: JointRmsDeg(qRef, s)))
                    .OrderBy(t => t.rms)
                    .First();

                foreach (var (sol, idx) in geoTcp.Select((s,i)=>(s,i)))
                {
                    var (posErr, oriErrDeg) = PoseErrors(robot.ForwardWithTool(sol, roll, 0), T_tcp_target);
                    var rms = JointRmsDeg(qRef, sol);
                    Console.WriteLine($"      [{idx}] JointRMS={rms:F4}deg PosErr={posErr:E3} OriErr={oriErrDeg:F4}deg");
                }
                CheckTolerance("    GeometricTCP pose match (best)", robot.ForwardWithTool(best.sol, roll, 0), T_tcp_target);
            }

            // Jacobian TCP IK
            bool jacTcpSuccess;
            double[] qJacTcp;
            try
            {
                qJacTcp = robot.InverseJacobianTcp(
                    T_tcp_target,
                    (double[])qRef.Clone(),
                    out jacTcpSuccess,
                    rollOverride: roll,
                    maxIterations: 900,
                    tolerance: 1e-10,
                    alpha: 0.1,
                    verbose: IKVerbose);
            }
            catch (MissingMethodException)
            {
                Console.WriteLine("    JacobianTCP IK: Not implemented.");
                continue;
            }

            if (!jacTcpSuccess)
            {
                Console.WriteLine("    JacobianTCP IK: NOT converged.");
            }
            else
            {
                var T_tcp_re = robot.ForwardWithTool(qJacTcp, roll, 0);
                var (posErr, oriErrDeg) = PoseErrors(T_tcp_re, T_tcp_target);
                var jointRms = JointRmsDeg(qRef, qJacTcp);
                Console.WriteLine($"    JacobianTCP IK: JointRMS={jointRms:F4}deg PosErr={posErr:E3} OriErr={oriErrDeg:F4}deg");
                CheckTolerance("    JacobianTCP pose match", T_tcp_re, T_tcp_target);
            }
        }
    }

    // ===== Evaluation Helpers =====
    private static double JointRmsDeg(double[] qRef, double[] q)
    {
        double sum = 0;
        for (int i = 0; i < 6; i++)
        {
            var d = NormalizeAngle(qRef[i] - q[i]);
            sum += d * d;
        }
        return Math.Sqrt(sum / 6.0) * 180.0 / Math.PI;
    }

    private static (double posErr, double oriErrDeg) PoseErrors(double[,] T, double[,] Ttarget)
    {
        var dx = T[0, 3] - Ttarget[0, 3];
        var dy = T[1, 3] - Ttarget[1, 3];
        var dz = T[2, 3] - Ttarget[2, 3];
        var posErr = Math.Sqrt(dx * dx + dy * dy + dz * dz);

        var tr =
            T[0, 0] * Ttarget[0, 0] + T[1, 0] * Ttarget[1, 0] + T[2, 0] * Ttarget[2, 0] +
            T[0, 1] * Ttarget[0, 1] + T[1, 1] * Ttarget[1, 1] + T[2, 1] * Ttarget[2, 1] +
            T[0, 2] * Ttarget[0, 2] + T[1, 2] * Ttarget[1, 2] + T[2, 2] * Ttarget[2, 2];

        var cosAng = (tr - 1.0) * 0.5;
        cosAng = Math.Clamp(cosAng, -1.0, 1.0);
        var oriErrDeg = Math.Acos(cosAng) * 180.0 / Math.PI;
        return (posErr, oriErrDeg);
    }

    private static void CheckTolerance(string label, double[,] T, double[,] Ttarget)
    {
        var (pErr, oErr) = PoseErrors(T, Ttarget);
        var okPos = pErr <= PosTol;
        var okOri = oErr <= OriTolDeg;
        Console.WriteLine($"{label}: {(okPos && okOri ? "OK" : "NG")} (Pos={pErr:E3} tol={PosTol}, Ori={oErr:F4}deg tol={OriTolDeg}deg)");
    }

    // ===== Formatting & math helpers =====
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

    public static string FormatJointAngles(double[] anglesRad) =>
        string.Join(", ", anglesRad.Select(a => (a * 180.0 / Math.PI).ToString("F3")));

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // 追加: 内部 IK デバッグ表示トグル
    private const bool ShowInternalIKDebug = true;

    private static void DebugGeometricInternals(Manipulator6DoF robot, double[,] T_target, double[][] sols)
    {
        try
        {
            if (sols.Length == 0)
            {
                Console.WriteLine("  [GeoDbg] No geometric solutions to inspect.");
                return;
            }

            // 反射で MDH を取得
            var mdhField = typeof(Manipulator6DoF)
                .GetField("_mdh", BindingFlags.NonPublic | BindingFlags.Instance);
            if (mdhField == null)
            {
                Console.WriteLine("  [GeoDbg] _mdh field not found.");
                return;
            }
            var mdh = (dynamic)mdhField.GetValue(robot);
            double[] A = (double[])mdh.A;
            double[] D = (double[])mdh.D;
            double[] Offset = (double[])mdh.Offset;
            double[] Alpha = (double[])mdh.Alpha;

            double a3 = A[2];
            double a4 = A[3];
            double d4 = D[3];
            double d6 = D[5];

            double Lw = Math.Sqrt(a4 * a4 + d4 * d4);
            double psi = Math.Atan2(d4, a4);

            // 目標フランジ→手首中心
            double ZX = T_target[0, 2], ZY = T_target[1, 2], ZZ = T_target[2, 2];
            double Px = T_target[0, 3], Py = T_target[1, 3], Pz = T_target[2, 3];
            double PwX = Px - d6 * ZX;
            double PwY = Py - d6 * ZY;
            double PwZ = Pz - d6 * ZZ;

            Console.WriteLine($"  [GeoDbg] a3={a3:F3} a4={a4:F3} d4={d4:F3}  Lw={Lw:F6} psi={psi*180/Math.PI:F4}deg");
            Console.WriteLine($"  [GeoDbg] Pw=({PwX:F3},{PwY:F3},{PwZ:F3})");

            for (int idx = 0; idx < sols.Length; idx++)
            {
                var q = sols[idx];
                double q1 = q[0] + Offset[0];
                double q2 = q[1] + Offset[1];
                double q3 = q[2] + Offset[2];

                // T01, T12 計算（ComputeW2 の再現）
                var T01 = RMLib.DHTransform(q1, D[0], A[0], Alpha[0]);
                var T12 = RMLib.DHTransform(q2, D[1], A[1], Alpha[1]);
                var T02 = RMLib.MatMul4x4(T01, T12);

                double O2x = T02[0, 3], O2y = T02[1, 3], O2z = T02[2, 3];
                double[] X2 = { T02[0, 0], T02[1, 0], T02[2, 0] };
                double[] Y2 = { T02[0, 1], T02[1, 1], T02[2, 1] };
                double[] Z2 = { T02[0, 2], T02[1, 2], T02[2, 2] };

                double Wx = PwX - O2x;
                double Wy = PwY - O2y;
                double Wz = PwZ - O2z;

                double vx = Wx * X2[0] + Wy * X2[1] + Wz * X2[2];
                double vy = Wx * Y2[0] + Wy * Y2[1] + Wz * Y2[2];
                double vz = Wx * Z2[0] + Wy * Z2[1] + Wz * Z2[2];

                double dx = vx - a3;
                double g = dx * dx + vy * vy - Lw * Lw;
                double q3_eff = q3 + psi; // q3 = q3_eff - psi なので逆

                // 再構成された“理論円上”期待値
                double vx_theory = a3 + Lw * Math.Cos(q3_eff);
                double vy_theory = Lw * Math.Sin(q3_eff);
                double evx = vx - vx_theory;
                double evy = vy - vy_theory;

                Console.WriteLine(
                    $"    [GeoDbg sol {idx}] q(deg)=[{q[0]*180/Math.PI:F2},{q[1]*180/Math.PI:F2},{q[2]*180/Math.PI:F2}] " +
                    $"vx={vx:F3} vy={vy:F3} vz={vz:F3} g={g:E3} " +
                    $"evx={evx:E3} evy={evy:E3}");
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"  [GeoDbg] Exception: {ex.Message}");
        }
    }
}
