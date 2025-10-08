using System;
using System.Collections.Generic;
using System.Linq;
using RowMeasureUtility; // ★ 追加: RMLib の角度正規化を使う

namespace KinematicsRM;

// MDH parameter container
public class MDHParameters
{
    public double[] Alpha
    {
        get;
    }
    public double[] A
    {
        get;
    }
    public double[] D
    {
        get;
    }
    public double[] Offset
    {
        get;
    }
    public double[] MinAnglesDeg
    {
        get;
    }
    public double[] MaxAnglesDeg
    {
        get;
    }

    public MDHParameters(double[] alpha, double[] a, double[] d, double[] offset, double[]? minAngles = null, double[]? maxAngles = null)
    {
        if (alpha.Length != 6 || a.Length != 6 || d.Length != 6 || offset.Length != 6)
            throw new ArgumentException("All MDH parameter arrays must have 6 elements.");
        Alpha = alpha;
        A = a;
        D = d;
        Offset = offset;
        MinAnglesDeg = minAngles ?? [-180, -180, -180, -180, -180, -360];
        MaxAnglesDeg = maxAngles ?? [180, 90, 180, 180, 180, 360];
    }
}

// Factory
public static class ManipulatorFactory
{
    private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["MiRobot"] = new MDHParameters(
            [0, Math.PI/2, Math.PI, -Math.PI/2, Math.PI/2, Math.PI/2],
            [0, 29.69, 108, 20, 0, 0],
            [127, 0, 0, 168.98, 0, 24.29],
            [0, Math.PI/2, 0, 0, -Math.PI/2, 0],
            [-180, -180, -180, -180, -180, -360],
            [180, 90, 180, 180, 180, 360]
        )
    };

    private static readonly Dictionary<string, double[]> _toolTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["null"]  = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ["tool0"] = [1.41421356, 0.0, 2.41421356, 0.70710678, 0.0, 0.70710678]
    };

    public static Manipulator6DoF CreateManipulator(string? robotName = null, string? toolName = null)
    {
        var r = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
        var t = string.IsNullOrWhiteSpace(toolName)  ? "null"   : toolName;
        if (!_mdhTable.TryGetValue(r, out var mdh)) throw new ArgumentException($"Unknown robot name: {r}");
        if (!_toolTable.TryGetValue(t, out var tool)) throw new ArgumentException($"Unknown tool name: {t}");
        return new Manipulator6DoF(mdh, tool);
    }

    public static IEnumerable<string> GetAvailableRobotNames() => _mdhTable.Keys;
    public static IEnumerable<string> GetAvailableToolNames() => _toolTable.Keys;
}

public class Manipulator6DoF
{
    private readonly MDHParameters _mdh;
    private readonly double[] _tool;
    private readonly double[] _minAnglesRad;
    private readonly double[] _maxAnglesRad;

    private static readonly double Deg2Rad = Math.PI / 180.0;

    public enum ToolReferenceAxis
    {
        FlangeY = 0, FlangeX = 1
    }
    private ToolReferenceAxis _toolRefAxis = ToolReferenceAxis.FlangeY;

    public Manipulator6DoF(MDHParameters mdh, double[] tool)
    {
        if (tool is null || tool.Length != 6)
            throw new ArgumentException("Tool must have 6 elements.");
        _mdh = mdh;
        _tool = (double[])tool.Clone();
        _minAnglesRad = _mdh.MinAnglesDeg.Select(a => a * Deg2Rad).ToArray();
        _maxAnglesRad = _mdh.MaxAnglesDeg.Select(a => a * Deg2Rad).ToArray();
    }

    public void SetToolReferenceAxis(ToolReferenceAxis axis) => _toolRefAxis = axis;

    // Forward (row-major internal axes; uses internal MDH recursion)
    // verbose:
    // 0: no output
    // 1: origins O0..O6 + Tool (Tool == flange)
    // 2: origins + axes directions for frames 1..6 + Tool (duplicate of frame 6)
    public double[,] Forward(double[] q, int verbose = 0)
    {
        if (q.Length != 6) throw new ArgumentException("q must have 6 elements");
        var alpha = _mdh.Alpha; var a = _mdh.A; var d = _mdh.D; var offset = _mdh.Offset;

        var origins = new double[8][];
        var axes = new double[8][,];
        origins[0] = new[] { 0.0, 0.0, 0.0 };
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][0, 1], axes[i][0, 2] };
            double[] Yprev = { axes[i][1, 0], axes[i][1, 1], axes[i][1, 2] };
            double[] Zprev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };

            var Xi = new double[3];
            Xi[0] = ct * Xprev[0] + st * (ca * Yprev[0] + sa * Zprev[0]);
            Xi[1] = ct * Xprev[1] + st * (ca * Yprev[1] + sa * Zprev[1]);
            Xi[2] = ct * Xprev[2] + st * (ca * Yprev[2] + sa * Zprev[2]);

            var Yi = new double[3];
            Yi[0] = -st * Xprev[0] + ct * (ca * Yprev[0] + sa * Zprev[0]);
            Yi[1] = -st * Xprev[1] + ct * (ca * Yprev[1] + sa * Zprev[1]);
            Yi[2] = -st * Xprev[2] + ct * (ca * Yprev[2] + sa * Zprev[2]);

            var Zi = new double[3];
            Zi[0] = -sa * Yprev[0] + ca * Zprev[0];
            Zi[1] = -sa * Yprev[1] + ca * Zprev[1];
            Zi[2] = -sa * Yprev[2] + ca * Zprev[2];

            axes[i + 1] = new double[,]
            {
                { Xi[0], Xi[1], Xi[2] },
                { Yi[0], Yi[1], Yi[2] },
                { Zi[0], Zi[1], Zi[2] }
            };

            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];
        }

        var T06 = new double[4, 4];
        for (var r = 0; r < 3; r++)
        {
            T06[r, 0] = axes[6][0, r];
            T06[r, 1] = axes[6][1, r];
            T06[r, 2] = axes[6][2, r];
            T06[r, 3] = origins[6][r];
        }
        T06[3, 0] = 0; T06[3, 1] = 0; T06[3, 2] = 0; T06[3, 3] = 1;

        origins[7] = new[] { origins[6][0], origins[6][1], origins[6][2] };
        axes[7] = new double[,]
        {
            { axes[6][0,0], axes[6][0,1], axes[6][0,2] },
            { axes[6][1,0], axes[6][1,1], axes[6][1,2] },
            { axes[6][2,0], axes[6][2,1], axes[6][2,2] }
        };

        if (verbose == 1 || verbose == 2)
        {
            Console.WriteLine("---- Origins (World) ----");
            for (var i = 0; i <= 6; i++)
                Console.WriteLine($"O{i}: ({origins[i][0]:F3}, {origins[i][1]:F3}, {origins[i][2]:F3})");
            Console.WriteLine($"Tool: ({origins[7][0]:F3}, {origins[7][1]:F3}, {origins[7][2]:F3})");
        }
        if (verbose == 2)
        {
            Console.WriteLine("---- Axes (World Directions) ----");
            for (var i = 1; i <= 6; i++)
            {
                Console.WriteLine($"Frame {i} X=({axes[i][0, 0]:F3},{axes[i][0, 1]:F3},{axes[i][0, 2]:F3}) " +
                                  $"Y=({axes[i][1, 0]:F3},{axes[i][1, 1]:F3},{axes[i][1, 2]:F3}) " +
                                  $"Z=({axes[i][2, 0]:F3},{axes[i][2, 1]:F3},{axes[i][2, 2]:F3})");
            }
            Console.WriteLine($"Tool   X=({axes[7][0, 0]:F3},{axes[7][0, 1]:F3},{axes[7][0, 2]:F3}) " +
                              $"Y=({axes[7][1, 0]:F3},{axes[7][1, 1]:F3},{axes[7][1, 2]:F3}) " +
                              $"Z=({axes[7][2, 0]:F3},{axes[7][2, 1]:F3},{axes[7][2, 2]:F3})");
        }

        return T06;
    }

    // --- NEW: Fully replaced InverseGeometric with crank-based elbow solver (option B) ---
    // This removes the former planar cosine-law approximation (r_sq, s, D_sq, L3, cosBase).
    // Method:
    // 1. Enumerate q1 candidates (atan2 + π).
    // 2. For each q1: compute wrist center Pw = P - d6 * Z_flange.
    // 3. Solve f(q2) = W2.z + d4 = 0  (W2 = R02^T (Pw - O2)) by bracket + secant refinement.
    // 4. For each root q2: compute q3 from   W2.x = a3 + a4 cos q3,  W2.y = a4 sin q3.
    // 5. Build T03, solve wrist (q4–q6) from T36 = T03^{-1} * T_target (same as previous method).
    // 6. Joint limit / duplicate filtering / forward verification.
    //
    // Notes:
    // - Uses original MDH arrays: a[2]=a3, a[3]=a4, d[3]=d4.
    // - Assumes offset[2]==0 (if not, adjust q3 by subtracting offset before normalization).
    // - If no roots found for a q1 branch, it produces no solutions for that branch.
    public double[][] InverseGeometric(double[,] T_target, out bool[] flags, int verbose = 0)
    {
        var allSolutions = new List<double[]>();
        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset;
        double a3 = a[2], a4 = a[3], d4 = d[3], d6 = d[5];

        double Px = T_target[0, 3], Py = T_target[1, 3], Pz = T_target[2, 3];
        double ZX = T_target[0, 2], ZY = T_target[1, 2], ZZ = T_target[2, 2];

        if (verbose >= 2)
        {
            Console.WriteLine("---- Target Tool Pose (World) ----");
            Console.WriteLine($"Pos : ({Px:F3}, {Py:F3}, {Pz:F3})");
            Console.WriteLine($"Z   : ({ZX:F3}, {ZY:F3}, {ZZ:F3})");
        }

        var PwX = Px - d6 * ZX;
        var PwY = Py - d6 * ZY;
        var PwZ = Pz - d6 * ZZ;
        if (verbose >= 2)
            Console.WriteLine($"Pw  : ({PwX:F3}, {PwY:F3}, {PwZ:F3})");

        var q1a = Math.Atan2(PwY, PwX);
        var q1b = RMLib.NormalizeAngleSigned(q1a + Math.PI);
        foreach (var q1raw in new[] { q1a, q1b })
        {
            var q1 = RMLib.NormalizeAngleSigned(q1raw);
            if (!WithinLimit(0, q1)) continue;
            if (verbose >= 2) Console.WriteLine($"\n-- q1 Candidate: {q1 * 180.0 / Math.PI:F4} deg");

            var q2Roots = SolveQ2Roots(q1, PwX, PwY, PwZ, verbose);
            if (q2Roots.Count == 0 && verbose >= 2)
                Console.WriteLine("   (No q2 roots)");

            foreach (var q2 in q2Roots)
            {
                if (!WithinLimit(1, q2)) continue;

                var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
                double vx = W2[0], vy = W2[1], vz = W2[2];

                if (Math.Abs(vy + d4) > 1e-3)      // ← 旧: (vz + d4)
                    continue;

                var cos3 = (vx - a3) / a4;
                var sin3 = vz / a4;             // ← sin は vz / a4 へ (旧: vy / a4)
                var norm = cos3 * cos3 + sin3 * sin3;
                if (Math.Abs(norm - 1.0) > 5e-3) continue;

                var q3 = RMLib.NormalizeAngleSigned(Math.Atan2(sin3, cos3));
                if (!WithinLimit(2, q3)) continue;

                var T01 = RMLib.DHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
                var T12 = RMLib.DHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
                var T23 = RMLib.DHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
                var T03 = RMLib.MatMul4x4(RMLib.MatMul4x4(T01, T12), T23);
                var T36 = RMLib.MatMul4x4(RMLib.InvertHomogeneous4x4(T03), T_target);

                var cos_q5 = T36[1, 2];
                if (cos_q5 < -1 || cos_q5 > 1) continue;

                foreach (var q5cand in new[] { Math.Acos(cos_q5), -Math.Acos(cos_q5) })
                {
                    var q5 = RMLib.NormalizeAngleSigned(q5cand);
                    var s5 = Math.Sin(q5);
                    double q4, q6;
                    if (Math.Abs(s5) > 1e-6)
                    {
                        q4 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 2] / s5, T36[0, 2] / s5));
                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[1, 1] / s5, -T36[1, 0] / s5));
                    }
                    else
                    {
                        q4 = 0;
                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 1], T36[2, 0]));
                    }

                    if (!WithinLimit(3, q4) || !WithinLimit(4, q5) || !WithinLimit(5, q6))
                        continue;

                    // ------------------- ここで O2..O6 をデバッグ出力（q4,q5,q6 が確定した後） -------------------
                    if (verbose >= 2)
                    {
                        // 再構築用 DH 行列（各関節オフセット込み角度）
                        var T01d = RMLib.DHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
                        var T12d = RMLib.DHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
                        var T23d = RMLib.DHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
                        var T34d = RMLib.DHTransform(q4 + off[3], d[3], a[3], _mdh.Alpha[3]);
                        var T45d = RMLib.DHTransform(q5 + off[4], d[4], a[4], _mdh.Alpha[4]);
                        var T56d = RMLib.DHTransform(q6 + off[5], d[5], a[5], _mdh.Alpha[5]);

                        var T02d = RMLib.MatMul4x4(T01d, T12d);
                        var T03d = RMLib.MatMul4x4(T02d, T23d);
                        var T04d = RMLib.MatMul4x4(T03d, T34d);
                        var T05d = RMLib.MatMul4x4(T04d, T45d);
                        var T06d = RMLib.MatMul4x4(T05d, T56d);

                        double[] O2dbg = { T02d[0, 3], T02d[1, 3], T02d[2, 3] };
                        double[] O3dbg = { T03d[0, 3], T03d[1, 3], T03d[2, 3] };
                        double[] O4dbg = { T04d[0, 3], T04d[1, 3], T04d[2, 3] };
                        double[] O5dbg = { T05d[0, 3], T05d[1, 3], T05d[2, 3] };
                        double[] O6dbg = { T06d[0, 3], T06d[1, 3], T06d[2, 3] };

                        Console.WriteLine("   -- Debug Origins (q1..q6 確定後) --");
                        Console.WriteLine($"     O2=({O2dbg[0]:F3},{O2dbg[1]:F3},{O2dbg[2]:F3})");
                        Console.WriteLine($"     O3=({O3dbg[0]:F3},{O3dbg[1]:F3},{O3dbg[2]:F3})");
                        Console.WriteLine($"     O4=({O4dbg[0]:F3},{O4dbg[1]:F3},{O4dbg[2]:F3})");
                        Console.WriteLine($"     O5=({O5dbg[0]:F3},{O5dbg[1]:F3},{O5dbg[2]:F3})");
                        Console.WriteLine($"     O6=({O6dbg[0]:F3},{O6dbg[1]:F3},{O6dbg[2]:F3})");
                        Console.WriteLine($"     Pw (再掲)=({PwX:F3},{PwY:F3},{PwZ:F3})  (※ O4=O5 と一致するはず)");
                    }
                    // ---------------------------------------------------------------------------------------------

                    var candidate = new double[6];
                    candidate[0] = RMLib.NormalizeAngleSigned(q1 - off[0]);
                    candidate[1] = RMLib.NormalizeAngleSigned(q2 - off[1]);
                    candidate[2] = RMLib.NormalizeAngleSigned(q3 - off[2]);
                    candidate[3] = RMLib.NormalizeAngleSigned(q4 - off[3]);
                    candidate[4] = RMLib.NormalizeAngleSigned(q5 - off[4]);
                    candidate[5] = RMLib.NormalizeAngleSigned(q6 - off[5]);

                    if (!WithinLimits(candidate)) continue;
                    if (IsDuplicate(allSolutions, candidate, 1e-3)) continue;

                    var Tchk = Forward(candidate, 0);
                    if (!PoseMatch(Tchk, T_target)) continue;

                    allSolutions.Add(candidate);
                    if (verbose >= 2)
                        Console.WriteLine($"   ACCEPT q= [{string.Join(", ", candidate.Select(v => (v * 180.0 / Math.PI).ToString("F3")))}]");
                }
            }
        }

        flags = new bool[allSolutions.Count];
        for (var i = 0; i < flags.Length; i++) flags[i] = true;
        return allSolutions.ToArray();
    }

    // ======== InverseJacobian (復活) ========
    public double[] InverseJacobian(
        double[,] T_target,
        double[] q_initial,
        out bool success,
        int maxIterations = 600,
        double tolerance = 1e-6,
        double alpha = 0.1,
        int verbose = 0)
    {
        var q = (double[])q_initial.Clone();
        var dX = new double[6];

        if (verbose >= 2)
        {
            Console.WriteLine("---- Target Pose (InverseJacobian) ----");
            Console.WriteLine($"Pos : ({T_target[0, 3]:F3},{T_target[1, 3]:F3},{T_target[2, 3]:F3})");
            Console.WriteLine($"Z   : ({T_target[0, 2]:F3},{T_target[1, 2]:F3},{T_target[2, 2]:F3})");
        }

        for (var iter = 0; iter < maxIterations; iter++)
        {
            var T_cur = Forward(q, 0);
            dX[0] = T_target[0, 3] - T_cur[0, 3];
            dX[1] = T_target[1, 3] - T_cur[1, 3];
            dX[2] = T_target[2, 3] - T_cur[2, 3];

            double[] n_c = { T_cur[0, 0], T_cur[1, 0], T_cur[2, 0] };
            double[] o_c = { T_cur[0, 1], T_cur[1, 1], T_cur[2, 1] };
            double[] a_c = { T_cur[0, 2], T_cur[1, 2], T_cur[2, 2] };
            double[] n_t = { T_target[0, 0], T_target[1, 0], T_target[2, 0] };
            double[] o_t = { T_target[0, 1], T_target[1, 1], T_target[2, 1] };
            double[] a_t = { T_target[0, 2], T_target[1, 2], T_target[2, 2] };

            var cn = RMLib.Cross3x3(n_c, n_t);
            var co = RMLib.Cross3x3(o_c, o_t);
            var ca = RMLib.Cross3x3(a_c, a_t);
            dX[3] = 0.5 * (cn[0] + co[0] + ca[0]);
            dX[4] = 0.5 * (cn[1] + co[1] + ca[1]);
            dX[5] = 0.5 * (cn[2] + co[2] + ca[2]);

            double errSq = 0;
            for (var k = 0; k < 6; k++) errSq += dX[k] * dX[k];
            if (errSq < tolerance * tolerance)
            {
                success = true;
                if (verbose >= 1)
                {
                    Console.WriteLine($"--- InverseJacobian converged iter={iter} err^2={errSq:E2} ---");
                    Forward(q, verbose >= 2 ? 2 : 1);
                }
                return q;
            }

            var J = CalculateJacobian(q);
            var JT = RMLib.TransposeRM(J);
            var dq = RMLib.MatVecMul(JT, dX);

            for (var j = 0; j < 6; j++)
            {
                q[j] = RMLib.NormalizeAngleSigned(q[j] + alpha * dq[j]);
                if (q[j] < _minAnglesRad[j]) q[j] = _minAnglesRad[j];
                if (q[j] > _maxAnglesRad[j]) q[j] = _maxAnglesRad[j];
            }

            if (verbose >= 2)
                Console.WriteLine($"Iter {iter} errSq={errSq:E3} q=[{string.Join(", ", q.Select(v => (v * 180 / Math.PI).ToString("F2")))}]");
        }

        success = false;
        if (verbose >= 1)
            Console.WriteLine("--- InverseJacobian: NOT converged ---");
        return q;
    }

    private double[,] CalculateJacobian(double[] q)
    {
        var J = new double[6, 6];
        var T_eff = Forward(q, 0);
        double[] p_eff = { T_eff[0, 3], T_eff[1, 3], T_eff[2, 3] };

        var alpha = _mdh.Alpha; var a = _mdh.A; var d = _mdh.D; var offset = _mdh.Offset;
        var origins = new double[7][];
        var axes = new double[7][,];
        origins[0] = [0.0, 0.0, 0.0];
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xprev = { axes[i][0, 0], axes[i][0, 1], axes[i][0, 2] };
            double[] Yprev = { axes[i][1, 0], axes[i][1, 1], axes[i][1, 2] };
            double[] Zprev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };

            var Xi = new double[3];
            Xi[0] = ct * Xprev[0] + st * (ca * Yprev[0] + sa * Zprev[0]);
            Xi[1] = ct * Xprev[1] + st * (ca * Yprev[1] + sa * Zprev[1]);
            Xi[2] = ct * Xprev[2] + st * (ca * Yprev[2] + sa * Zprev[2]);

            var Yi = new double[3];
            Yi[0] = -st * Xprev[0] + ct * (ca * Yprev[0] + sa * Zprev[0]);
            Yi[1] = -st * Xprev[1] + ct * (ca * Yprev[1] + sa * Zprev[1]);
            Yi[2] = -st * Xprev[2] + ct * (ca * Yprev[2] + sa * Zprev[2]);

            var Zi = new double[3];
            Zi[0] = -sa * Yprev[0] + ca * Zprev[0];
            Zi[1] = -sa * Yprev[1] + ca * Zprev[1];
            Zi[2] = -sa * Yprev[2] + ca * Zprev[2];

            axes[i + 1] = new double[,]
            {
                { Xi[0], Xi[1], Xi[2] },
                { Yi[0], Yi[1], Yi[2] },
                { Zi[0], Zi[1], Zi[2] }
            };

            origins[i + 1] = new double[3];
            origins[i + 1][0] = origins[i][0] + a[i] * Xprev[0] + d[i] * Zi[0];
            origins[i + 1][1] = origins[i][1] + a[i] * Xprev[1] + d[i] * Zi[1];
            origins[i + 1][2] = origins[i][2] + a[i] * Xprev[2] + d[i] * Zi[2];

            // Revolute joint axis = previous frame Z
            double[] zAxisPrev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };
            var p_i = origins[i];
            double[] p_diff = { p_eff[0] - p_i[0], p_eff[1] - p_i[1], p_eff[2] - p_i[2] };
            var Jv = RMLib.Cross3x3(zAxisPrev, p_diff);

            J[0, i] = Jv[0]; J[1, i] = Jv[1]; J[2, i] = Jv[2];
            J[3, i] = zAxisPrev[0]; J[4, i] = zAxisPrev[1]; J[5, i] = zAxisPrev[2];
        }
        return J;
    }

    // ======== q2 root solve helpers (既存) ========
    private List<double> SolveQ2Roots(double q1, double PwX, double PwY, double PwZ, int verbose)
    {
        var roots = new List<double>();
        var q2Min = _minAnglesRad[1];
        var q2Max = _maxAnglesRad[1];
        var samples = 31;
        var step = (q2Max - q2Min) / (samples - 1);

        var prevQ = double.NaN;
        var prevF = double.NaN;
        for (var i = 0; i < samples; i++)
        {
            var q2 = q2Min + i * step;
            var f = F_q2(q1, q2, PwX, PwY, PwZ);
            if (i > 0)
            {
                if (prevF * f <= 0.0)
                {
                    var r = RefineRootSecant(q1, prevQ, q2, PwX, PwY, PwZ);
                    if (double.IsFinite(r))
                    {
                        var fr = F_q2(q1, r, PwX, PwY, PwZ);
                        if (Math.Abs(fr) < 1e-3)
                        {
                            r = RMLib.NormalizeAngleSigned(r);
                            if (!roots.Any(x => Math.Abs(RMLib.NormalizeAngleSigned(x - r)) < 1e-3))
                                roots.Add(r);
                        }
                    }
                }
                else if (Math.Abs(f) < 1e-3)
                {
                    if (!roots.Any(x => Math.Abs(RMLib.NormalizeAngleSigned(x - q2)) < 1e-3))
                        roots.Add(q2);
                }
            }
            prevQ = q2; prevF = f;

            if (verbose >= 2)
                Console.WriteLine($"   sample q2={q2 * 180 / Math.PI:F3} f={f:F6}");
        }

        if (verbose >= 2 && roots.Count > 0)
            Console.WriteLine("   q2 roots(deg): " + string.Join(", ", roots.Select(r => (r * 180.0 / Math.PI).ToString("F3"))));
        return roots;
    }

    // f(q2) 定義を修正
    private double F_q2(double q1, double q2, double PwX, double PwY, double PwZ)
    {
        var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
        var d4 = _mdh.D[3];
        return W2[1] + d4;    // ← W2[1] (Y成分) に変更
    }

    private double[] ComputeW2(double q1, double q2, double PwX, double PwY, double PwZ)
    {
        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset; var alpha = _mdh.Alpha;
        var T01 = RMLib.DHTransform(q1 + off[0], d[0], a[0], alpha[0]);
        var T12 = RMLib.DHTransform(q2 + off[1], d[1], a[1], alpha[1]);
        var T02 = RMLib.MatMul4x4(T01, T12);

        double O2x = T02[0, 3], O2y = T02[1, 3], O2z = T02[2, 3];
        double[] X2 = { T02[0, 0], T02[1, 0], T02[2, 0] };
        double[] Y2 = { T02[0, 1], T02[1, 1], T02[2, 1] };
        double[] Z2 = { T02[0, 2], T02[1, 2], T02[2, 2] };

        var Wx = PwX - O2x;
        var Wy = PwY - O2y;
        var Wz = PwZ - O2z;

        var vx = Wx * X2[0] + Wy * X2[1] + Wz * X2[2];
        var vy = Wx * Y2[0] + Wy * Y2[1] + Wz * Y2[2];
        var vz = Wx * Z2[0] + Wy * Z2[1] + Wz * Z2[2];
        return new[] { vx, vy, vz };
    }

    private double RefineRootSecant(double q1, double q2a, double q2b,
                                    double PwX, double PwY, double PwZ,
                                    int maxIter = 25, double tol = 1e-8)
    {
        var fa = F_q2(q1, q2a, PwX, PwY, PwZ);
        var fb = F_q2(q1, q2b, PwX, PwY, PwZ);
        double a0 = q2a, b0 = q2b;
        for (var i = 0; i < maxIter; i++)
        {
            if (Math.Abs(fb - fa) < 1e-14) break;
            var c = b0 - fb * (b0 - a0) / (fb - fa);
            var fc = F_q2(q1, c, PwX, PwY, PwZ);
            a0 = b0; fa = fb;
            b0 = c; fb = fc;
            if (Math.Abs(fc) < tol) return c;
        }
        return b0;
    }

    // ======== Utility ========
    private bool WithinLimit(int idx, double valRad) =>
        valRad >= _minAnglesRad[idx] && valRad <= _maxAnglesRad[idx];

    private bool WithinLimits(double[] q)
    {
        for (var i = 0; i < 6; i++)
            if (!WithinLimit(i, q[i])) return false;
        return true;
    }

    private static bool IsDuplicate(List<double[]> sols, double[] cand, double tol)
    {
        foreach (var s in sols)
        {
            double sum = 0;
            for (var i = 0; i < 6; i++)
            {
                var d = RMLib.NormalizeAngleSigned(s[i] - cand[i]);
                sum += d * d;
            }
            if (sum < tol * tol) return true;
        }
        return false;
    }

    private static bool PoseMatch(double[,] A, double[,] B, double posTol = 1e-3, double angTol = 1e-3)
    {
        var dp =
            (A[0, 3] - B[0, 3]) * (A[0, 3] - B[0, 3]) +
            (A[1, 3] - B[1, 3]) * (A[1, 3] - B[1, 3]) +
            (A[2, 3] - B[2, 3]) * (A[2, 3] - B[2, 3]);
        if (dp > posTol * posTol) return false;

        var tr =
            A[0, 0] * B[0, 0] + A[1, 0] * B[1, 0] + A[2, 0] * B[2, 0] +
            A[0, 1] * B[0, 1] + A[1, 1] * B[1, 1] + A[2, 1] * B[2, 1] +
            A[0, 2] * B[0, 2] + A[1, 2] * B[1, 2] + A[2, 2] * B[2, 2];

        var cosAng = (tr - 1.0) * 0.5;
        cosAng = Math.Clamp(cosAng, -1.0, 1.0);
        var ang = Math.Acos(cosAng);
        return ang < angTol;
    }
}
