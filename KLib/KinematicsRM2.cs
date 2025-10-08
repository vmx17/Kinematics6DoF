using System;
using System.Collections.Generic;
using System.Linq;
using RowMeasureUtility; // ★ 追加: RMLib の角度正規化を使う

namespace KinematicsRM2;

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

                var T01 = RMLib.MDHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
                var T12 = RMLib.MDHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
                var T23 = RMLib.MDHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
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
                        var T01d = RMLib.MDHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
                        var T12d = RMLib.MDHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
                        var T23d = RMLib.MDHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
                        var T34d = RMLib.MDHTransform(q4 + off[3], d[3], a[3], _mdh.Alpha[3]);
                        var T45d = RMLib.MDHTransform(q5 + off[4], d[4], a[4], _mdh.Alpha[4]);
                        var T56d = RMLib.MDHTransform(q6 + off[5], d[5], a[5], _mdh.Alpha[5]);

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

    // ======== InverseJacobian ========
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

    // ================== Jacobian (single implementation) ==================
    private double[,] CalculateJacobian(double[] q)
    {
        var J = new double[6, 6];

        // エンドエフェクタ位置
        var T_eff = Forward(q, 0);
        double[] p_eff = { T_eff[0, 3], T_eff[1, 3], T_eff[2, 3] };

        var alpha = _mdh.Alpha;
        var a = _mdh.A;
        var d = _mdh.D;
        var offset = _mdh.Offset;

        // 再計算用（Forward と同じ再帰ロジック）
        var origins = new double[7][];
        var axes = new double[7][,];
        origins[0] = new[] { 0.0, 0.0, 0.0 };
        axes[0] = new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

        for (int i = 0; i < 6; i++)
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

            // 回転関節：軸 = 直前フレームZ
            double[] zAxisPrev = { axes[i][2, 0], axes[i][2, 1], axes[i][2, 2] };
            var p_i = origins[i];
            double[] p_diff = { p_eff[0] - p_i[0], p_eff[1] - p_i[1], p_eff[2] - p_i[2] };
            var Jv = RMLib.Cross3x3(zAxisPrev, p_diff);

            J[0, i] = Jv[0];
            J[1, i] = Jv[1];
            J[2, i] = Jv[2];
            J[3, i] = zAxisPrev[0];
            J[4, i] = zAxisPrev[1];
            J[5, i] = zAxisPrev[2];
        }

        return J;
    }

    // Tool transform:
    // _tool[0..2] : translation of tool TCP origin in flange frame (meters/mm consistent with model units)
    // _tool[3..5] : tool local +Z axis direction expressed in flange frame (need not be perfectly normalized)
    // Only axis direction is specified -> roll about Z is chosen deterministically:
    //   Pick a reference vector least aligned with Z (from {X,Y,Z_world}) then build X = normalize(ref × Z), Y = Z × X
    // Resulting homogeneous transform maps points from tool frame to flange frame? (We want flange->tool)
    // Convention here: columns store tool frame unit axes expressed in flange frame (same as your MDH usage),
    // so multiplying flange->tool = [R | p].
    private double[,] GetToolTransform()
    {
        double tx = _tool[0], ty = _tool[1], tz = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        double roll = 0.0;                         // additional roll about tool Z
        if (_tool.Length >= 7) roll = _tool[6];    // radians

        // If this is the 'null' tool (identity orientation + zero translation)
        var normZ = Math.Sqrt(zx * zx + zy * zy + zz * zz);
        var nullLike =
            Math.Abs(tx) < 1e-12 &&
            Math.Abs(ty) < 1e-12 &&
            Math.Abs(tz) < 1e-12 &&
            (normZ < 1e-12 || (Math.Abs(zx) < 1e-12 && Math.Abs(zy) < 1e-12 && Math.Abs(zz - 1.0) < 1e-12));

        if (nullLike)
        {
            var I = RMLib.Identity4x4();
            return I;
        }

        // Normalize Z (fallback to +Z if degenerate)
        if (normZ < 1e-10)
        {
            zx = 0; zy = 0; zz = 1;
            normZ = 1;
        }
        zx /= normZ; zy /= normZ; zz /= normZ;

        // Choose a reference vector minimally aligned with Z
        // Candidates: ex=(1,0,0), ey=(0,1,0), ez=(0,0,1)
        double[] ref1 = [1, 0, 0];
        double[] ref2 = [0, 1, 0];
        double[] ref3 = [0, 0, 1];

        var dot1 = Math.Abs(ref1[0] * zx + ref1[1] * zy + ref1[2] * zz);
        var dot2 = Math.Abs(ref2[0] * zx + ref2[1] * zy + ref2[2] * zz);
        var dot3 = Math.Abs(ref3[0] * zx + ref3[1] * zy + ref3[2] * zz);

        double[] refVec;
        if (dot1 <= dot2 && dot1 <= dot3) refVec = ref1;
        else if (dot2 <= dot1 && dot2 <= dot3) refVec = ref2;
        else refVec = ref3;

        // X = normalize(ref × Z)
        var cx = refVec[1] * zz - refVec[2] * zy;
        var cy = refVec[2] * zx - refVec[0] * zz;
        var cz = refVec[0] * zy - refVec[1] * zx;
        var nC = Math.Sqrt(cx * cx + cy * cy + cz * cz);
        if (nC < 1e-12)
        {
            // Fallback: pick arbitrary orthogonal
            if (Math.Abs(zx) < 0.9) { cx = 0; cy = zz; cz = -zy; }
            else { cx = -zz; cy = 0; cz = zx; }
            nC = Math.Sqrt(cx * cx + cy * cy + cz * cz);
        }
        cx /= nC; cy /= nC; cz /= nC;

        // Y = Z × X
        var yx = zy * cz - zz * cy;
        var yy = zz * cx - zx * cz;
        var yz = zx * cy - zy * cx;

        // Orthonormalization refinement (optional small Gram-Schmidt)
        // Recompute X = Y × Z to reduce accumulated error
        var rx = yy * zz - yz * zy;
        var ry = yz * zx - yx * zz;
        var rz = yx * zy - yy * zx;
        var nR = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        if (nR > 1e-12)
        {
            cx = rx / nR; cy = ry / nR; cz = rz / nR;
        }

        // Build homogeneous transform (row-major; columns = basis vectors)
        var T = RMLib.Identity4x4();
        // Column 0 = X axis
        T[0, 0] = cx; T[1, 0] = cy; T[2, 0] = cz;
        // Column 1 = Y axis
        T[0, 1] = yx; T[1, 1] = yy; T[2, 1] = yz;
        // Column 2 = Z axis (given)
        T[0, 2] = zx; T[1, 2] = zy; T[2, 2] = zz;
        // Translation
        T[0, 3] = tx; T[1, 3] = ty; T[2, 3] = tz;

        return T;
    }

    // -------------------- Tool roll support API (UNIFIED) --------------------

    public double GetToolRoll() => _tool.Length >= 7 ? _tool[6] : 0.0;

    public void SetToolRoll(double roll)
    {
        if (_tool.Length >= 7)
        {
            _tool[6] = roll;
            return;
        }
        var ext = new double[7];
        Array.Copy(_tool, ext, 6);
        ext[6] = roll;
        for (int i = 0; i < ext.Length; i++) _tool[i] = ext[i];
    }

    /// <summary>
    /// Build flange->tool transform (columns = tool axes in flange) with optional roll override.
    /// If rollOverride is null use stored roll (or 0 if none).
    /// </summary>
    public double[,] GetToolTransform(double? rollOverride = null)
    {
        double tx = _tool[0], ty = _tool[1], tz = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        double roll = rollOverride ?? GetToolRoll();

        double nZ = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (nZ < 1e-14)
        {
            var I = RMLib.Identity4x4();
            I[0,3] = tx; I[1,3] = ty; I[2,3] = tz;
            return I;
        }
        zx/=nZ; zy/=nZ; zz/=nZ;

        // choose reference least aligned with Z
        double ax = Math.Abs(zx), ay = Math.Abs(zy), az = Math.Abs(zz);
        double[] refVec;
        if (ax <= ay && ax <= az) refVec = new[]{0.0,1.0,0.0};
        else if (ay <= ax && ay <= az) refVec = new[]{1.0,0.0,0.0};
        else refVec = new[]{1.0,0.0,0.0};

        // X0 = ref × Z
        double x0x = refVec[1]*zz - refVec[2]*zy;
        double x0y = refVec[2]*zx - refVec[0]*zz;
        double x0z = refVec[0]*zy - refVec[1]*zx;
        double nX = Math.Sqrt(x0x*x0x + x0y*x0y + x0z*x0z);
        if (nX < 1e-12)
        {
            if (Math.Abs(zx) < 0.9) { x0x=0; x0y=zz; x0z=-zy; }
            else { x0x=-zz; x0y=0; x0z=zx; }
            nX = Math.Sqrt(x0x*x0x + x0y*x0y + x0z*x0z);
        }
        x0x/=nX; x0y/=nX; x0z/=nX;

        // Y0 = Z × X0
        double y0x = zy*x0z - zz*x0y;
        double y0y = zz*x0x - zx*x0z;
        double y0z = zx*x0y - zy*x0x;

        if (Math.Abs(roll) > 1e-15)
        {
            double cr = Math.Cos(roll), sr = Math.Sin(roll);
            double Xx = cr*x0x + sr*y0x;
            double Xy = cr*x0y + sr*y0y;
            double Xz = cr*x0z + sr*y0z;
            double Yx = -sr*x0x + cr*y0x;
            double Yy = -sr*x0y + cr*y0y;
            double Yz = -sr*x0z + cr*y0z;
            x0x = Xx; x0y = Xy; x0z = Xz;
            y0x = Yx; y0y = Yy; y0z = Yz;
        }

        var T = RMLib.Identity4x4();
        T[0,0]=x0x; T[1,0]=x0y; T[2,0]=x0z;
        T[0,1]=y0x; T[1,1]=y0y; T[2,1]=y0z;
        T[0,2]=zx;  T[1,2]=zy;  T[2,2]=zz;
        T[0,3]=tx;  T[1,3]=ty;  T[2,3]=tz;
        return T;
    }

    /// <summary>
    /// Forward with tool (returns TCP). rollOverride rotates X/Y about tool Z.
    /// </summary>
    public double[,] ForwardWithTool(double[] q, double? rollOverride = null, int verbose = 0)
    {
        var Tflange = Forward(q, verbose);
        var Ttool   = GetToolTransform(rollOverride);
        return RMLib.MatMul4x4(Tflange, Ttool);
    }

    /// <summary>
    /// Geometric IK for TCP target.
    /// </summary>
    public double[][] InverseGeometricTcp(double[,] T_tcp_target, out bool[] flags,
                                          double? rollOverride = null, int verbose = 0)
    {
        var T_ft     = GetToolTransform(rollOverride);
        var T_ft_inv = RMLib.InvertHomogeneous4x4(T_ft);
        var T_flange_target = RMLib.MatMul4x4(T_tcp_target, T_ft_inv);
        return InverseGeometric(T_flange_target, out flags, verbose);
    }

    /// <summary>
    /// Jacobian IK for TCP target.
    /// </summary>
    public double[] InverseJacobianTcp(double[,] T_tcp_target, double[] q_initial,
                                       out bool success, double? rollOverride = null,
                                       int maxIterations = 600, double tolerance = 1e-6,
                                       double alpha = 0.1, int verbose = 0)
    {
        var T_ft     = GetToolTransform(rollOverride);
        var T_ft_inv = RMLib.InvertHomogeneous4x4(T_ft);
        var T_flange_target = RMLib.MatMul4x4(T_tcp_target, T_ft_inv);
        return InverseJacobian(T_flange_target, q_initial, out success,
                               maxIterations, tolerance, alpha, verbose);
    }

    // --- OLD: InverseGeometric (for reference) ---
    //    public double[][] InverseGeometric(double[,] T_target, out bool[] flags, int verbose = 0)
    //    {
    //        var allSolutions = new List<double[]>();
    //        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset;
    //        double a3 = a[2], a4 = a[3], d4 = d[3], d6 = d[5];

    //        double Px = T_target[0, 3], Py = T_target[1, 3], Pz = T_target[2, 3];
    //        double ZX = T_target[0, 2], ZY = T_target[1, 2], ZZ = T_target[2, 2];

    //        if (verbose >= 2)
    //        {
    //            Console.WriteLine("---- Target Tool Pose (World) ----");
    //            Console.WriteLine($"Pos : ({Px:F3}, {Py:F3}, {Pz:F3})");
    //            Console.WriteLine($"Z   : ({ZX:F3}, {ZY:F3}, {ZZ:F3})");
    //        }

    //        var PwX = Px - d6 * ZX;
    //        var PwY = Py - d6 * ZY;
    //        var PwZ = Pz - d6 * ZZ;
    //        if (verbose >= 2)
    //            Console.WriteLine($"Pw  : ({PwX:F3}, {PwY:F3}, {PwZ:F3})");

    //        var q1a = Math.Atan2(PwY, PwX);
    //        var q1b = RMLib.NormalizeAngleSigned(q1a + Math.PI);
    //        foreach (var q1raw in new[] { q1a, q1b })
    //        {
    //            var q1 = RMLib.NormalizeAngleSigned(q1raw);
    //            if (!WithinLimit(0, q1)) continue;
    //            if (verbose >= 2) Console.WriteLine($"\n-- q1 Candidate: {q1 * 180.0 / Math.PI:F4} deg");

    //            var q2Roots = SolveQ2Roots(q1, PwX, PwY, PwZ, verbose);
    //            if (q2Roots.Count == 0 && verbose >= 2)
    //                Console.WriteLine("   (No q2 roots)");

    //            foreach (var q2 in q2Roots)
    //            {
    //                if (!WithinLimit(1, q2)) continue;

    //                var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
    //                double vx = W2[0], vy = W2[1], vz = W2[2];

    //                if (Math.Abs(vy + d4) > 1e-3)      // ← 旧: (vz + d4)
    //                    continue;

    //                var cos3 = (vx - a3) / a4;
    //                var sin3 = vz / a4;             // ← sin は vz / a4 へ (旧: vy / a4)
    //                var norm = cos3 * cos3 + sin3 * sin3;
    //                if (Math.Abs(norm - 1.0) > 5e-3) continue;

    //                var q3 = RMLib.NormalizeAngleSigned(Math.Atan2(sin3, cos3));
    //                if (!WithinLimit(2, q3)) continue;

    //                var T01 = RMLib.MDHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
    //                var T12 = RMLib.MDHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
    //                var T23 = RMLib.MDHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
    //                var T03 = RMLib.MatMul4x4(RMLib.MatMul4x4(T01, T12), T23);
    //                var T36 = RMLib.MatMul4x4(RMLib.InvertHomogeneous4x4(T03), T_target);

    //                var cos_q5 = T36[1, 2];
    //                if (cos_q5 < -1 || cos_q5 > 1) continue;

    //                foreach (var q5cand in new[] { Math.Acos(cos_q5), -Math.Acos(cos_q5) })
    //                {
    //                    var q5 = RMLib.NormalizeAngleSigned(q5cand);
    //                    var s5 = Math.Sin(q5);
    //                    double q4, q6;
    //                    if (Math.Abs(s5) > 1e-6)
    //                    {
    //                        q4 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 2] / s5, T36[0, 2] / s5));
    //                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[1, 1] / s5, -T36[1, 0] / s5));
    //                    }
    //                    else
    //                    {
    //                        q4 = 0;
    //                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 1], T36[2, 0]));
    //                    }

    //                    if (!WithinLimit(3, q4) || !WithinLimit(4, q5) || !WithinLimit(5, q6))
    //                        continue;

    //                    // ------------------- ここで O2..O6 をデバッグ出力（q4,q5,q6 が確定した後） -------------------
    //                    if (verbose >= 2)
    //                    {
    //                        // 再構築用 DH 行列（各関節オフセット込み角度）
    //                        var T01d = RMLib.MDHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
    //                        var T12d = RMLib.MDHTransform(q2 + off[1], d[1], a[1], _mdh.Alpha[1]);
    //                        var T23d = RMLib.MDHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
    //                        var T34d = RMLib.MDHTransform(q4 + off[3], d[3], a[3], _mdh.Alpha[3]);
    //                        var T45d = RMLib.MDHTransform(q5 + off[4], d[4], a[4], _mdh.Alpha[4]);
    //                        var T56d = RMLib.MDHTransform(q6 + off[5], d[5], a[5], _mdh.Alpha[5]);

    //                        var T02d = RMLib.MatMul4x4(T01d, T12d);
    //                        var T03d = RMLib.MatMul4x4(T02d, T23d);
    //                        var T04d = RMLib.MatMul4x4(T03d, T34d);
    //                        var T05d = RMLib.MatMul4x4(T04d, T45d);
    //                        var T06d = RMLib.MatMul4x4(T05d, T56d);

    //                        double[] O2dbg = { T02d[0, 3], T02d[1, 3], T02d[2, 3] };
    //                        double[] O3dbg = { T03d[0, 3], T03d[1, 3], T03d[2, 3] };
    //                        double[] O4dbg = { T04d[0, 3], T04d[1, 3], T04d[2, 3] };
    //                        double[] O5dbg = { T05d[0, 3], T05d[1, 3], T05d[2, 3] };
    //                        double[] O6dbg = { T06d[0, 3], T06d[1, 3], T06d[2, 3] };

    //                        Console.WriteLine("   -- Debug Origins (q1..q6 確定後) --");
    //                        Console.WriteLine($"     O2=({O2dbg[0]:F3},{O2dbg[1]:F3},{O2dbg[2]:F3})");
    //                        Console.WriteLine($"     O3=({O3dbg[0]:F3},{O3dbg[1]:F3},{O3dbg[2]:F3})");
    //                        Console.WriteLine($"     O4=({O4dbg[0]:F3},{O4dbg[1]:F3},{O4dbg[2]:F3})");
    //                        Console.WriteLine($"     O5=({O5dbg[0]:F3},{O5dbg[1]:F3},{O5dbg[2]:F3})");
    //                        Console.WriteLine($"     O6=({O6dbg[0]:F3},{O6dbg[1]:F3},{O6dbg[2]:F3})");
    //                        Console.WriteLine($"     Pw (再掲)=({PwX:F3},{PwY:F3},{PwZ:F3})  (※ O4=O5 と一致するはず)");
    //                    }
    //                    // ---------------------------------------------------------------------------------------------

    //                    var candidate = new double[6];
    //                    candidate[0] = RMLib.NormalizeAngleSigned(q1 - off[0]);
    //                    candidate[1] = RMLib.NormalizeAngleSigned(q2 - off[1]);
    //                    candidate[2] = RMLib.NormalizeAngleSigned(q3 - off[2]);
    //                    candidate[3] = RMLib.NormalizeAngleSigned(q4 - off[3]);
    //                    candidate[4] = RMLib.NormalizeAngleSigned(q5 - off[4]);
    //                    candidate[5] = RMLib.NormalizeAngleSigned(q6 - off[5]);

    //                    if (!WithinLimits(candidate)) continue;
    //                    if (IsDuplicate(allSolutions, candidate, 1e-3)) continue;

    //                    var Tchk = Forward(candidate, 0);
    //                    if (!PoseMatch(Tchk, T_target)) continue;

    //                    allSolutions.Add(candidate);
    //                    if (verbose >= 2)
    //                        Console.WriteLine($"   ACCEPT q= [{string.Join(", ", candidate.Select(v => (v * 180.0 / Math.PI).ToString("F3")))}]");
    //                }
    //            }
    //        }

    //        flags = new bool[allSolutions.Count];
    //        for (var i = 0; i < flags.Length; i++) flags[i] = true;
    //        return allSolutions.ToArray();
    //    }

    // ======== InverseJacobian ========
    // public double[] InverseJacobian(
    //     double[,] T_target,
    //     double[] q_initial,
    //     out bool success,
    //     int maxIterations = 600,
    //     double tolerance = 1e-6,
    //     double alpha = 0.1,
    //     int verbose = 0)
    // {
    //     var q = (double[])q_initial.Clone();
    //     var dX = new double[6];

    //     if (verbose >= 2)
    //     {
    //         Console.WriteLine("---- Target Pose (InverseJacobian) ----");
    //         Console.WriteLine($"Pos : ({T_target[0, 3]:F3},{T_target[1, 3]:F3},{T_target[2, 3]:F3})");
    //         Console.WriteLine($"Z   : ({T_target[0, 2]:F3},{T_target[1, 2]:F3},{T_target[2, 2]:F3})");
    //     }

    //     for (var iter = 0; iter < maxIterations; iter++)
    //     {
    //         var T_cur = Forward(q, 0);
    //         dX[0] = T_target[0, 3] - T_cur[0, 3];
    //         dX[1] = T_target[1, 3] - T_cur[1, 3];
    //         dX[2] = T_target[2, 3] - T_cur[2, 3];

    //         double[] n_c = { T_cur[0, 0], T_cur[1, 0], T_cur[2, 0] };
    //         double[] o_c = { T_cur[0, 1], T_cur[1, 1], T_cur[2, 1] };
    //         double[] a_c = { T_cur[0, 2], T_cur[1, 2], T_cur[2, 2] };
    //         double[] n_t = { T_target[0, 0], T_target[1, 0], T_target[2, 0] };
    //         double[] o_t = { T_target[0, 1], T_target[1, 1], T_target[2, 1] };
    //         double[] a_t = { T_target[0, 2], T_target[1, 2], T_target[2, 2] };

    //         var cn = RMLib.Cross3x3(n_c, n_t);
    //         var co = RMLib.Cross3x3(o_c, o_t);
    //         var ca = RMLib.Cross3x3(a_c, a_t);
    //         dX[3] = 0.5 * (cn[0] + co[0] + ca[0]);
    //         dX[4] = 0.5 * (cn[1] + co[1] + ca[1]);
    //         dX[5] = 0.5 * (cn[2] + co[2] + ca[2]);

    //         double errSq = 0;
    //         for (var k = 0; k < 6; k++) errSq += dX[k] * dX[k];
    //         if (errSq < tolerance * tolerance)
    //         {
    //             success = true;
    //             if (verbose >= 1)
    //             {
    //                 Console.WriteLine($"--- InverseJacobian converged iter={iter} err^2={errSq:E2} ---");
    //                 Forward(q, verbose >= 2 ? 2 : 1);
    //             }
    //             return q;
    //         }

    //         var J = CalculateJacobian(q);
    //         var JT = RMLib.TransposeRM(J);
    //         var dq = RMLib.MatVecMul(JT, dX);

    //         for (var j = 0; j < 6; j++)
    //         {
    //             q[j] = RMLib.NormalizeAngleSigned(q[j] + alpha * dq[j]);
    //             if (q[j] < _minAnglesRad[j]) q[j] = _minAnglesRad[j];
    //             if (q[j] > _maxAnglesRad[j]) q[j] = _maxAnglesRad[j];
    //         }

    //         if (verbose >= 2)
    //             Console.WriteLine($"Iter {iter} errSq={errSq:E3} q=[{string.Join(", ", q.Select(v => (v * 180 / Math.PI).ToString("F2")))}]");
    //     }

    //     success = false;
    //     if (verbose >= 1)
    //         Console.WriteLine("--- InverseJacobian: NOT converged ---");
    //     return q;
    // }

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

            //if (verbose >= 2) Console.WriteLine($"   sample q2={q2 * 180 / Math.PI:F3} f={f:F6}");
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

    /// <summary>
    /// Check if candidate solution is a duplicate of any in the list (within tol in joint space)
    /// </summary>
    /// <param name="sols"></param>
    /// <param name="cand"></param>
    /// <param name="tol"></param>
    /// <returns></returns>
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

    /// <summary>
    /// Returns (Rx,Ry,Rz) about flange X,Y,Z axes such that:
    /// R = RotX(Rx)*RotY(Ry)*RotZ(Rz) with Rz fixed to 0 by convention,
    /// and R * (0,0,1) = tool Z direction (normalized).
    /// Since only tool Z is specified in _tool, roll about that axis is undefined;
    /// we set Rz=0 to pick a deterministic representative.
    /// </summary>
    public (double Rx, double Ry, double Rz) GetToolTiltAngles()
    {
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        double n = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (n < 1e-12) return (0, 0, 0);
        zx /= n; zy /= n; zz /= n;

        // Handle near-horizontal (cosRy -> 0) case carefully.
        // Ry from sinRy = zx
        double sinRy = zx;
        if (sinRy > 1.0) sinRy = 1.0;
        if (sinRy < -1.0) sinRy = -1.0;
        double Ry = Math.Asin(sinRy);
        double cosRy = Math.Cos(Ry);

        double Rx;
        if (Math.Abs(cosRy) < 1e-9)
        {
            // Degenerate: tool Z almost in ±X direction; set Rx from zy≈ -sinRx*cosRy ~ 0 -> choose 0
            Rx = 0.0;
        }
        else
        {
            // sinRx = -zy / cosRy, cosRx = zz / cosRy
            double sinRx = -zy / cosRy;
            double cosRx = zz / cosRy;
            // Clamp to protect floating noise
            double mag = Math.Sqrt(sinRx*sinRx + cosRx*cosRx);
            if (mag > 1e-12)
            {
                sinRx /= mag;
                cosRx /= mag;
            }
            Rx = Math.Atan2(sinRx, cosRx);
        }

        // Rz = 0 by convention (no roll about final Z)
        double Rz = 0.0;

        Rx = RMLib.NormalizeAngleSigned(Rx);
        Ry = RMLib.NormalizeAngleSigned(Ry);
        return (Rx, Ry, Rz);
    }

    /// <summary>
    /// Returns full tool pose relative to flange as (tx,ty,tz,Rx,Ry,Rz) with Rz=0 convention (see GetToolTiltAngles).
    /// </summary>
    public (double tx, double ty, double tz, double Rx, double Ry, double Rz) GetToolPoseDecomposed()
    {
        var (Rx, Ry, Rz) = GetToolTiltAngles();
        return (_tool[0], _tool[1], _tool[2], Rx, Ry, Rz);
    }

    /// <summary>
    /// Build a homogeneous 4x4 that maps flange frame to the tool frame using the (Rx,Ry,0) convention.
    /// This matches only the Z-direction; roll about Z is the one implicitly chosen by the internal X construction.
    /// </summary>
    public double[,] GetToolTiltTransform()
    {
        var (Rx, Ry, Rz) = GetToolTiltAngles(); // Rz always 0
        double sx = Math.Sin(Rx), cx = Math.Cos(Rx);
        double sy = Math.Sin(Ry), cy = Math.Cos(Ry);
        double sz = 0.0, cz = 1.0; // Rz = 0

        // R = Rx * Ry * Rz ? We defined mapping by RotX * RotY * RotZ (Rz=0)
        // Compute RotX * RotY first:
        // RotX:
        // [1 0 0; 0 cx -sx; 0 sx cx]
        // RotY:
        // [cy 0 sy; 0 1 0; -sy 0 cy]
        double r00 = 1*cy + 0;         // = cy
        double r01 = 0;                // ignoring Rz (will multiply by identity)
        double r02 = 1*sy;
        double r10 = 0*cy + cx*0 + (-sx)*(-sy); // sx*sy
        double r11 = cx;
        double r12 = 0*sy + cx*0 + (-sx)*cy;    // -sx*cy
        double r20 = 0*cy + sx*0 + cx*(-sy);    // -cx*sy
        double r21 = sx;
        double r22 = cx*cy;       // cx*cy

        var T = RMLib.Identity4x4();
        T[0, 0] = r00; T[0, 1] = r01; T[0, 2] = r02;
        T[1, 0] = r10; T[1, 1] = r11; T[1, 2] = r12;
        T[2, 0] = r20; T[2, 1] = r21; T[2, 2] = r22;
        T[0, 3] = _tool[0];
        T[1, 3] = _tool[1];
        T[2, 3] = _tool[2];
        return T;
    }

    /// <summary>
    /// Compute rotation (axis-angle) that aligns tool Z to canonical (0,0,1).
    /// Returns (ax,ay,az, angle). This is independent of arbitrary roll.
    /// </summary>
    public (double ax, double ay, double az, double angle) GetToolZAlignment()
    {
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        double n = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (n < 1e-12) return (0, 0, 1, 0);
        zx /= n; zy /= n; zz /= n;
        double dot = Math.Clamp(zz, -1.0, 1.0);
        double angle = Math.Acos(dot);
        if (angle < 1e-12) return (0, 0, 1, 0);
        // axis = z_tool x z_canonical = (zy, -zx, 0)
        double ax = zy, ay = -zx, az = 0;
        double m = Math.Sqrt(ax*ax + ay*ay);
        if (m < 1e-12) return (1, 0, 0, Math.PI); // opposed
        return (ax / m, ay / m, 0, angle);
    }

    /// <summary>
    /// Return 6 doubles representing the inverse (flange←tool) pose of the current tool specification
    /// using the same (Rx,Ry,Rz=0) convention as GetToolTiltAngles() / GetToolPoseDecomposed().
    /// Output order: (Tx, Ty, Tz, Rx, Ry, Rz) where applying these to the tool frame
    /// (i.e. RotX(Rx)*RotY(Ry)*RotZ(Rz) and then translate by (Tx,Ty,Tz)) will bring the tool
    /// origin/orientation back to the flange origin with Z aligned (0,0,1).
    /// Notes:
    /// - Because only the tool Z direction is stored, Rz is fixed to 0 (no roll).
    /// - Inverse transform:
    ///     Forward (flange->tool):  p_tool = R * p_flange + t
    ///     Inverse (tool->flange):  p_flange = R^T * (p_tool - t)
    ///   Therefore inverse translation in flange coordinates is  t_inv = -R^T * t
    ///   and inverse rotation angles are simply (-Rx, -Ry, -Rz) under this simple X-Y-Z (with Z=0) sequence.
    /// </summary>
    public double[] GetToolInversePose()
    {
        // Acquire forward convention pose (tx,ty,tz,Rx,Ry,Rz=0)
        var (TxF, TyF, TzF, RxF, RyF, RzF) = GetToolPoseDecomposed(); // RzF=0 by convention

        // Build rotation R = RotX(RxF) * RotY(RyF) * RotZ(RzF=0)
        double sx = Math.Sin(RxF), cx = Math.Cos(RxF);
        double sy = Math.Sin(RyF), cy = Math.Cos(RyF);
        // RotX * RotY (Z rotation is identity):
        // Derived earlier (same as in GetToolTiltTransform)
        var r00 = cy;
        var r01 = 0.0;
        var r02 = sy;

        var r10 = sx * sy;
        var r11 = cx;
        var r12 = -sx * cy;

        var r20 = -cx * sy;
        var r21 = sx;
        var r22 = cx * cy;

        // Forward translation t = (TxF,TyF,TzF)
        // Inverse translation t_inv = -R^T * t
        var tInvX = -(r00 * TxF + r10 * TyF + r20 * TzF);
        var tInvY = -(r01 * TxF + r11 * TyF + r21 * TzF);
        var tInvZ = -(r02 * TxF + r12 * TyF + r22 * TzF);

        // Inverse rotation angles (normalize)
        var RxInv = RMLib.NormalizeAngleSigned(-RxF);
        var RyInv = RMLib.NormalizeAngleSigned(-RyF);
        var RzInv = RMLib.NormalizeAngleSigned(-RzF); // zero

        return
        [
            tInvX, tInvY, tInvZ,
            RxInv, RyInv, RzInv
        ];
    }
}