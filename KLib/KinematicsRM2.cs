using System;
using System.Collections.Generic;
using System.Linq;
using RowMeasureUtility; // RMLib

namespace KinematicsRM2;

public class MDHParameters
{
    public double[] Alpha { get; }
    public double[] A { get; }
    public double[] D { get; }
    public double[] Offset { get; }
    public double[] MinAnglesDeg { get; }
    public double[] MaxAnglesDeg { get; }

    public MDHParameters(double[] alpha, double[] a, double[] d, double[] offset,
                         double[]? minAngles = null, double[]? maxAngles = null)
    {
        if (alpha.Length != 6 || a.Length != 6 || d.Length != 6 || offset.Length != 6)
            throw new ArgumentException("All MDH parameter arrays must have 6 elements.");
        Alpha = alpha;
        A = a;
        D = d;
        Offset = offset;
        MinAnglesDeg = minAngles ?? new[] { -180.0, -180.0, -180.0, -180.0, -180.0, -360.0 };
        MaxAnglesDeg = maxAngles ?? new[] {  180.0,   90.0,  180.0,  180.0,  180.0,  360.0 };
    }
}

public static class ManipulatorFactory
{
    private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["MiRobot"] = new MDHParameters(
            new[] {0, Math.PI/2, Math.PI, -Math.PI/2, Math.PI/2, Math.PI/2},
            new[] {0, 29.69, 108, 20, 0, 0},
            new[] {127, 0, 0, 168.98, 0, 24.29},
            new[] {0, Math.PI/2, 0, 0, -Math.PI/2, 0},
            new double[] {-180, -180, -180, -180, -180, -360},
            new double[] { 180,   90,  180,  180,  180,  360}
        ),
        // MiRobot2: a4=168.98, d4=-20 (保持) -> 汎用幾何IKで a4,d4 同時扱い
        ["MiRobot2"] = new MDHParameters(
            new[] {0, Math.PI/2, 0, Math.PI/2, Math.PI/2, -Math.PI/2},
            new[] {0, 29.69, 108, 168.98, 0, 0},
            new[] {127, 0, 0, -20, 0, 24.29},
            new[] {0, Math.PI/2, -Math.PI/2, Math.PI, 0, 0},
            new double[] { -180, -180, -180, -180, -180, -360 },
            new double[] { 180, 90, 180, 180, 180, 360 }
        )
    };

    private static readonly Dictionary<string, double[]> _toolTable = new(StringComparer.OrdinalIgnoreCase)
    {
        ["null"]  = new[] {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        ["tool0"] = new[] {1.41421356, 0.0, 2.41421356, 0.70710678, 0.0, 0.70710678}
    };

    public static Manipulator6DoF CreateManipulator(string? robotName = null, string? toolName = null)
    {
        var r = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
        var t = string.IsNullOrWhiteSpace(toolName) ? "null" : toolName;
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

    public enum ToolReferenceAxis { FlangeY = 0, FlangeX = 1 }
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

    // Forward (現行混成MDHロジック)
    public double[,] Forward(double[] q, int verbose = 0)
    {
        if (q.Length != 6) throw new ArgumentException("q must have 6 elements");
        var alpha = _mdh.Alpha; var a = _mdh.A; var d = _mdh.D; var offset = _mdh.Offset;

        var origins = new double[8][];
        var axes = new double[8][,];
        origins[0] = new[] {0.0, 0.0, 0.0};
        axes[0] = new double[,] {{1,0,0},{0,1,0},{0,0,1}};

        for (int i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            double[] Xp = {axes[i][0,0], axes[i][0,1], axes[i][0,2]};
            double[] Yp = {axes[i][1,0], axes[i][1,1], axes[i][1,2]};
            double[] Zp = {axes[i][2,0], axes[i][2,1], axes[i][2,2]};

            var Xi = new double[3];
            Xi[0] = ct*Xp[0] + st*(ca*Yp[0] + sa*Zp[0]);
            Xi[1] = ct*Xp[1] + st*(ca*Yp[1] + sa*Zp[1]);
            Xi[2] = ct*Xp[2] + st*(ca*Yp[2] + sa*Zp[2]);

            var Yi = new double[3];
            Yi[0] = -st*Xp[0] + ct*(ca*Yp[0] + sa*Zp[0]);
            Yi[1] = -st*Xp[1] + ct*(ca*Yp[1] + sa*Zp[1]);
            Yi[2] = -st*Xp[2] + ct*(ca*Yp[2] + sa*Zp[2]);

            var Zi = new double[3];
            Zi[0] = -sa*Yp[0] + ca*Zp[0];
            Zi[1] = -sa*Yp[1] + ca*Zp[1];
            Zi[2] = -sa*Yp[2] + ca*Zp[2];

            axes[i+1] = new double[,]
            {
                {Xi[0], Xi[1], Xi[2]},
                {Yi[0], Yi[1], Yi[2]},
                {Zi[0], Zi[1], Zi[2]}
            };

            origins[i+1] = new double[3];
            origins[i+1][0] = origins[i][0] + a[i] * Xp[0] + d[i] * Zi[0];
            origins[i+1][1] = origins[i][1] + a[i] * Xp[1] + d[i] * Zi[1];
            origins[i+1][2] = origins[i][2] + a[i] * Xp[2] + d[i] * Zi[2];
        }

        var T06 = new double[4,4];
        for (int r=0; r<3; r++)
        {
            T06[r,0] = axes[6][0,r];
            T06[r,1] = axes[6][1,r];
            T06[r,2] = axes[6][2,r];
            T06[r,3] = origins[6][r];
        }
        T06[3,0]=0; T06[3,1]=0; T06[3,2]=0; T06[3,3]=1;

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
            for (int i=0;i<=6;i++)
                Console.WriteLine($"O{i}: ({origins[i][0]:F3}, {origins[i][1]:F3}, {origins[i][2]:F3})");
            Console.WriteLine($"Tool: ({origins[7][0]:F3}, {origins[7][1]:F3}, {origins[7][2]:F3})");
        }
        if (verbose == 2)
        {
            Console.WriteLine("---- Axes (World Directions) ----");
            for (int i=1;i<=6;i++)
                Console.WriteLine($"Frame {i} X=({axes[i][0,0]:F3},{axes[i][0,1]:F3},{axes[i][0,2]:F3}) " +
                                  $"Y=({axes[i][1,0]:F3},{axes[i][1,1]:F3},{axes[i][1,2]:F3}) " +
                                  $"Z=({axes[i][2,0]:F3},{axes[i][2,1]:F3},{axes[i][2,2]:F3})");
        }
        return T06;
    }

    // ================== InverseGeometric (Exact 2R reduction with a4,d4 kept intact – NO MDH change) ==================
    // 目的: 物理 MDH (a4≠0, d4≠0) を変更せず、a4,d4 を Lw,psi へ内部変換して q2,q3 を厳密 2R 化。
    // 近似は一切行わない (浮動小数点丸めのみ)。既存ユーティリティ (ComputeW2, Forward, PoseMatch 等) を再利用。
    // 旧/他方式との衝突を避けるためこのメソッド全体を既存 InverseGeometric と入れ替えてください。
    public double[][] InverseGeometric(double[,] T_target, out bool[] flags, int verbose = 0)
    {
        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset;

        // 固定リンク長 / オフセット
        double a3 = a[2];
        double a4 = a[3];
        double d4 = d[3];
        double d6 = d[5];

        // a4,d4 の厳密合成（位相回転)
        double Lw  = Math.Sqrt(a4 * a4 + d4 * d4);
        double psi = Math.Atan2(d4, a4); // q3_eff = q3 + psi

        // 目標フランジ姿勢から手首中心 Pw
        double Px = T_target[0, 3], Py = T_target[1, 3], Pz = T_target[2, 3];
        double ZX = T_target[0, 2], ZY = T_target[1, 2], ZZ = T_target[2, 2];
        double PwX = Px - d6 * ZX;
        double PwY = Py - d6 * ZY;
        double PwZ = Pz - d6 * ZZ;

        if (verbose >= 2)
        {
            Console.WriteLine("---- Target (Flange) ----");
            Console.WriteLine($"P = ({Px:F3},{Py:F3},{Pz:F3})  Z=({ZX:F3},{ZY:F3},{ZZ:F3})");
            Console.WriteLine($"Pw= ({PwX:F3},{PwY:F3},{PwZ:F3})  (a4={a4:F3}, d4={d4:F3}, Lw={Lw:F3}, psi={psi*180/Math.PI:F3}°)");
        }

        var solutions = new List<double[]>();

        // q1 候補（標準 2 枝）
        var q1cands = new[]
        {
            Math.Atan2(PwY, PwX),
            RMLib.NormalizeAngleSigned(Math.Atan2(PwY, PwX) + Math.PI)
        };

        foreach (var q1_try in q1cands)
        {
            double q1 = RMLib.NormalizeAngleSigned(q1_try);
            if (!WithinLimit(0, q1)) continue;
            if (verbose >= 2) Console.WriteLine($"\n-- q1={q1*180/Math.PI:F3} deg");

            // q2 roots: g(q2) = (vx - a3)^2 + vy^2 - Lw^2 = 0
            var q2Roots = SolveQ2ByRadius(q1, PwX, PwY, PwZ, a3, Lw, 61, verbose);
            if (q2Roots.Count == 0)
            {
                if (verbose >= 2) Console.WriteLine("   (no q2 roots)");
                continue;
            }

            foreach (var q2 in q2Roots)
            {
                double q2n = RMLib.NormalizeAngleSigned(q2);
                if (!WithinLimit(1, q2n)) continue;

                // 再計算 W2
                var W2 = ComputeW2(q1, q2n, PwX, PwY, PwZ);
                double vx = W2[0], vy = W2[1], vz = W2[2];

                // 円残差確認
                double dx = vx - a3;
                double g = dx * dx + vy * vy - Lw * Lw;
                if (Math.Abs(g) > 1e-2) // 許容: 単位 mm^2 想定。必要に応じ調整
                {
                    if (verbose >= 3) Console.WriteLine($"   reject q2={q2n*180/Math.PI:F3} |g|={g:F4}");
                    continue;
                }

                // 有効肘角 (q3_eff) → 物理 q3
                double q3_eff = Math.Atan2(vy, dx);       // = q3 + psi
                double q3 = RMLib.NormalizeAngleSigned(q3_eff - psi);
                if (!WithinLimit(2, q3)) continue;

                if (verbose >= 3)
                    Console.WriteLine($"   q2={q2n*180/Math.PI:F3} q3={q3*180/Math.PI:F3} (eff={q3_eff*180/Math.PI:F3})");

                // R03 構築
                var T01 = RMLib.MDHTransform(q1 + off[0], d[0], a[0], _mdh.Alpha[0]);
                var T12 = RMLib.MDHTransform(q2n + off[1], d[1], a[1], _mdh.Alpha[1]);
                var T23 = RMLib.MDHTransform(q3 + off[2], d[2], a[2], _mdh.Alpha[2]);
                var T03 = RMLib.MatMul4x4(RMLib.MatMul4x4(T01, T12), T23);

                // 物理 a4,d4 を含む第4リンクまで（q4 未知なのでここでは“q4=0”軸基底で先に手首ローテーション抽出）
                var T34_q4zero = RMLib.MDHTransform(0 + off[3], d[3], a[3], _mdh.Alpha[3]);
                var T03q4zero = RMLib.MatMul4x4(T03, T34_q4zero);

                // R36 = (T03 with q4=0)^{-1} * T_target
                var T03_inv = RMLib.InvertHomogeneous4x4(T03q4zero);
                var T36 = RMLib.MatMul4x4(T03_inv, T_target);

                // 手首姿勢抽出（非球面でも回転抽出は回転成分だけ利用なので OK）
                double cos_q5 = T36[1, 2];
                if (cos_q5 < -1.0 || cos_q5 > 1.0) continue;

                foreach (var q5cand in new[] { Math.Acos(cos_q5), -Math.Acos(cos_q5) })
                {
                    double q5 = RMLib.NormalizeAngleSigned(q5cand);
                    double s5 = Math.Sin(q5);
                    double q4, q6;
                    if (Math.Abs(s5) > 1e-6)
                    {
                        q4 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 2] / s5, T36[0, 2] / s5));
                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[1, 1] / s5, -T36[1, 0] / s5));
                    }
                    else
                    {
                        // 特異: q5 ≈ 0 → q4 任意。ここでは 0 にし q6 を簡易計算。
                        q4 = 0;
                        q6 = RMLib.NormalizeAngleSigned(Math.Atan2(T36[2, 1], T36[2, 0]));
                    }

                    if (!WithinLimit(3, q4) || !WithinLimit(4, q5) || !WithinLimit(5, q6))
                        continue;

                    var cand = new double[6];
                    cand[0] = RMLib.NormalizeAngleSigned(q1 - off[0]);
                    cand[1] = RMLib.NormalizeAngleSigned(q2n - off[1]);
                    cand[2] = RMLib.NormalizeAngleSigned(q3 - off[2]);
                    cand[3] = RMLib.NormalizeAngleSigned(q4 - off[3]);
                    cand[4] = RMLib.NormalizeAngleSigned(q5 - off[4]);
                    cand[5] = RMLib.NormalizeAngleSigned(q6 - off[5]);

                    if (!WithinLimits(cand)) continue;
                    if (IsDuplicate(solutions, cand, 1e-3)) continue;

                    // Forward 検証
                    var Tchk = Forward(cand, 0);
                    if (!PoseMatch(Tchk, T_target))
                    {
                        if (verbose >= 3) Console.WriteLine("   (reject: FWD mismatch)");
                        continue;
                    }

                    solutions.Add(cand);
                    if (verbose >= 2)
                        Console.WriteLine("   ACCEPT q=[" +
                            string.Join(", ", cand.Select(v => (v * 180.0 / Math.PI).ToString("F2"))) + "]");
                }
            }
        }

        flags = Enumerable.Repeat(true, solutions.Count).ToArray();
        if (verbose >= 1)
            Console.WriteLine($"[InverseGeometric] Solutions={solutions.Count}");
        return solutions.ToArray();
    }

    // ---- g(q2) = (vx - a3)^2 + vy^2 - Lw^2 root 検索 (1D) ----
    private List<double> SolveQ2ByRadius(
        double q1, double PwX, double PwY, double PwZ,
        double a3, double Lw,
        int samples, int verbose)
    {
        var roots = new List<double>();
        double q2Min = _minAnglesRad[1], q2Max = _maxAnglesRad[1];
        double step = (q2Max - q2Min) / (samples - 1);

        double prevQ = double.NaN, prevG = double.NaN;

        for (int i = 0; i < samples; i++)
        {
            double q2 = q2Min + step * i;
            var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
            double g = (W2[0] - a3) * (W2[0] - a3) + W2[1] * W2[1] - Lw * Lw;

            if (i > 0 && prevG * g <= 0.0)
            {
                double r = RefineQ2RootSecant(q1, prevQ, q2, PwX, PwY, PwZ, a3, Lw);
                if (double.IsFinite(r))
                {
                    var Wt = ComputeW2(q1, r, PwX, PwY, PwZ);
                    double gt = (Wt[0] - a3) * (Wt[0] - a3) + Wt[1] * Wt[1] - Lw * Lw;
                    if (Math.Abs(gt) < 1e-3)
                    {
                        r = RMLib.NormalizeAngleSigned(r);
                        if (!roots.Any(x => Math.Abs(RMLib.NormalizeAngleSigned(x - r)) < 1e-3))
                            roots.Add(r);
                    }
                }
            }
            prevQ = q2; prevG = g;
        }

        if (roots.Count == 0)
        {
            // 近傍最小 |g| を許容採用
            double bestQ = 0, bestAbs = double.MaxValue;
            for (int i = 0; i < samples; i++)
            {
                double q2 = q2Min + step * i;
                var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
                double g = (W2[0] - a3) * (W2[0] - a3) + W2[1] * W2[1] - Lw * Lw;
                double ag = Math.Abs(g);
                if (ag < bestAbs) { bestAbs = ag; bestQ = q2; }
            }
            if (bestAbs < 5e-2) // 閾値調整可能
            {
                roots.Add(bestQ);
                if (verbose >= 2)
                    Console.WriteLine($"   (near-root adopt q2={bestQ * 180 / Math.PI:F2} deg |g|={bestAbs:E3})");
            }
        }

        if (verbose >= 2 && roots.Count > 0)
            Console.WriteLine("   q2 roots (deg): " + string.Join(", ", roots.Select(r => (r * 180 / Math.PI).ToString("F3"))));
        return roots;
    }

    private double RefineQ2RootSecant(double q1, double q2a, double q2b,
                                       double PwX, double PwY, double PwZ,
                                       double a3, double Lw,
                                       int maxIter = 25, double tol = 1e-6)
    {
        double G(double q2)
        {
            var W = ComputeW2(q1, q2, PwX, PwY, PwZ);
            return (W[0] - a3) * (W[0] - a3) + W[1] * W[1] - Lw * Lw;
        }

        double fa = G(q2a);
        double fb = G(q2b);
        double A = q2a, B = q2b;

        for (int i = 0; i < maxIter; i++)
        {
            if (Math.Abs(fb - fa) < 1e-14) break;
            double C = B - fb * (B - A) / (fb - fa);
            double fc = G(C);
            A = B; fa = fb;
            B = C; fb = fc;
            if (Math.Abs(fc) < tol) return C;
        }
        return B;
    }

    // ---- ComputeW2 (既存再利用) ----
    private double[] ComputeW2(double q1, double q2, double PwX, double PwY, double PwZ)
    {
        var a = _mdh.A; var d = _mdh.D; var off = _mdh.Offset; var alpha = _mdh.Alpha;
        var T01 = RMLib.DHTransform(q1 + off[0], d[0], a[0], alpha[0]);
        var T12 = RMLib.DHTransform(q2 + off[1], d[1], a[1], alpha[1]);
        var T02 = RMLib.MatMul4x4(T01, T12);

        double O2x = T02[0,3], O2y = T02[1,3], O2z = T02[2,3];
        double[] X2 = { T02[0,0], T02[1,0], T02[2,0] };
        double[] Y2 = { T02[0,1], T02[1,1], T02[2,1] };
        double[] Z2 = { T02[0,2], T02[1,2], T02[2,2] };

        double Wx = PwX - O2x;
        double Wy = PwY - O2y;
        double Wz = PwZ - O2z;

        double vx = Wx*X2[0] + Wy*X2[1] + Wz*X2[2];
        double vy = Wx*Y2[0] + Wy*Y2[1] + Wz*Y2[2];
        double vz = Wx*Z2[0] + Wy*Z2[1] + Wz*Z2[2];
        return new[] { vx, vy, vz };
    }

    // ================= Jacobian IK (既存) =================
    public double[] InverseJacobian(double[,] T_target, double[] q_initial,
                                    out bool success, int maxIterations = 600,
                                    double tolerance = 1e-6, double alpha = 0.1, int verbose = 0)
    {
        var q = (double[])q_initial.Clone();
        var dX = new double[6];

        for (int iter=0; iter<maxIterations; iter++)
        {
            var Tc = Forward(q, 0);
            dX[0] = T_target[0,3] - Tc[0,3];
            dX[1] = T_target[1,3] - Tc[1,3];
            dX[2] = T_target[2,3] - Tc[2,3];

            double[] n_c = { Tc[0,0], Tc[1,0], Tc[2,0] };
            double[] o_c = { Tc[0,1], Tc[1,1], Tc[2,1] };
            double[] a_c = { Tc[0,2], Tc[1,2], Tc[2,2] };
            double[] n_t = { T_target[0,0], T_target[1,0], T_target[2,0] };
            double[] o_t = { T_target[0,1], T_target[1,1], T_target[2,1] };
            double[] a_t = { T_target[0,2], T_target[1,2], T_target[2,2] };

            var cn = RMLib.Cross3x3(n_c, n_t);
            var co = RMLib.Cross3x3(o_c, o_t);
            var ca = RMLib.Cross3x3(a_c, a_t);
            dX[3] = 0.5*(cn[0] + co[0] + ca[0]);
            dX[4] = 0.5*(cn[1] + co[1] + ca[1]);
            dX[5] = 0.5*(cn[2] + co[2] + ca[2]);

            double errSq = dX.Sum(v => v*v);
            if (errSq < tolerance * tolerance)
            {
                success = true;
                return q;
            }

            var J = CalculateJacobian(q);
            var JT = RMLib.TransposeRM(J);
            var dq = RMLib.MatVecMul(JT, dX);

            for (int j=0;j<6;j++)
            {
                q[j] = RMLib.NormalizeAngleSigned(q[j] + alpha * dq[j]);
                if (q[j] < _minAnglesRad[j]) q[j] = _minAnglesRad[j];
                if (q[j] > _maxAnglesRad[j]) q[j] = _maxAnglesRad[j];
            }
        }
        success = false;
        return q;
    }

    private double[,] CalculateJacobian(double[] q)
    {
        var J = new double[6,6];
        var T_eff = Forward(q, 0);
        double[] p_eff = { T_eff[0,3], T_eff[1,3], T_eff[2,3] };

        var alpha=_mdh.Alpha; var a=_mdh.A; var d=_mdh.D; var off=_mdh.Offset;

        var origins = new double[7][];
        var axes = new double[7][,];
        origins[0] = new[] {0.0,0.0,0.0};
        axes[0] = new double[,] {{1,0,0},{0,1,0},{0,0,1}};

        for (int i=0;i<6;i++)
        {
            var theta = q[i] + off[i];
            double ct=Math.Cos(theta), st=Math.Sin(theta);
            double ca=Math.Cos(alpha[i]), sa=Math.Sin(alpha[i]);

            double[] Xp = { axes[i][0,0], axes[i][0,1], axes[i][0,2] };
            double[] Yp = { axes[i][1,0], axes[i][1,1], axes[i][1,2] };
            double[] Zp = { axes[i][2,0], axes[i][2,1], axes[i][2,2] };

            var Xi = new double[3];
            Xi[0]=ct*Xp[0]+st*(ca*Yp[0]+sa*Zp[0]);
            Xi[1]=ct*Xp[1]+st*(ca*Yp[1]+sa*Zp[1]);
            Xi[2]=ct*Xp[2]+st*(ca*Yp[2]+sa*Zp[2]);

            var Yi = new double[3];
            Yi[0]=-st*Xp[0]+ct*(ca*Yp[0]+sa*Zp[0]);
            Yi[1]=-st*Xp[1]+ct*(ca*Yp[1]+sa*Zp[1]);
            Yi[2]=-st*Xp[2]+ct*(ca*Yp[2]+sa*Zp[2]);

            var Zi = new double[3];
            Zi[0]=-sa*Yp[0]+ca*Zp[0];
            Zi[1]=-sa*Yp[1]+ca*Zp[1];
            Zi[2]=-sa*Yp[2]+ca*Zp[2];

            axes[i+1] = new double[,] {
                { Xi[0], Xi[1], Xi[2] },
                { Yi[0], Yi[1], Yi[2] },
                { Zi[0], Zi[1], Zi[2] }
            };
            origins[i+1] = new double[3];
            origins[i+1][0] = origins[i][0] + a[i]*Xp[0] + d[i]*Zi[0];
            origins[i+1][1] = origins[i][1] + a[i]*Xp[1] + d[i]*Zi[1];
            origins[i+1][2] = origins[i][2] + a[i]*Xp[2] + d[i]*Zi[2];

            double[] zAxisPrev = { axes[i][2,0], axes[i][2,1], axes[i][2,2] };
            var p_i = origins[i];
            double[] dp = { p_eff[0]-p_i[0], p_eff[1]-p_i[1], p_eff[2]-p_i[2] };
            var Jv = RMLib.Cross3x3(zAxisPrev, dp);

            J[0,i]=Jv[0]; J[1,i]=Jv[1]; J[2,i]=Jv[2];
            J[3,i]=zAxisPrev[0]; J[4,i]=zAxisPrev[1]; J[5,i]=zAxisPrev[2];
        }
        return J;
    }

    // ==== Utility ====
    private bool WithinLimit(int idx, double valRad) =>
        valRad >= _minAnglesRad[idx] && valRad <= _maxAnglesRad[idx];

    private bool WithinLimits(double[] q)
    {
        for (int i=0;i<6;i++)
            if (!WithinLimit(i,q[i])) return false;
        return true;
    }

    private static bool IsDuplicate(List<double[]> sols, double[] cand, double tol)
    {
        foreach (var s in sols)
        {
            double sum=0;
            for (int i=0;i<6;i++)
            {
                double d = RMLib.NormalizeAngleSigned(s[i] - cand[i]);
                sum += d*d;
            }
            if (sum < tol*tol) return true;
        }
        return false;
    }

    private static bool PoseMatch(double[,] A, double[,] B, double posTol=1e-3, double angTol=1e-3)
    {
        double dp =
            (A[0,3]-B[0,3])*(A[0,3]-B[0,3]) +
            (A[1,3]-B[1,3])*(A[1,3]-B[1,3]) +
            (A[2,3]-B[2,3])*(A[2,3]-B[2,3]);
        if (dp > posTol*posTol) return false;

        double tr =
            A[0,0]*B[0,0] + A[1,0]*B[1,0] + A[2,0]*B[2,0] +
            A[0,1]*B[0,1] + A[1,1]*B[1,1] + A[2,1]*B[2,1] +
            A[0,2]*B[0,2] + A[1,2]*B[1,2] + A[2,2]*B[2,2];
        double cosAng = (tr - 1.0)*0.5;
        cosAng = Math.Clamp(cosAng, -1.0, 1.0);
        double ang = Math.Acos(cosAng);
        return ang < angTol;
    }

    // Tool tilt / pose helpers (既存そのまま)
    public (double Rx, double Ry, double Rz) GetToolTiltAngles()
    {
        double zx=_tool[3], zy=_tool[4], zz=_tool[5];
        double n = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (n < 1e-12) return (0,0,0);
        zx/=n; zy/=n; zz/=n;

        double sinRy = zx; sinRy = Math.Clamp(sinRy, -1, 1);
        double Ry = Math.Asin(sinRy);
        double cosRy = Math.Cos(Ry);

        double Rx;
        if (Math.Abs(cosRy) < 1e-9)
            Rx = 0.0;
        else
        {
            double sinRx = -zy / cosRy;
            double cosRx =  zz / cosRy;
            double mag = Math.Sqrt(sinRx*sinRx + cosRx*cosRx);
            if (mag > 1e-12){ sinRx/=mag; cosRx/=mag; }
            Rx = Math.Atan2(sinRx, cosRx);
        }
        double Rz = 0.0;
        Rx = RMLib.NormalizeAngleSigned(Rx);
        Ry = RMLib.NormalizeAngleSigned(Ry);
        return (Rx,Ry,Rz);
    }

    public (double tx,double ty,double tz,double Rx,double Ry,double Rz) GetToolPoseDecomposed()
    {
        var (Rx,Ry,Rz) = GetToolTiltAngles();
        return (_tool[0], _tool[1], _tool[2], Rx, Ry, Rz);
    }

    public double[,] GetToolTiltTransform()
    {
        var (Rx,Ry,Rz) = GetToolTiltAngles();
        double sx=Math.Sin(Rx), cx=Math.Cos(Rx);
        double sy=Math.Sin(Ry), cy=Math.Cos(Ry);
        double r00=cy, r01=0, r02=sy;
        double r10=sx*sy, r11=cx, r12=-sx*cy;
        double r20=-cx*sy, r21=sx, r22=cx*cy;

        var T = RMLib.Identity4x4();
        T[0,0]=r00; T[0,1]=r01; T[0,2]=r02;
        T[1,0]=r10; T[1,1]=r11; T[1,2]=r12;
        T[2,0]=r20; T[2,1]=r21; T[2,2]=r22;
        T[0,3]=_tool[0]; T[1,3]=_tool[1]; T[2,3]=_tool[2];
        return T;
    }

    public (double ax,double ay,double az,double angle) GetToolZAlignment()
    {
        double zx=_tool[3], zy=_tool[4], zz=_tool[5];
        double n = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (n < 1e-12) return (0,0,1,0);
        zx/=n; zy/=n; zz/=n;
        double dot = Math.Clamp(zz, -1.0, 1.0);
        double angle = Math.Acos(dot);
        if (angle < 1e-12) return (0,0,1,0);
        double ax = zy, ay = -zx, az=0;
        double m = Math.Sqrt(ax*ax + ay*ay);
        if (m < 1e-12) return (1,0,0,Math.PI);
        return (ax/m, ay/m, 0, angle);
    }

    public double[] GetToolInversePose()
    {
        var (TxF,TyF,TzF,RxF,RyF,RzF) = GetToolPoseDecomposed();
        double sx=Math.Sin(RxF), cx=Math.Cos(RxF);
        double sy=Math.Sin(RyF), cy=Math.Cos(RyF);
        double r00=cy, r01=0.0, r02=sy;
        double r10=sx*sy, r11=cx,  r12=-sx*cy;
        double r20=-cx*sy,r21=sx,  r22=cx*cy;

        double tInvX = -(r00*TxF + r10*TyF + r20*TzF);
        double tInvY = -(r01*TxF + r11*TyF + r21*TzF);
        double tInvZ = -(r02*TxF + r12*TyF + r22*TzF);

        double RxInv = RMLib.NormalizeAngleSigned(-RxF);
        double RyInv = RMLib.NormalizeAngleSigned(-RyF);
        double RzInv = RMLib.NormalizeAngleSigned(-RzF);

        return new[] { tInvX, tInvY, tInvZ, RxInv, RyInv, RzInv };
    }

    // === (Add back) Tool transform + ForwardWithTool / InverseGeometricTcp / InverseJacobianTcp ===
    /// <summary>
    /// Build flange->tool transform (optional roll about tool Z).
    /// _tool[0..2] = translation (flange frame)
    /// _tool[3..5] = tool local +Z direction in flange frame (need not be unit)
    /// rollOverride: radians to rotate X/Y about tool Z (null => 0)
    /// </summary>
    public double[,] GetToolTransform(double? rollOverride = null)
    {
        double tx = _tool[0], ty = _tool[1], tz = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];
        double roll = rollOverride ?? 0.0;

        // Normalize Z
        double nZ = Math.Sqrt(zx*zx + zy*zy + zz*zz);
        if (nZ < 1e-14)
        {
            // Fallback identity
            var I = RMLib.Identity4x4();
            I[0,3] = tx; I[1,3] = ty; I[2,3] = tz;
            return I;
        }
        zx/=nZ; zy/=nZ; zz/=nZ;

        // Choose a reference minimally aligned with Z
        double ax = Math.Abs(zx), ay = Math.Abs(zy), az = Math.Abs(zz);
        double[] refVec;
        if (ax <= ay && ax <= az)      refVec = new[]{0.0,1.0,0.0};
        else if (ay <= ax && ay <= az) refVec = new[]{1.0,0.0,0.0};
        else                           refVec = new[]{1.0,0.0,0.0}; // if Z near global Z

        // X0 = ref × Z
        double x0x = refVec[1]*zz - refVec[2]*zy;
        double x0y = refVec[2]*zx - refVec[0]*zz;
        double x0z = refVec[0]*zy - refVec[1]*zx;
        double nX = Math.Sqrt(x0x*x0x + x0y*x0y + x0z*x0z);
        if (nX < 1e-12)
        {
            // Degenerate fallback
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
            x0x=Xx; x0y=Xy; x0z=Xz;
            y0x=Yx; y0y=Yy; y0z=Yz;
        }

        var T = RMLib.Identity4x4();
        T[0,0]=x0x; T[1,0]=x0y; T[2,0]=x0z;
        T[0,1]=y0x; T[1,1]=y0y; T[2,1]=y0z;
        T[0,2]=zx;  T[1,2]=zy;  T[2,2]=zz;
        T[0,3]=tx;  T[1,3]=ty;  T[2,3]=tz;
        return T;
    }

    /// <summary>
    /// Forward kinematics of TCP (flange + tool) with optional roll override (about tool Z).
    /// </summary>
    public double[,] ForwardWithTool(double[] q, double? rollOverride = null, int verbose = 0)
    {
        var Tflange = Forward(q, verbose);
        var Ttool   = GetToolTransform(rollOverride);
        return RMLib.MatMul4x4(Tflange, Ttool);
    }

    /// <summary>
    /// Geometric IK to reach a TCP pose (uses current geometric flange IK + tool inverse).
    /// </summary>
    public double[][] InverseGeometricTcp(double[,] T_tcp_target, out bool[] flags,
                                      double? rollOverride = null, int verbose = 0)
    {
        var T_tool = GetToolTransform(rollOverride);
        var T_tool_inv = RMLib.InvertHomogeneous4x4(T_tool);
        var T_flange_target = RMLib.MatMul4x4(T_tcp_target, T_tool_inv);
        return InverseGeometric(T_flange_target, out flags, verbose);
    }

    /// <summary>
    /// Jacobian IK for TCP target (tool separated).
    /// </summary>
    public double[] InverseJacobianTcp(double[,] T_tcp_target, double[] q_initial,
                                   out bool success, double? rollOverride = null,
                                   int maxIterations = 600, double tolerance = 1e-6,
                                   double alpha = 0.1, int verbose = 0)
    {
        var T_tool = GetToolTransform(rollOverride);
        var T_tool_inv = RMLib.InvertHomogeneous4x4(T_tool);
        var T_flange_target = RMLib.MatMul4x4(T_tcp_target, T_tool_inv);
        return InverseJacobian(T_flange_target, q_initial, out success,
                           maxIterations, tolerance, alpha, verbose);
    }
}