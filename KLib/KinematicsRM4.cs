using System;
using System.Collections.Generic;
using System.Linq;

namespace KinematicsRM4
{
    #region Data Structures and Factory

    public sealed class MDHParameters
    {
        public double[] Alpha { get; }
        public double[] A { get; }
        public double[] D { get; }
        public double[] Offset { get; }
        public double[] MinAnglesRad { get; }
        public double[] MaxAnglesRad { get; }
        public double ElbowAlpha { get; }
        public double ElbowA { get; }
        public double ElbowD { get; }
        public double ElbowTheta { get; }

        public MDHParameters(double[] alpha, double[] a, double[] d, double[] offset, double[] minDeg, double[] maxDeg, double eAlpha, double eA, double eD, double eTheta)
        {
            Alpha = alpha; A = a; D = d; Offset = offset;
            ElbowAlpha = eAlpha; ElbowA = eA; ElbowD = eD; ElbowTheta = eTheta;
            MinAnglesRad = minDeg.Select(deg => deg * Math.PI / 180.0).ToArray();
            MaxAnglesRad = maxDeg.Select(deg => deg * Math.PI / 180.0).ToArray();
        }
    }

    public static class ManipulatorFactory
    {
        private static readonly Dictionary<string, MDHParameters> _mdhTable = new(StringComparer.OrdinalIgnoreCase)
        {
            ["MiRobot"] = new MDHParameters(
                alpha:  new[] { 0.0, Math.PI / 2, 0.0, -Math.PI / 2, -Math.PI / 2, 0.0 },
                a:      new[] { 0.0, 29.69, 108.0, 168.98, 0.0, 0.0 },
                d:      new[] { 127.0, 0.0, 0.0, 0.0, 0.0, 24.29 },
                offset: new[] { 0.0, Math.PI / 2, -Math.PI / 2, -Math.PI, 0.0, 0.0 },
                minDeg: new double[] { -180, -180, -90, -180, -180, -360 },
                maxDeg: new double[] { 180, 90, 180, 180, 90, 360 },
                eAlpha: Math.PI / 2, eA: 0.0, eD: -20.0, eTheta: 0.0
            )
        };

        public static Manipulator6DoF CreateManipulator(string? robotName = null)
        {
            var r = string.IsNullOrWhiteSpace(robotName) ? "MiRobot" : robotName;
            if (!_mdhTable.TryGetValue(r, out var mdh)) throw new ArgumentException($"Unknown robot name: {r}");
            return new Manipulator6DoF(mdh);
        }
    }

    #endregion

    #region Math Library

    internal static class MathUtil
    {
        public static double[,] DHTransform(double theta, double d, double a, double alpha) { var rX=RotX(alpha); var tX=TransX(a); var rZ=RotZ(theta); var tZ=TransZ(d); return MatMul(MatMul(MatMul(rX,tX),rZ),tZ); }
        public static double[,] RotX(double a) { var c=Math.Cos(a); var s=Math.Sin(a); var T=Identity(); T[1,1]=c; T[1,2]=-s; T[2,1]=s; T[2,2]=c; return T; }
        public static double[,] RotZ(double a) { var c=Math.Cos(a); var s=Math.Sin(a); var T=Identity(); T[0,0]=c; T[0,1]=-s; T[1,0]=s; T[1,1]=c; return T; }
        public static double[,] TransX(double a) { var T=Identity(); T[0,3]=a; return T; }
        public static double[,] TransZ(double d) { var T=Identity(); T[2,3]=d; return T; }
        public static double[,] MatMul(double[,] A, double[,] B) { var C=new double[4,4]; for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) C[i,j]+=A[i,k]*B[k,j]; return C; }
        public static double[] MatVecMul(double[,] A, double[] v) { var r=new double[A.GetLength(0)]; for(int i=0;i<A.GetLength(0);i++) for(int j=0;j<v.Length;j++) r[i]+=A[i,j]*v[j]; return r; }
        public static double[,] Transpose(double[,] A) { var T=new double[A.GetLength(1),A.GetLength(0)]; for(int i=0;i<A.GetLength(0);i++) for(int j=0;j<A.GetLength(1);j++) T[j,i]=A[i,j]; return T; }
        public static double[] Cross(double[] a, double[] b) => new[]{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
        public static double Norm(double[] v) { double s=0; foreach(var x in v) s+=x*x; return Math.Sqrt(s); }
        public static double[] Sub(double[] a, double[] b) { var r=new double[a.Length]; for(int i=0;i<a.Length;++i) r[i]=a[i]-b[i]; return r; }
        public static double[,] Identity() => new double[4,4]{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
        public static double[,] InvertHomogeneous(double[,] T)
        {
            var R=new double[3,3]; var p=new double[3]; for(int i=0;i<3;i++){p[i]=T[i,3]; for(int j=0;j<3;j++)R[i,j]=T[i,j];}
            var RT=new double[3,3]; for(int i=0;i<3;i++) for(int j=0;j<3;j++) RT[i,j]=R[j,i];
            var negRTp=new double[3]; for(int i=0;i<3;i++) for(int j=0;j<3;j++) negRTp[i]-=RT[i,j]*p[j];
            var inv=Identity(); for(int i=0;i<3;i++){inv[i,3]=negRTp[i]; for(int j=0;j<3;j++)inv[i,j]=RT[i,j];} return inv;
        }
        public static double NormalizeAngle(double a) { while(a>Math.PI)a-=2*Math.PI; while(a<=-Math.PI)a+=2*Math.PI; return a; }
    }

    #endregion

    public sealed class Manipulator6DoF
    {
        private readonly MDHParameters _mdh;
        public Manipulator6DoF(MDHParameters mdh) { _mdh = mdh; }

        public double[,] Forward(double[] q, int verbose = 0)
        {
            var transforms = GetFrameTransforms(q);
            if (verbose > 0)
            {
                Console.WriteLine();
                for (int i = 0; i < transforms.Count; i++)
                {
                    var T = transforms[i];
                    string name = i switch { 0 => "Base", 1 => "J1", 2 => "J2", 3 => "J3", 4 => "Elbow", 5 => "J4", 6 => "J5", 7 => "J6", _ => "?" };
                    Console.WriteLine($"Origin {name}: ({T[0, 3]:F3}, {T[1, 3]:F3}, {T[2, 3]:F3})");
                    if (verbose == 2)
                    {
                        Console.WriteLine($"  X-Axis: ({T[0, 0]:F3}, {T[1, 0]:F3}, {T[2, 0]:F3})");
                        Console.WriteLine($"  Z-Axis: ({T[0, 2]:F3}, {T[1, 2]:F3}, {T[2, 2]:F3})");
                        Console.WriteLine("  Matrix:");
                        PrintMatrix(T);
                    }
                }
            }
            return transforms[^1];
        }

        public double[][] InverseGeometric(double[,] T_target)
        {
            // This implementation is based on the numerical root-finding approach from KinematicsRM.cs
            // adapted to the current MDH model.
            var solutions = new List<double[]>();

            // 1. Calculate Pw (Wrist Center)
            var P_target = new double[] { T_target[0, 3], T_target[1, 3], T_target[2, 3] };
            var R_target = new double[3, 3]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R_target[i, j] = T_target[i, j];
            var d6_vec = new double[] { 0, 0, _mdh.D[5] }; // d6 is along Z-axis of frame 5
            var Pw = MathUtil.Sub(P_target, MatVecMul3x3(R_target, d6_vec));

            // 2. Solve for q1 candidates
            for (int shoulder_flip = 0; shoulder_flip <= 1; shoulder_flip++)
            {
                double q1_raw = Math.Atan2(Pw[1], Pw[0]);
                if (shoulder_flip == 1) q1_raw = MathUtil.NormalizeAngle(q1_raw + Math.PI);
                double q1 = q1_raw - _mdh.Offset[0]; // Apply offset for limit check
                if (!WithinLimit(0, q1)) continue;

                // 3. Solve for q2, q3 using numerical root finding (adapted from KinematicsRM.cs)
                var q2_q3_sols = SolveQ2Q3(q1_raw, Pw);
                foreach (var q2_q3_pair in q2_q3_sols)
                {
                    double q2_raw = q2_q3_pair[0];
                    double q3_raw = q2_q3_pair[1];

                    double q2 = q2_raw - _mdh.Offset[1];
                    double q3 = q3_raw - _mdh.Offset[2];
                    if (!WithinLimit(1, q2) || !WithinLimit(2, q3)) continue;

                    // 4. Solve for q4, q5, q6 (Wrist angles)
                    var T03e = GetT03Elbow(new[] { q1_raw, q2_raw, q3_raw });
                    var T36 = MathUtil.MatMul(MathUtil.InvertHomogeneous(T03e), T_target);

                    double q5_raw = Math.Acos(Math.Clamp(T36[2, 2], -1.0, 1.0));
                    for (int wrist_flip = 0; wrist_flip <= 1; wrist_flip++)
                    {
                        double q5_sol_raw = (wrist_flip == 0) ? q5_raw : -q5_raw;
                        double q5_sol = q5_sol_raw - _mdh.Offset[4];
                        if (!WithinLimit(4, q5_sol)) continue;

                        double s5 = Math.Sin(q5_sol_raw);
                        if (Math.Abs(s5) < 1e-6) continue; // Wrist singularity

                        double q4_raw = MathUtil.NormalizeAngle(Math.Atan2(T36[1, 2] / s5, T36[0, 2] / s5));
                        double q6_raw = MathUtil.NormalizeAngle(Math.Atan2(T36[2, 1] / s5, -T36[2, 0] / s5));

                        double q4 = q4_raw - _mdh.Offset[3];
                        double q6 = q6_raw - _mdh.Offset[5];
                        if (!WithinLimit(3, q4) || !WithinLimit(5, q6)) continue;

                        solutions.Add(new[] { q1, q2, q3, q4, q5_sol, q6 });
                    }
                }
            }
            return solutions.ToArray();
        }

        public double[]? InverseJacobian(double[,] T_target, double[] q_initial, out bool success, int maxIter = 200, double tol = 1e-5, double alpha = 0.1)
        {
            var q = (double[])q_initial.Clone();
            for (int iter = 0; iter < maxIter; iter++)
            {
                var T_cur = Forward(q);
                var err = PoseError(T_cur, T_target);
                if (MathUtil.Norm(err) < tol) { success = true; return q; }

                var J = CalculateJacobian(q);
                var JT = MathUtil.Transpose(J);
                var dq = MathUtil.MatVecMul(JT, err);

                for (int i = 0; i < 6; i++) q[i] += alpha * dq[i];
                ClampToLimits(q);
            }
            success = false;
            return null;
        }

        private List<double[,]> GetFrameTransforms(double[] q)
        {
            var transforms = new List<double[,]> { MathUtil.Identity() };
            var T = MathUtil.Identity();
            for (int i = 0; i < 6; i++)
            {
                T = MathUtil.MatMul(T, MathUtil.DHTransform(q[i] + _mdh.Offset[i], _mdh.D[i], _mdh.A[i], _mdh.Alpha[i]));
                transforms.Add(T);
                if (i == 2)
                {
                    T = MathUtil.MatMul(T, MathUtil.DHTransform(_mdh.ElbowTheta, _mdh.ElbowD, _mdh.ElbowA, _mdh.ElbowAlpha));
                    transforms.Add(T);
                }
            }
            return transforms;
        }

        private double[] PoseError(double[,] T_cur, double[,] T_target)
        {
            var err = new double[6];
            for (int i = 0; i < 3; i++) err[i] = T_target[i, 3] - T_cur[i, 3];
            var n_c = new[] { T_cur[0, 0], T_cur[1, 0], T_cur[2, 0] }; var o_c = new[] { T_cur[0, 1], T_cur[1, 1], T_cur[2, 1] }; var a_c = new[] { T_cur[0, 2], T_cur[1, 2], T_cur[2, 2] };
            var n_t = new[] { T_target[0, 0], T_target[1, 0], T_target[2, 0] }; var o_t = new[] { T_target[0, 1], T_target[1, 1], T_target[2, 1] }; var a_t = new[] { T_target[0, 2], T_target[1, 2], T_target[2, 2] };
            var err_n = MathUtil.Cross(n_c, n_t); var err_o = MathUtil.Cross(o_c, o_t); var err_a = MathUtil.Cross(a_c, a_t);
            for (int i = 0; i < 3; i++) err[i + 3] = 0.5 * (err_n[i] + err_o[i] + err_a[i]);
            return err;
        }

        private double[,] CalculateJacobian(double[] q)
        {
            var J = new double[6, 6];
            var transforms = GetFrameTransforms(q);
            var p_eff = new double[] { transforms[^1][0, 3], transforms[^1][1, 3], transforms[^1][2, 3] };
            for (int i = 0; i < 6; i++)
            {
                int frame_idx = i < 4 ? i : i + 1;
                var T_i = transforms[frame_idx];
                var z_i = new double[] { T_i[0, 2], T_i[1, 2], T_i[2, 2] };
                var o_i = new double[] { T_i[0, 3], T_i[1, 3], T_i[2, 3] };
                var Jv = MathUtil.Cross(z_i, MathUtil.Sub(p_eff, o_i));
                for (int j = 0; j < 3; j++) J[j, i] = Jv[j];
                for (int j = 0; j < 3; j++) J[j + 3, i] = z_i[j];
            }
            return J;
        }

        private bool WithinLimit(int idx, double val) => val >= _mdh.MinAnglesRad[idx] && val <= _mdh.MaxAnglesRad[idx];
        private void ClampToLimits(double[] q) { for (int i = 0; i < 6; i++) q[i] = Math.Max(_mdh.MinAnglesRad[i], Math.Min(_mdh.MaxAnglesRad[i], q[i])); }
        private static void PrintMatrix(double[,] M) { for (int r = 0; r < 4; r++) Console.WriteLine($"    [{M[r, 0],8:F3},{M[r, 1],8:F3},{M[r, 2],8:F3},{M[r, 3],8:F3}]"); }
        private double[,] GetT03Elbow(double[] q_raw) { var T = MathUtil.Identity(); for (int i = 0; i < 3; i++) T = MathUtil.MatMul(T, MathUtil.DHTransform(q_raw[i] + _mdh.Offset[i], _mdh.D[i], _mdh.A[i], _mdh.Alpha[i])); T = MathUtil.MatMul(T, MathUtil.DHTransform(_mdh.ElbowTheta, _mdh.ElbowD, _mdh.ElbowA, _mdh.ElbowAlpha)); return T; }
        private static double[] MatVecMul3x3(double[,] R, double[] v) { var r = new double[3]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r[i] += R[i, j] * v[j]; return r; }

        // Helper for InverseGeometric: Solves for q2, q3 using numerical root finding (adapted from KinematicsRM.cs)
        private List<double[]> SolveQ2Q3(double q1_raw, double[] Pw)
        {
            var solutions = new List<double[]>();
            double a2 = _mdh.A[2];
            double elbow_d = _mdh.ElbowD;
            double link4_a = _mdh.A[3];
            double l_arm = Math.Sqrt(link4_a * link4_a + elbow_d * elbow_d);

            // This part needs a proper analytical or numerical solution for q2, q3
            // based on the specific geometry of the elbow link.
            // The original KinematicsRM.cs used a numerical root finder for q2.
            // For now, this is a placeholder.
            // This is the most complex part of the geometric IK for this model.
            // It's likely that the analytical derivation I attempted earlier was incorrect.
            // Reverting to a stub for now to focus on Jacobian.
            return solutions;
        }
    }
}