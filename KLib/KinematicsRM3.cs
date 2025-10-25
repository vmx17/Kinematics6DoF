using System;
using System.Collections.Generic;
using System.Linq;
// Copilot
namespace KinematicsRM3
{
    // --- Math utilities ---
    public static class Math4
    {
        public struct Vec3
        {
            public double X, Y, Z;
            public Vec3(double x, double y, double z)
            {
                X = x; Y = y; Z = z;
            }
            public static Vec3 operator +(Vec3 a, Vec3 b) => new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
            public static Vec3 operator -(Vec3 a, Vec3 b) => new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
            public static Vec3 operator *(double s, Vec3 a) => new Vec3(s * a.X, s * a.Y, s * a.Z);
            public double Norm() => Math.Sqrt(X * X + Y * Y + Z * Z);
            public Vec3 Normalize()
            {
                var n = Norm(); return n > 0 ? new Vec3(X / n, Y / n, Z / n) : new Vec3(0, 0, 0);
            }
        }

        public class Mat4
        {
            public double[,] M = new double[4, 4];
            public Mat4()
            {
                for (int i = 0; i < 4; i++) M[i, i] = 1.0;
            }
            public static Mat4 Identity() => new Mat4();

            public static Mat4 Multiply(Mat4 A, Mat4 B)
            {
                var C = new Mat4();
                for (int i = 0; i < 4; i++)
                    for (int j = 0; j < 4; j++)
                    {
                        double s = 0;
                        for (int k = 0; k < 4; k++) s += A.M[i, k] * B.M[k, j];
                        C.M[i, j] = s;
                    }
                return C;
            }

            public static Mat4 RotX(double a)
            {
                var c = Math.Cos(a); var s = Math.Sin(a);
                var R = Identity();
                R.M[1, 1] = c; R.M[1, 2] = -s;
                R.M[2, 1] = s; R.M[2, 2] = c;
                return R;
            }
            public static Mat4 RotZ(double t)
            {
                var c = Math.Cos(t); var s = Math.Sin(t);
                var R = Identity();
                R.M[0, 0] = c; R.M[0, 1] = -s;
                R.M[1, 0] = s; R.M[1, 1] = c;
                return R;
            }
            public static Mat4 TransX(double a)
            {
                var T = Identity(); T.M[0, 3] = a; return T;
            }
            public static Mat4 TransZ(double d)
            {
                var T = Identity(); T.M[2, 3] = d; return T;
            }

            public double[,] Rotation3()
            {
                var R = new double[3, 3];
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        R[i, j] = M[i, j];
                return R;
            }

            public static Mat4 InverseRigid(Mat4 T)
            {
                var Rt = new double[3, 3];
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        Rt[i, j] = T.M[j, i];
                var t = new Vec3(T.M[0, 3], T.M[1, 3], T.M[2, 3]);
                var tInv = new Vec3(
                    -(Rt[0, 0] * t.X + Rt[0, 1] * t.Y + Rt[0, 2] * t.Z),
                    -(Rt[1, 0] * t.X + Rt[1, 1] * t.Y + Rt[1, 2] * t.Z),
                    -(Rt[2, 0] * t.X + Rt[2, 1] * t.Y + Rt[2, 2] * t.Z));
                return FromRotationTranslation(Rt, tInv);
            }

            public static Mat4 FromRotationTranslation(double[,] R3, Vec3 t)
            {
                var T = Identity();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        T.M[i, j] = R3[i, j];
                T.M[0, 3] = t.X; T.M[1, 3] = t.Y; T.M[2, 3] = t.Z;
                return T;
            }
            public Vec3 TransformPoint(Vec3 p)
            {
                double x = M[0,0] * p.X + M[0,1] * p.Y + M[0,2] * p.Z + M[0,3];
                double y = M[1,0] * p.X + M[1,1] * p.Y + M[1,2] * p.Z + M[1,3];
                double z = M[2,0] * p.X + M[2,1] * p.Y + M[2,2] * p.Z + M[2,3];
                return new Vec3(x, y, z);
            }
            public Vec3 TransformVector(Vec3 v)
            {
                double x = M[0,0] * v.X + M[0,1] * v.Y + M[0,2] * v.Z;
                double y = M[1,0] * v.X + M[1,1] * v.Y + M[1,2] * v.Z;
                double z = M[2,0] * v.X + M[2,1] * v.Y + M[2,2] * v.Z;
                return new Vec3(x, y, z);
            }
        }

        public static double[,] RzRyRx(double rz, double ry, double rx)
        {
            double cz = Math.Cos(rz), sz = Math.Sin(rz);
            double cy = Math.Cos(ry), sy = Math.Sin(ry);
            double cx = Math.Cos(rx), sx = Math.Sin(rx);

            double[,] Rz = { { cz, -sz, 0 }, { sz, cz, 0 }, { 0, 0, 1 } };
            double[,] Ry = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
            double[,] Rx = { { 1, 0, 0 }, { 0, cx, -sx }, { 0, sx, cx } };

            return Multiply3x3(Multiply3x3(Rz, Ry), Rx);
        }

        public static double[,] Multiply3x3(double[,] A, double[,] B)
        {
            var C = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 3; k++) s += A[i, k] * B[k, j];
                    C[i, j] = s;
                }
            return C;
        }

        public static double Clamp(double x, double min, double max) => Math.Max(min, Math.Min(max, x));
        public static double NormAngle(double a)
        {
            // Normalize to (-pi, pi]
            a = Math.IEEERemainder(a, 2.0 * Math.PI);
            if (a <= -Math.PI) a += 2.0 * Math.PI;
            if (a > Math.PI) a -= 2.0 * Math.PI;
            return a;
        }

        public static double[] NormalizeQ(double[] q)
        {
            var r = (double[])q.Clone();
            for (int i = 0; i < r.Length; i++) r[i] = NormAngle(r[i]);
            return r;
        }

        public static bool SameSolution(double[] a, double[] b, double tol = 1e-6)
        {
            // Compare modulo 2π by normalized angles
            var na = NormalizeQ(a);
            var nb = NormalizeQ(b);
            for (int i = 0; i < na.Length; i++)
                if (Math.Abs(na[i] - nb[i]) > tol) return false;
            return true;
        }
    }

    // --- Robot model ---
    public class RobotModel
    {
        public double[] alpha = new double[7];
        public double[] a = new double[7];
        public double[] d = new double[7];
        public double[] thetaOffset = new double[7];
        public double[] Tool = new double[6] { 0, 0, 0, 0, 0, 0 };

        public RobotModel()
        {
            alpha[0] = 0.0; a[0] = 0.0; d[1] = 127.0; thetaOffset[1] = 0.0;
            alpha[1] = Math.PI / 2.0; a[1] = 29.69; d[2] = 0.0; thetaOffset[2] = Math.PI / 2.0;
            alpha[2] = 0.0; a[2] = 108.0; d[3] = 0.0; thetaOffset[3] = -Math.PI / 2.0;
            alpha[3] = -Math.PI / 2.0; a[3] = 168.98; d[4] = 0.0; thetaOffset[4] = -Math.PI;
            alpha[4] = -Math.PI / 2.0; a[4] = 0.0; d[5] = 0.0; thetaOffset[5] = 0.0;
            alpha[5] = 0.0; a[5] = 0.0; d[6] = 24.29; thetaOffset[6] = 0.0;
        }

        public Math4.Mat4 FixedTe()
        {
            var T = Math4.Mat4.Identity();
            T = Math4.Mat4.Multiply(T, Math4.Mat4.RotX(Math.PI / 2.0));
            T = Math4.Mat4.Multiply(T, Math4.Mat4.TransZ(-20.0));
            return T;
        }

        public Math4.Mat4 ToolTransformOnSigma6()
        {
            var xt = Tool[0]; var yt = Tool[1]; var zt = Tool[2];
            var rz = Tool[3]; var ry = Tool[4]; var rx = Tool[5];
            var R = Math4.RzRyRx(rz, ry, rx);
            return Math4.Mat4.FromRotationTranslation(R, new Math4.Vec3(xt, yt, zt));
        }

        public Math4.Mat4 Ti(int i, double theta_i)
        {
            if (i < 1 || i > 6) throw new ArgumentOutOfRangeException(nameof(i));
            double th = theta_i + thetaOffset[i];

            var T = Math4.Mat4.Identity();
            T = Math4.Mat4.Multiply(T, Math4.Mat4.RotX(alpha[i - 1]));
            T = Math4.Mat4.Multiply(T, Math4.Mat4.TransX(a[i - 1]));
            T = Math4.Mat4.Multiply(T, Math4.Mat4.RotZ(th));
            T = Math4.Mat4.Multiply(T, Math4.Mat4.TransZ(d[i]));
            return T;
        }
    }

    // --- Kinematics solver ---
    public class Kinematics
    {
        private readonly RobotModel model;
        public Kinematics(RobotModel m)
        {
            model = m;
        }
        /// <summary>
        /// FK
        /// </summary>
        /// <param name="jointAngles"></param>
        /// <param name="tools"></param>
        /// <param name="verbose"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public double[,] Forward(double[] q, double[] tools = null, int verbose = 0)
        {
            if (q == null || q.Length != 6)
                throw new ArgumentException("q must be length 6");
            if (tools == null) tools = new double[6]; // {0,0,0,0,0,0}

            // Ti uses thetaOffset[i] == 0 in this analysis-optimized setup
            var T = Math4.Mat4.Identity();

            var T1 = model.Ti(1, q[0]);
            var T2 = model.Ti(2, q[1]);
            var T3 = model.Ti(3, q[2]);
            if (verbose >= 2) { PrintPose("^0T1", T1); PrintPose("^1T2", T2); PrintPose("^2T3", T3); }

            T = Math4.Mat4.Multiply(T, T1);
            T = Math4.Mat4.Multiply(T, T2);
            T = Math4.Mat4.Multiply(T, T3);

            var Te = model.FixedTe(); // RotX(+π/2) → TransZ(-20)
            if (verbose >= 2) PrintPose("^3T_e (FixedTe)", Te);
            T = Math4.Mat4.Multiply(T, Te);

            var T4 = model.Ti(4, q[3]);
            var T5 = model.Ti(5, q[4]);
            var T6 = model.Ti(6, q[5]);
            if (verbose >= 2) { PrintPose("^eT4", T4); PrintPose("^4T5", T5); PrintPose("^5T6", T6); }

            T = Math4.Mat4.Multiply(T, T4);
            T = Math4.Mat4.Multiply(T, T5);
            T = Math4.Mat4.Multiply(T, T6);

            // Tool transform ^6T_t
            var Rtool = Math4.RzRyRx(tools[3], tools[4], tools[5]); // rz→ry→rx
            var Ttool = Math4.Mat4.FromRotationTranslation(Rtool, new Math4.Vec3(tools[0], tools[1], tools[2]));
            if (verbose >= 2) PrintPose("^6T_t (tool)", Ttool);

            T = Math4.Mat4.Multiply(T, Ttool);

            if (verbose >= 2) PrintPose("^0T_t (final)", T);
            return T.M;
        }

        /// <summary>
        /// IK using geometric method
        /// </summary>
        /// <param name="T_target"></param>
        /// <param name="tools"></param>
        /// <param name="verbose"></param>
        /// <returns></returns>
        public double[][] InverseGeometric(double[,] T_target, double[] tools = null, int verbose = 0)
        {
            if (tools == null) tools = new double[6];

            // Build tool and remove: T06_target = T_target * inv(^6T_t)
            var Rtool = Math4.RzRyRx(tools[3], tools[4], tools[5]);
            var Ttool = Math4.Mat4.FromRotationTranslation(Rtool, new Math4.Vec3(tools[0], tools[1], tools[2]));
            var Ttar = FromArray(T_target);
            var T06 = Math4.Mat4.Multiply(Ttar, Math4.Mat4.InverseRigid(Ttool));

            if (verbose >= 2) { PrintPose("T_target (^0Tt)", Ttar); PrintPose("^6T_t (tool)", Ttool); PrintPose("T06_target (^0T6)", T06); }

            // Wrist center pw = p6 - d6 * z6
            var R06 = T06.Rotation3();
            var z6 = new Math4.Vec3(R06[0,2], R06[1,2], R06[2,2]);
            var p6 = new Math4.Vec3(T06.M[0,3], T06.M[1,3], T06.M[2,3]);
            var pw = p6 - model.d[6] * z6;
            if (verbose >= 2) { PrintVec3("p6", p6); PrintVec3("z6", z6); PrintVec3("pw (wrist center)", pw); }

            // q1
            double q1 = Math.Atan2(pw.Y, pw.X); // thetaOffset were zeroed for analysis

            // Project to Σ1
            var T01 = model.Ti(1, q1);
            var T10 = Math4.Mat4.InverseRigid(T01);
            var pw1 = T10.TransformPoint(pw);
            if (verbose >= 2) { PrintPose("^0T1(q1)", T01); PrintPose("^1T0", T10); PrintVec3("pw1", pw1); }

            // Planar 2-link for q2, q3: L2=a[1], L3=a[2], with d2=d3=0
            double L2 = model.a[1], L3 = model.a[2];
            double x = pw1.X, z = pw1.Z;
            double r = Math.Sqrt(x*x + z*z);
            double c3 = Math4.Clamp((r*r - L2*L2 - L3*L3) / (2*L2*L3), -1, 1);
            double s3 = Math.Sqrt(Math.Max(0, 1 - c3*c3));
            if (verbose >= 2) Console.WriteLine($"L2={L2:F6}, L3={L3:F6}, r={r:F6}, c3={c3:F6}, s3={s3:F6}");

            var sols = new List<double[]>();
            foreach (double sgn in new[] { +1.0, -1.0 }) // elbow-up/down
            {
                double theta3 = Math.Atan2(sgn * s3, c3);
                double phi = Math.Atan2(z, x);
                double psi = Math.Atan2(L3 * Math.Sin(theta3), L2 + L3 * Math.Cos(theta3));
                double theta2 = phi - psi;

                double q2 = theta2;
                double q3 = theta3;

                // Build T0e
                var T03 = Math4.Mat4.Identity();
                T03 = Math4.Mat4.Multiply(T03, model.Ti(1, q1));
                T03 = Math4.Mat4.Multiply(T03, model.Ti(2, q2));
                T03 = Math4.Mat4.Multiply(T03, model.Ti(3, q3));
                var T0e = Math4.Mat4.Multiply(T03, model.FixedTe());
                if (verbose >= 2) { PrintPose("^0T3", T03); PrintPose("^0T_e", T0e); }

                // Wrist orientation: Re6 = R0e^T * R06
                var R0e = T0e.Rotation3();
                var Rt_e0 = new double[3,3] {
            { R0e[0,0], R0e[1,0], R0e[2,0] },
            { R0e[0,1], R0e[1,1], R0e[2,1] },
            { R0e[0,2], R0e[1,2], R0e[2,2] }
        };
                var Re6 = Math4.Multiply3x3(Rt_e0, R06);
                if (verbose >= 2)
                {
                    Console.WriteLine("Re6:");
                    Console.WriteLine($"{Re6[0, 0],11:F6} {Re6[0, 1],11:F6} {Re6[0, 2],11:F6}");
                    Console.WriteLine($"{Re6[1, 0],11:F6} {Re6[1, 1],11:F6} {Re6[1, 2],11:F6}");
                    Console.WriteLine($"{Re6[2, 0],11:F6} {Re6[2, 1],11:F6} {Re6[2, 2],11:F6}");
                }

                // Extract q4–q6 from Re6 (ZYZではなく、手首の軸順に合わせた解法)
                double theta5 = Math.Atan2(Math.Sqrt(Re6[0,2]*Re6[0,2] + Re6[1,2]*Re6[1,2]), Re6[2,2]);
                foreach (int sign in new[] { +1, -1 }) // wrist flip
                {
                    double s5 = sign * Math.Sin(theta5);
                    double theta4 = Math.Atan2(Re6[1,2] / s5, Re6[0,2] / s5);
                    double theta6 = Math.Atan2(Re6[2,1] / s5, -Re6[2,0] / s5);

                    double q4 = theta4;
                    double q5 = sign * theta5;
                    double q6 = theta6;

                    var cand = Math4.NormalizeQ(new[] { q1, q2, q3, q4, q5, q6 });
                    sols.Add(cand);

                    if (verbose >= 2)
                    {
                        Console.WriteLine($"cand: [{string.Join(", ", cand.Select(v => v.ToString("F6")))}]");
                        var T06_sol = FromArray(Forward(cand, null));
                        var Tt_sol  = FromArray(Forward(cand, tools));
                        var e06 = PoseError(T06_sol.M, T06.M);
                        var et  = PoseError(Tt_sol.M,  Ttar.M);
                        double e06p = Math.Sqrt(e06[0]*e06[0] + e06[1]*e06[1] + e06[2]*e06[2]);
                        double e06r = Math.Sqrt(e06[3]*e06[3] + e06[4]*e06[4] + e06[5]*e06[5]);
                        double etp  = Math.Sqrt(et[0]*et[0]   + et[1]*et[1]   + et[2]*et[2]);
                        double etr  = Math.Sqrt(et[3]*et[3]   + et[4]*et[4]   + et[5]*et[5]);
                        Console.WriteLine($"error ^0T6: pos={e06p:E3}, rot={e06r:E3}");
                        Console.WriteLine($"error ^0Tt: pos={etp:E3}, rot={etr:E3}");
                    }
                }
            }

            if (verbose > 0) Console.WriteLine($"Geometric IK produced {sols.Count} solutions.");
            return sols.ToArray();
        }



        /// <summary>
        /// IK using Jacobian matrix
        /// </summary>
        /// <param name="T_target"></param>
        /// <param name="q_initial"></param>
        /// <param name="success"></param>
        /// <param name="tools"></param>
        /// <param name="maxIterations"></param>
        /// <param name="tolerance"></param>
        /// <param name="alpha"></param>
        /// <param name="damping"></param>
        /// <param name="verbose"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public double[] InverseJacobian(
            double[,] T_target,
            double[] q_initial,
            out bool success,
            double[]? tools = null,
            int maxIterations = 300,
            double tolerance = 1e-6,
            double alpha = 1.0,
            double damping = 1e-3,
            int verbose = 0)
        {
            if (tools == null) tools = new double[6];
            var Rtool = Math4.RzRyRx(tools[3], tools[4], tools[5]);
            var Ttool = Math4.Mat4.FromRotationTranslation(Rtool, new Math4.Vec3(tools[0], tools[1], tools[2]));
            var Ttar = FromArray(T_target);
            var T06_target = Math4.Mat4.Multiply(Ttar, Math4.Mat4.InverseRigid(Ttool));
            if (verbose >= 2) { PrintPose("T_target (^0Tt)", Ttar); PrintPose("^6T_t", Ttool); PrintPose("T06_target (^0T6)", T06_target); }

            var q = (double[])q_initial.Clone();
            success = false;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                var Tcur = FromArray(Forward(q, null)); // ^0T6
                var e = PoseError(Tcur.M, T06_target.M);
                double pos = Math.Sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
                double rot = Math.Sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]);
                double errNorm = Math.Sqrt(e.Select(x => x*x).Sum());

                if (verbose >= 2)
                {
                    Console.WriteLine($"Iter {iter}: pos={pos:E3}, rot={rot:E3}, ||e||={errNorm:E3}");
                    PrintPose("^0T6 current", Tcur);
                }
                if (errNorm < tolerance) { success = true; break; }

                var J = NumericalJacobian(q, null);
                var dq = DampedLeastSquares(J, e, damping, alpha);
                for (int i = 0; i < 6; i++) q[i] += dq[i];
            }

            if (verbose >= 2)
            {
                var T06_fin = FromArray(Forward(q, null));
                var Tt_fin  = FromArray(Forward(q, tools));
                PrintPose("^0T6 final", T06_fin);
                PrintPose("^0Tt final", Tt_fin);
                var e06 = PoseError(T06_fin.M, T06_target.M);
                var et  = PoseError(Tt_fin.M,  Ttar.M);
                Console.WriteLine($"final error ^0T6: pos={Math.Sqrt(e06[0] * e06[0] + e06[1] * e06[1] + e06[2] * e06[2]):E3}, rot={Math.Sqrt(e06[3] * e06[3] + e06[4] * e06[4] + e06[5] * e06[5]):E3}");
                Console.WriteLine($"final error ^0Tt: pos={Math.Sqrt(et[0] * et[0] + et[1] * et[1] + et[2] * et[2]):E3}, rot={Math.Sqrt(et[3] * et[3] + et[4] * et[4] + et[5] * et[5]):E3}");
            }

            return Math4.NormalizeQ(q);
        }



        // --- Helpers ---
        private static Math4.Mat4 FromArray(double[,] A)
        {
            var T = Math4.Mat4.Identity();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    T.M[i, j] = A[i, j];
            return T;
        }

        private double[] PoseError(double[,] Tcur, double[,] Ttar)
        {
            var dp = new double[] {
                Ttar[0,3]-Tcur[0,3],
                Ttar[1,3]-Tcur[1,3],
                Ttar[2,3]-Tcur[2,3]
            };
            var Rcur = new double[3,3]; var Rtar = new double[3,3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { Rcur[i, j] = Tcur[i, j]; Rtar[i, j] = Ttar[i, j]; }
            // transpose of Rcur
            var Rct = new double[3,3] {
                { Rcur[0,0], Rcur[1,0], Rcur[2,0] },
                { Rcur[0,1], Rcur[1,1], Rcur[2,1] },
                { Rcur[0,2], Rcur[1,2], Rcur[2,2] }
            };
            var Rrel = Math4.Multiply3x3(Rct, Rtar);

            double rx = 0.5 * (Rrel[2,1] - Rrel[1,2]);
            double ry = 0.5 * (Rrel[0,2] - Rrel[2,0]);
            double rz = 0.5 * (Rrel[1,0] - Rrel[0,1]);

            return new[] { dp[0], dp[1], dp[2], rx, ry, rz };
        }

        private double[,] NumericalJacobian(double[] q, double[]? tools)
        {
            int n = 6;
            var J = new double[6, n];
            var f0 = FromArray(Forward(q, tools)); // tools == null → ^0T6
            var eps = 1e-6;

            for (int i = 0; i < n; i++)
            {
                var q1 = (double[])q.Clone();
                q1[i] += eps;
                var f1 = FromArray(Forward(q1, tools));
                var e = PoseError(f1.M, f0.M);
                for (int r = 0; r < 6; r++) J[r, i] = e[r] / eps;
            }
            return J;
        }

        private double[] DampedLeastSquares(double[,] J, double[] e, double lambda, double alpha)
        {
            int m = 6, n = 6;
            var JT = new double[n, m];
            for (int i = 0; i < m; i++)
                for (int j = 0; j < n; j++)
                    JT[j, i] = J[i, j];

            var JTJ = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                {
                    double s = 0;
                    for (int k = 0; k < m; k++) s += JT[i, k] * J[k, j];
                    JTJ[i, j] = s + (i == j ? lambda * lambda : 0.0);
                }

            var JTe = new double[n];
            for (int i = 0; i < n; i++)
            {
                double s = 0;
                for (int k = 0; k < m; k++) s += JT[i, k] * e[k];
                JTe[i] = s;
            }

            var dq = SolveLinear(JTJ, JTe);
            for (int i = 0; i < n; i++) dq[i] *= alpha;
            return dq;
        }

        private double[] SolveLinear(double[,] A, double[] b)
        {
            int n = b.Length;
            var M = new double[n, n + 1];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) M[i, j] = A[i, j];
                M[i, n] = b[i];
            }

            for (int i = 0; i < n; i++)
            {
                double piv = M[i, i];
                if (Math.Abs(piv) < 1e-12) piv = 1e-12;
                for (int j = i; j <= n; j++) M[i, j] /= piv;
                for (int k = 0; k < n; k++)
                {
                    if (k == i) continue;
                    double f = M[k, i];
                    for (int j = i; j <= n; j++) M[k, j] -= f * M[i, j];
                }
            }

            var x = new double[n];
            for (int i = 0; i < n; i++) x[i] = M[i, n];
            return x;
        }
        // Utilities for verbose
        private void PrintPose(string name, Math4.Mat4 T)
        {
            Console.WriteLine($"{name}:");
            for (int i = 0; i < 4; i++)
                Console.WriteLine($"{T.M[i, 0],11:F6} {T.M[i, 1],11:F6} {T.M[i, 2],11:F6} {T.M[i, 3],11:F6}");
        }
        private void PrintMatrix(string name, Math4.Mat4 T)
        {
            Console.WriteLine($"{name}:");
            for (int i = 0; i < 4; i++)
                Console.WriteLine($"{T.M[i, 0],10:F6} {T.M[i, 1],10:F6} {T.M[i, 2],10:F6} {T.M[i, 3],10:F6}");
        }

        private void PrintVec3(string name, Math4.Vec3 v)
        {
            Console.WriteLine($"{name}: [{v.X:F6}, {v.Y:F6}, {v.Z:F6}]");
        }
    }
}