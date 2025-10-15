using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinematicsRM;
using RowMeasureUtility;

namespace KLibUT
{
    [TestClass]
    public class ManipulatorKinematicsTests
    {
        private static Manipulator6DoF CreateDefaultRobot() =>
            ManipulatorFactory.CreateManipulator("MiRobot"); // uses default "null" tool

        private static double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }

        private static double JointVectorMaxError(double[] a, double[] b) =>
            a.Zip(b, (x, y) => Math.Abs(NormalizeAngle(x - y))).Max();

        private static double PosePositionError(double[,] A, double[,] B)
        {
            double s = 0;
            for (var i = 0; i < 3; i++)
            {
                var d = A[i, 3] - B[i, 3];
                s += d * d;
            }
            return Math.Sqrt(s);
        }

        private static double PoseOrientationFroError(double[,] A, double[,] B)
        {
            double s = 0;
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                {
                    var d = A[r, c] - B[r, c];
                    s += d * d;
                }
            return Math.Sqrt(s);
        }

        private static bool MatrixIsOrthonormal(double[,] R, double tolerance = 1e-6)
        {
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                {
                    double dot = 0;
                    for (var k = 0; k < 3; k++) dot += R[k, r] * R[k, c];
                    if (r == c)
                    {
                        if (Math.Abs(dot - 1.0) > tolerance) return false;
                    }
                    else if (Math.Abs(dot) > tolerance) return false;
                }
            var det =
                R[0,0]*(R[1,1]*R[2,2]-R[1,2]*R[2,1]) -
                R[0,1]*(R[1,0]*R[2,2]-R[1,2]*R[2,0]) +
                R[0,2]*(R[1,0]*R[2,1]-R[1,1]*R[2,0]);
            return Math.Abs(det - 1.0) < 1e-4;
        }

        [TestMethod]
        public void Forward_ForZeroAngles_ReturnsOrthonormalRotation()
        {
            var robot = CreateDefaultRobot();
            var q = new double[6];
            var T = robot.Forward(q);
            var R = new double[3,3];
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    R[r,c] = T[r,c];
            Assert.IsTrue(MatrixIsOrthonormal(R), "Rotation not orthonormal at zero joints.");
        }

        [TestMethod]
        public void InverseJacobian_RoundTripZeroPose_Succeeds()
        {
            var robot = CreateDefaultRobot();
            var q_target = new double[6];
            var T_target = robot.Forward(q_target);
            var q_guess = new double[6];
            var q_sol = robot.InverseJacobian(T_target, q_guess, out var success,
                                              maxIterations : 500, tolerance : 1e-8, alpha:0.5, damping:1e-3);
            Assert.IsTrue(success && q_sol != null, "InverseJacobian failed on zero pose.");
            Assert.IsLessThan(1e-4, JointVectorMaxError(q_target, q_sol!), "Joint error too large.");
        }

        [TestMethod]
        public void InverseJacobian_RoundTripRandomSamples()
        {
            var robot = CreateDefaultRobot();
            var rnd = new Random(1234);
            const int samples = 5;
            const double jointScale = 0.6; // conservative
            for (var s = 0; s < samples; s++)
            {
                var q = Enumerable.Range(0,6)
                    .Select(_ => (rnd.NextDouble()*2 - 1) * Math.PI * jointScale)
                    .ToArray();
                var T = robot.Forward(q);
                var q_guess = new double[6];
                var q_sol = robot.InverseJacobian(T, q_guess, out var success,
                                                  maxIterations:800, tolerance:1e-6, alpha:0.5, damping:1e-3);
                Assert.IsTrue(success && q_sol != null, $"Sample {s}: numeric IK failed.");
                Assert.IsLessThan(5e-3, JointVectorMaxError(q, q_sol!), $"Sample {s}: joint error too large.");
            }
        }

        [TestMethod]
        public void InverseGeometric_ContainsZeroConfiguration()
        {
            var robot = CreateDefaultRobot();
            var q_target = new double[6];
            var T_target = robot.Forward(q_target);
            var solutions = robot.InverseGeometric(T_target, verbose:0);
            Assert.IsNotEmpty(solutions, "No geometric solutions.");
            var found = solutions.Any(sol => JointVectorMaxError(sol, q_target) < 1e-4);
            Assert.IsTrue(found, "Zero configuration absent.");
        }

        [TestMethod]
        public void NumericSolution_AppearsInGeometricSolutions()
        {
            var robot = CreateDefaultRobot();
            var q_target = new[] { 0.1, -0.2, 0.05, 0.3, -0.15, 0.2 };
            var T_target = robot.Forward(q_target);
            var q_guess = new double[6];
            var q_num = robot.InverseJacobian(T_target, q_guess, out var success,
                                              maxIterations:700, tolerance:1e-6, alpha:0.5, damping:1e-3);
            Assert.IsTrue(success && q_num != null, "Numeric IK failed.");
            var solutions = robot.InverseGeometric(T_target, verbose:0);
            var present = solutions.Any(sol => JointVectorMaxError(sol, q_num!) < 5e-3);
            Assert.IsTrue(present, "Numeric solution not in geometric set.");
        }

        [TestMethod]
        public void RoundTripPoseErrorsWithinTolerance()
        {
            var robot = CreateDefaultRobot();
            var q = new[] { 0.2, -0.1, 0.05, -0.3, 0.4, -0.25 };
            var T = robot.Forward(q);
            var q_guess = new double[6];
            var q_sol = robot.InverseJacobian(T, q_guess, out var success,
                                              maxIterations:900, tolerance:1e-7, alpha:0.4, damping:1e-3);
            Assert.IsTrue(success && q_sol != null, "InverseJacobian failed.");
            var T_chk = robot.Forward(q_sol!);
            var posErr = PosePositionError(T, T_chk);
            var oriErr = PoseOrientationFroError(T, T_chk);
            Assert.IsLessThan(1e-3, posErr, $"Position error {posErr}");
            Assert.IsLessThan(1e-3, oriErr, $"Orientation error {oriErr}");
        }

        [TestMethod]
        public void ToolOffsetChangesTCPButNotFlange()
        {
            var robot = CreateDefaultRobot();
            var q = new double[6];
            var flangePose = ExtractFlange(robot, q);
            var tcpDefault = robot.Forward(q);

            var toolName = "testTool";
            ManipulatorFactory.RegisterTool(toolName,
                positionLocal: new[] { 10.0, 0.0, 5.0 },
                rxyz: new[] { 0.0, 0.0, 0.0 },
                overwrite: true);
            var robot2 = ManipulatorFactory.CreateManipulator("MiRobot", toolName);
            var tcpWithTool = robot2.Forward(q);
            var flangePose2 = ExtractFlange(robot2, q);

            var flangeShift = PosePositionError(flangePose, flangePose2);
            Assert.IsLessThan(1e-9, flangeShift, $"Flange moved: {flangeShift}");
            Assert.AreNotEqual(tcpDefault[0,3], tcpWithTool[0,3], "TCP X unchanged.");
            Assert.AreNotEqual(tcpDefault[2,3], tcpWithTool[2,3], "TCP Z unchanged.");
        }

        // Extract flange by removing tool transform from TCP.
        private static double[,] ExtractFlange(Manipulator6DoF robot, double[] q)
        {
            var T_tcp = robot.Forward(q);
            var tool = robot.Tool;
            // Inverse of tool transform (R^T and -R^T * p)
            var R = tool.Rotation;
            var p = tool.PositionLocal;
            var RT = new double[3,3];
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                    RT[r,c] = R[c,r];
            var Rp = new[]
            {
                RT[0,0]*p[0] + RT[0,1]*p[1] + RT[0,2]*p[2],
                RT[1,0]*p[0] + RT[1,1]*p[1] + RT[1,2]*p[2],
                RT[2,0]*p[0] + RT[2,1]*p[1] + RT[2,2]*p[2]
            };
            var T_flange = MathUtil.Identity();
            // R_flange = R_tcp * R_tool^T
            for (var r = 0; r < 3; r++)
                for (var c = 0; c < 3; c++)
                {
                    double v = 0;
                    for (var k = 0; k < 3; k++) v += T_tcp[r,k] * RT[k,c];
                    T_flange[r,c] = v;
                }
            // P_flange = P_tcp - R_flange * p_tool  (same as T_tcp * tool_inv)
            var p_flange = new[]
            {
                T_tcp[0,3] - (T_flange[0,0]*p[0] + T_flange[0,1]*p[1] + T_flange[0,2]*p[2]),
                T_tcp[1,3] - (T_flange[1,0]*p[0] + T_flange[1,1]*p[1] + T_flange[1,2]*p[2]),
                T_tcp[2,3] - (T_flange[2,0]*p[0] + T_flange[2,1]*p[1] + T_flange[2,2]*p[2])
            };
            T_flange[0,3] = p_flange[0];
            T_flange[1,3] = p_flange[1];
            T_flange[2,3] = p_flange[2];
            return T_flange;
        }
    }
}
