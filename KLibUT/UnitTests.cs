using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Kinematics;

namespace KLibUT;

// 既存の UnitTest1 はそのまま
[TestClass]
public partial class UnitTest1
{
    [TestMethod]
    public void TestMethod1()
    {
        Assert.AreEqual(0, 0);
    }

    // UIテスト不要ならこのメソッドごと削除してもOK
    // もし残す場合は [TestMethod] に変更
    [TestMethod]
    public void TestMethod2()
    {
        // var grid = new Microsoft.UI.Xaml.Controls.Grid();
        // Assert.AreEqual(0, grid.MinWidth);
        Assert.AreEqual(1, 1); // ダミー
    }
}

// ManipulatorKinematicsTests をトップレベルの TestClass として分離
[TestClass]
public class ManipulatorKinematicsTests
{
    private static readonly double[] tool_test = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    private static readonly double[] q_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    private static readonly double[] q_test = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

    private static readonly Manipulator6DoF robot = ManipulatorFactory.CreateDefaultManipulator();

    private static bool CompareMatrices(double[,] matrixA, double[,] matrixB, double tolerance = 1e-5)
    {
        if (matrixA.GetLength(0) != 4 || matrixA.GetLength(1) != 4 ||
            matrixB.GetLength(0) != 4 || matrixB.GetLength(1) != 4)
        {
            return false;
        }
        for (var i = 0; i < 4; i++)
        {
            for (var j = 0; j < 4; j++)
            {
                if (Math.Abs(matrixA[i, j] - matrixB[i, j]) > tolerance)
                {
                    return false;
                }
            }
        }
        return true;
    }

    [TestMethod]
    public void ForwardKinematics_AllZeros_ShouldMatchExpectedPose()
    {
        var T_zeros = robot.Forward(q_zeros, tool_test, 0);
        Assert.IsNotNull(T_zeros);
        Assert.AreEqual(4, T_zeros.GetLength(0));
        Assert.AreEqual(4, T_zeros.GetLength(1));
    }

    [TestMethod]
    public void JacobianIK_AllZeros_ShouldConvergeAndMatchForward()
    {
        var T_zeros = robot.Forward(q_zeros, tool_test, 0);
        bool success;
        var q_result = robot.InverseJacobian(T_zeros, q_zeros, out success);
        Assert.IsTrue(success, "Jacobian IK did not converge for zero pose.");
        var T_recheck = robot.Forward(q_result, tool_test, 0);
        Assert.IsTrue(CompareMatrices(T_zeros, T_recheck), "Jacobian IK result does not match target pose.");
    }

    [TestMethod]
    public void ForwardKinematics_GeneralAngles_ShouldMatchExpectedPose()
    {
        var T_general = robot.Forward(q_test, tool_test, 0);
        Assert.IsNotNull(T_general);
        Assert.AreEqual(4, T_general.GetLength(0));
        Assert.AreEqual(4, T_general.GetLength(1));
    }

    [TestMethod]
    public void JacobianIK_GeneralAngles_ShouldConvergeAndMatchForward()
    {
        var T_general = robot.Forward(q_test, tool_test, 0);
        bool success;
        var q_result = robot.InverseJacobian(T_general, q_test, out success);
        Assert.IsTrue(success, "Jacobian IK did not converge for general pose.");
        var T_recheck = robot.Forward(q_result, tool_test, 0);
        Assert.IsTrue(CompareMatrices(T_general, T_recheck), "Jacobian IK result does not match target pose.");
    }
}
