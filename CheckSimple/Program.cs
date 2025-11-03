using System;
using System.Globalization;
using System.Text;
using KinematicsRM2; // 指定された名前空間を使用

/// <summary>
/// KinematicsRM3.ManipulatorKinematics モジュールのテスト用コンソールアプリ。
/// </summary>
public class Program
{
    public static void Main(string[] args)
    {
        // カルチャ設定 (小数点記号を '.' に固定)
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        var kin = new Manipulator6DoF();

        Console.WriteLine("=============================================");
        Console.WriteLine("    KinematicsRM3 モジュール テスト");
        Console.WriteLine("=============================================");

        // --- 基本姿勢 (全関節 0) の定義 ---
        // 仕様書 [cite: 76] に基づく
        double[] q_basic = { 0, 0, 0, 0, 0, 0 };

        // --- 1. 順運動学 (FK) テスト ---
        TestForwardKinematics(kin, q_basic);

        Console.WriteLine("\n=============================================");

        // --- 2. 逆運動学 (IK) テスト ---
        TestInverseKinematics(kin, q_basic);

        Console.WriteLine("=============================================");
    }

    /// <summary>
    /// 順運動学 (FK) のテスト
    /// </summary>
    public static void TestForwardKinematics(ManipulatorKinematics kin, double[] q_basic)
    {
        Console.WriteLine("--- 1. 順運動学 (FK) テスト ---");
        Console.WriteLine("[1.1] 姿勢: 基本姿勢 (All Zero)");

        // verbose=1 で中間フレームの原点も出力 [cite: 117]
        double[,] T_result = kin.Forward(q_basic, null, 1);

        Console.WriteLine("\n  角度 (入力):");
        Console.WriteLine($"  {TestHelper.FormatAngles(q_basic)}");
        Console.WriteLine("\n  結果 (TCP):");
        Console.WriteLine($"  {TestHelper.FormatPose(T_result)}");

        // 仕様書 [cite: 86] の O6 (Flange) の値と比較
        // O6 (J6:Flange) (198.670, 0, 230.710)
        double[] expected_O6 = { 198.670, 0, 230.710 };
        double[] actual_O6 = KinematicsCM3.MatrixUtils.GetTranslation(T_result);

        if (TestHelper.ValidatePosition(actual_O6, expected_O6))
        {
            Console.WriteLine("\n  [検証: OK] FK結果は仕様書の O6 の値と一致します。");
        }
        else
        {
            Console.WriteLine($"\n  [検証: NG] FK結果が仕様書 (O6) と異なります。");
            Console.WriteLine($"    期待値: ({expected_O6[0]:F3}, {expected_O6[1]:F3}, {expected_O6[2]:F3})");
        }
    }

    /// <summary>
    /// 逆運動学 (IK) のテスト
    /// </summary>
    public static void TestInverseKinematics(ManipulatorKinematics kin, double[] q_basic)
    {
        Console.WriteLine("--- 2. 逆運動学 (IK) テスト ---");

        // FKの結果をIKのターゲットとして使用
        double[,] T_target = kin.Forward(q_basic);

        Console.WriteLine($"[2.1] ターゲット姿勢 (FKからの逆算):");
        Console.WriteLine($"  {TestHelper.FormatPose(T_target)}");
        Console.WriteLine($"  ターゲット角度 (期待値):");
        Console.WriteLine($"  {TestHelper.FormatAngles(q_basic)}");

        // --- IK1: 幾何学的アプローチ ---
        Console.WriteLine("\n[2.2] 幾何学的IK (InverseGeometric) の試行...");
        // verbose=1 で Pw を出力 [cite: 119]
        double[] q_ik_geo = kin.InverseGeometric(T_target, 1);

        if (q_ik_geo == null)
        {
            Console.WriteLine("  IK (Geo) 結果: 未実装です。");
        }

        // --- IK2: ヤコビアンアプローチ ---
        Console.WriteLine("\n[2.3] ヤコビアンIK (InverseJacobian) の試行...");
        // 初期値としてターゲット角度 (q_basic) をそのまま使用
        bool success;
        double[] q_ik_jac = kin.InverseJacobian(T_target, q_basic, out success);

        if (!success)
        {
            Console.WriteLine("  IK (Jac) 結果: ソルバーが未実装、または収束しませんでした。");
        }
        else
        {
            Console.WriteLine($"  IK (Jac) 収束角度:");
            Console.WriteLine($"  {TestHelper.FormatAngles(q_ik_jac)}");
        }
    }
}

/// <summary>
/// テスト用の書式設定および検証ヘルパー
/// </summary>
internal static class TestHelper
{
    // rad -> deg 変換
    public static double ToDegrees(double rad)
    {
        return rad * 180.0 / Math.PI;
    }

    /// <summary>
    /// 位置と姿勢を指定の書式で文字列化します。
    /// 位置: 小数点以下3桁 [cite: 21], 角度: 小数点以下4桁 [cite: 21]
                /// </summary>
    public static string FormatPose(double[,] T_matrix)
    {
        double x = T_matrix[0, 3];
        double y = T_matrix[1, 3];
        double z = T_matrix[2, 3];

        // オイラー角 (Z-Y-X順: rtz, rty, rtx) を抽出
        double rty_rad = Math.Asin(-T_matrix[2, 0]);
        double rtx_rad, rtz_rad;

        if (Math.Abs(Math.Cos(rty_rad)) > 1e-6) // 特異点でない場合
        {
            rtx_rad = Math.Atan2(T_matrix[2, 1], T_matrix[2, 2]);
            rtz_rad = Math.Atan2(T_matrix[1, 0], T_matrix[0, 0]);
        }
        else // 特異点 (Gimbal lock)
        {
            rtz_rad = 0;
            rtx_rad = Math.Atan2(-T_matrix[1, 2], T_matrix[1, 1]);
        }

        return $"P:({x:F3}, {y:F3}, {z:F3}) mm, R:({ToDegrees(rtx_rad):F4}, {ToDegrees(rty_rad):F4}, {ToDegrees(rtz_rad):F4}) deg";
    }

    /// <summary>
    /// 関節角度配列を指定の書式で文字列化します。
    /// </summary>
    public static string FormatAngles(double[] jointAngles)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("[");
        for (int i = 0; i < jointAngles.Length; i++)
        {
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F4}", ToDegrees(jointAngles[i]));
            if (i < jointAngles.Length - 1) sb.Append(", ");
        }
        sb.Append("] deg");
        return sb.ToString();
    }

    /// <summary>
    /// 2つの位置ベクトルがほぼ等しいか検証します (0.001mm 以内)
    /// </summary>
    public static bool ValidatePosition(double[] pos1, double[] pos2, double tolerance = 0.001)
    {
        return Math.Abs(pos1[0] - pos2[0]) < tolerance &&
               Math.Abs(pos1[1] - pos2[1]) < tolerance &&
               Math.Abs(pos1[2] - pos2[2]) < tolerance;
    }
}
