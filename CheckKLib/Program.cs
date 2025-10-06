using System;
using System.Numerics; // (still available if later you add quaternions)
using Kinematics2;

internal class Program
{
    public static void Main()
    {
        Console.WriteLine("--- ManipulatorKinematics Test Program ---");

        var robot = ManipulatorFactory.CreateManipulator("MiRobot", "tool0");

        // 0. All-zero joint angles FK
        Console.WriteLine("\n--- Forward Kinematics Test (All Zeros) ---");
        double[] q_zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Console.WriteLine($"Test Joint Angles (q_zeros deg): [{FormatJointAngles(q_zeros)}]");

        var T_zeros = robot.Forward(q_zeros, 2);
        Console.WriteLine("\nEnd-Effector Pose (T_zeros):");
        PrintMatrix(T_zeros);

        // Orientation quick summary (axis-angle)
        var (axis0, angle0) = GetAxisAngle(T_zeros);
        Console.WriteLine($"Orientation axis-angle (from identity): axis=({axis0.X:F4},{axis0.Y:F4},{axis0.Z:F4}) angle={angle0 * 180 / Math.PI:F4} deg");

        Console.WriteLine("\n--- Inverse Kinematics Test (Target: All Zeros Pose) ---");
        var T_target_zeros = T_zeros;

        // Geometric IK
        Console.WriteLine("\n--- Geometric IK Test (Target: T_zeros) ---");
        bool[] ik_flags_zeros;
        var ik_solutions_zeros = robot.InverseGeometric(T_target_zeros, out ik_flags_zeros, verbose: 1);

        Console.WriteLine("Geometric IK Solutions for T_zeros:");
        for (var i = 0; i < ik_solutions_zeros.Length; i++)
        {
            if (!ik_flags_zeros[i]) continue;
            Console.WriteLine($"  Solution {i + 1} (Valid): [{FormatJointAngles(ik_solutions_zeros[i])}]");
            var T_recheck = robot.Forward(ik_solutions_zeros[i], 0);
            if (CompareMatrices(T_target_zeros, T_recheck))
                Console.WriteLine("    Verification: SUCCESS");
            else
            {
                Console.WriteLine("    Verification: FAILED");
                PrintMatrix(T_recheck);
            }
        }

        // Jacobian IK
        Console.WriteLine("\n--- Jacobian IK Test (Target: T_zeros) ---");
        double[] q_initial_guess_zeros = { 0, 0, 0, 0, 0, 0 };
        bool jacobian_success_zeros;
        var q_jacobian_result_zeros = robot.InverseJacobian(T_target_zeros, q_initial_guess_zeros,
            out jacobian_success_zeros, verbose: 1);

        if (jacobian_success_zeros)
        {
            Console.WriteLine($"Jacobian IK converged. Result: [{FormatJointAngles(q_jacobian_result_zeros)}]");
            var T_jacobian = robot.Forward(q_jacobian_result_zeros, 0);
            Console.WriteLine("  Re-calculated Pose:"); PrintMatrix(T_jacobian);
            Console.WriteLine(CompareMatrices(T_target_zeros, T_jacobian)
                ? "    Verification: SUCCESS"
                : "    Verification: FAILED");
        }
        else
            Console.WriteLine("Jacobian IK did NOT converge.");

        // General angles FK
        Console.WriteLine("\n--- Forward Kinematics Test (General Angles) ---");
        double[] q_test = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
        Console.WriteLine($"Test Joint Angles (deg): [{FormatJointAngles(q_test)}]");

        var T_target_general = robot.Forward(q_test, 0);
        Console.WriteLine("\nEnd-Effector Pose (T_target_general):");
        PrintMatrix(T_target_general);
        var (axisG, angleG) = GetAxisAngle(T_target_general);
        Console.WriteLine($"Orientation axis-angle (from identity): axis=({axisG.X:F4},{axisG.Y:F4},{axisG.Z:F4}) angle={angleG * 180 / Math.PI:F4} deg");

        // Geometric IK for general pose
        Console.WriteLine("\n--- Geometric IK Test (Target: T_general) ---");
        bool[] ik_flags_general;
        var ik_solutions_general = robot.InverseGeometric(T_target_general, out ik_flags_general, verbose: 1);

        Console.WriteLine("\nGeometric IK Solutions for T_general:");
        for (var i = 0; i < ik_solutions_general.Length; i++)
        {
            if (!ik_flags_general[i]) continue;
            Console.WriteLine($"  Solution {i + 1} (Valid): [{FormatJointAngles(ik_solutions_general[i])}]");
            var T_recheck_general = robot.Forward(ik_solutions_general[i], 0);
            if (CompareMatrices(T_target_general, T_recheck_general))
                Console.WriteLine("    Verification: SUCCESS");
            else
            {
                Console.WriteLine("    Verification: FAILED");
                PrintMatrix(T_recheck_general);
            }
        }

        // Jacobian IK for general pose
        Console.WriteLine("\n--- Jacobian IK Test (Target: T_general) ---");
        var q_initial_guess_general = (double[])q_test.Clone();
        bool jacobian_success_general;
        var q_jacobian_result_general = robot.InverseJacobian(T_target_general, q_initial_guess_general,
            out jacobian_success_general, verbose: 1);

        if (jacobian_success_general)
        {
            Console.WriteLine($"Jacobian IK converged. Result: [{FormatJointAngles(q_jacobian_result_general)}]");
            var T_jacobian_recheck_general = robot.Forward(q_jacobian_result_general, 0);
            Console.WriteLine("\n  Re-calculated Pose:");
            PrintMatrix(T_jacobian_recheck_general);
            Console.WriteLine(CompareMatrices(T_target_general, T_jacobian_recheck_general)
                ? "    Verification: SUCCESS"
                : "    Verification: FAILED");
        }
        else
            Console.WriteLine("Jacobian IK did NOT converge.");

        Console.WriteLine("\n--- Test Program End ---");
    }

    public static void PrintMatrix(double[,] matrix)
    {
        for (var i = 0; i < 4; i++)
        {
            Console.Write("[");
            for (var j = 0; j < 4; j++)
            {
                Console.Write(j < 3
                    ? $"{matrix[i, j]:F6}"
                    : $"{matrix[i, j]:F3}");
                if (j < 3) Console.Write(", ");
            }
            Console.WriteLine("]");
        }
    }

    public static bool CompareMatrices(double[,] A, double[,] B, double tolerance = 1e-5)
    {
        if (A.GetLength(0) != 4 || A.GetLength(1) != 4 || B.GetLength(0) != 4 || B.GetLength(1) != 4)
            return false;
        for (var i = 0; i < 4; i++)
            for (var j = 0; j < 4; j++)
                if (Math.Abs(A[i, j] - B[i, j]) > tolerance) return false;
        return true;
    }

    public static string FormatJointAngles(double[] anglesRad)
    {
        var s = new string[anglesRad.Length];
        for (var i = 0; i < anglesRad.Length; i++)
            s[i] = (anglesRad[i] * 180 / Math.PI).ToString("F4");
        return string.Join(", ", s);
    }

    // Axis-angle relative to identity (i.e., rotation part only)
    private static (System.Numerics.Vector3 axis, double angle) GetAxisAngle(double[,] T)
    {
        // Extract rotation
        double r00 = T[0, 0], r01 = T[0, 1], r02 = T[0, 2];
        double r10 = T[1, 0], r11 = T[1, 1], r12 = T[1, 2];
        double r20 = T[2, 0], r21 = T[2, 1], r22 = T[2, 2];

        var trace = r00 + r11 + r22;
        var cosAng = Math.Clamp((trace - 1.0) * 0.5, -1.0, 1.0);
        var angle = Math.Acos(cosAng);

        if (angle < 1e-9)
            return (new System.Numerics.Vector3(1, 0, 0), 0.0);

        var denom = 2 * Math.Sin(angle);
        var ax = (r21 - r12) / denom;
        var ay = (r02 - r20) / denom;
        var az = (r10 - r01) / denom;
        return (new System.Numerics.Vector3((float)ax, (float)ay, (float)az), angle);
    }
}
