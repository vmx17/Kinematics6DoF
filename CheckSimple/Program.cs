using System;
using SimpleKinematics;

internal static class Program
{
    private static double Rad2Deg(double r) => r * 180.0 / Math.PI;

    private static void Main()
    {
        Console.WriteLine("--- ManipulatorKinematics Forward Kinematics Test (Registry Based) ---\n");

        var robot = new Manipulator6DoF(robotName: Manipulator6DoF.DefaultRobot, toolName: Manipulator6DoF.DefaultTool, verbose: 0);

        // サンプル関節角（全ゼロ）
        var q = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Console.WriteLine("Joint Angles (rad):");
        PrintArray(q);
        Console.WriteLine();

        // 使用可能ツール名一覧取得（登録済み）
        var toolNames = Manipulator6DoF.GetToolNames();
        Array.Sort(toolNames, StringComparer.OrdinalIgnoreCase);

        foreach (var toolName in toolNames)
        {
            robot.SetTool(toolName);
            Console.WriteLine($"== Tool: {toolName} ==");
            var fk = robot.Forward(q);
            PrintKinematics(fk, showOrigins: true);
            Console.WriteLine();
        }

        Console.WriteLine("Expectation check:");
        Console.WriteLine(" tool0 Z-axis (0.7071,0,0.7071) => rotation about +Y ≈ +π/4 (45°), so ToolRy ≈ +0.785398 rad.\n");

        Console.WriteLine("Done.");
    }

    private static void PrintKinematics(KinematicsResult r, bool showOrigins)
    {
        Console.WriteLine("-- Flange Pose --");
        Console.WriteLine($"Flange Position : {Fmt(r.FlangePos)}");
        PrintRot("Flange Rotation", r.FlangeRot);
        Console.WriteLine($"Flange (Z,Y,X) rad: ({FmtRad(r.FlangeRz)},{FmtRad(r.FlangeRy)},{FmtRad(r.FlangeRx)})");
        Console.WriteLine($"Flange (Z,Y,X) deg: ({FmtDeg(r.FlangeRz)},{FmtDeg(r.FlangeRy)},{FmtDeg(r.FlangeRx)})");

        Console.WriteLine("-- Tool Tip Pose --");
        Console.WriteLine($"Tool Tip Position : {Fmt(r.EndEffectorPos)}");
        PrintRot("Tool Rotation", r.EndEffectorRot);
        Console.WriteLine($"Tool Rel (Z,Y,X) rad: ({FmtRad(r.ToolRz)},{FmtRad(r.ToolRy)},{FmtRad(r.ToolRx)})");
        Console.WriteLine($"Tool Rel (Z,Y,X) deg: ({FmtDeg(r.ToolRz)},{FmtDeg(r.ToolRy)},{FmtDeg(r.ToolRx)})");

        Console.WriteLine("-- Tool Direction (Z-axis) --");
        Console.WriteLine($"Local: {Fmt(r.ToolDirectionLocal)}  World: {Fmt(r.ToolDirectionWorld)}");

        if (showOrigins)
        {
            Console.WriteLine("-- Joint Origins (O0..O6) --");
            for (var i = 0; i < r.Origins.Length; i++)
                Console.WriteLine($"O{i}: {Fmt(r.Origins[i])}");
        }
    }

    private static void PrintRot(string label, ColumnMeasureUtility.CMLib.Mat3 R)
    {
        Console.WriteLine($"{label} (columns = frame axes):");
        Console.WriteLine($"  X: {Fmt(R.Xaxis)}");
        Console.WriteLine($"  Y: {Fmt(R.Yaxis)}");
        Console.WriteLine($"  Z: {Fmt(R.Zaxis)}");
    }

    private static void PrintArray(double[] arr)
    {
        Console.Write("[ ");
        for (var i = 0; i < arr.Length; i++)
        {
            if (i > 0) Console.Write(", ");
            Console.Write($"{arr[i]:+0.000000;-0.000000;0.000000}");
        }
        Console.WriteLine(" ]");
    }

    private static string Fmt(ColumnMeasureUtility.CMLib.Vec3 v) =>
        $"({v.X:+0.000000;-0.000000;0.000000}, {v.Y:+0.000000;-0.000000;0.000000}, {v.Z:+0.000000;-0.000000;0.000000})";

    private static string FmtRad(double v) => v.ToString("+0.000000;-0.000000;0.000000");
    private static string FmtDeg(double v) => Rad2Deg(v).ToString("+0.000;-0.000;0.000");
}
