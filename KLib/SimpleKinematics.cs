using System;
using System.Collections.Generic;
using ColumnMeasureUtility;

namespace SimpleKinematics;

public struct KinematicsResult
{
    public ColumnMeasureUtility.CMLib.Vec3 FlangePos;
    public ColumnMeasureUtility.CMLib.Mat3 FlangeRot;

    public ColumnMeasureUtility.CMLib.Vec3 EndEffectorPos;
    public ColumnMeasureUtility.CMLib.Mat3 EndEffectorRot;

    public ColumnMeasureUtility.CMLib.Vec3 ToolDirectionLocal;
    public ColumnMeasureUtility.CMLib.Vec3 ToolDirectionWorld;

    public double FlangeRz;
    public double FlangeRy;
    public double FlangeRx;

    public double ToolRz;
    public double ToolRy;
    public double ToolRx;

    public ColumnMeasureUtility.CMLib.Vec3[] Origins;
    public ColumnMeasureUtility.CMLib.Mat3[] Axes;
}

/// <summary>
/// MDH parameter set (Modified DH). Joint limits stored in degrees for convenience.
/// </summary>
public sealed record MDHParameters(
    double[] Alpha,
    double[] A,
    double[] D,
    double[] Offset,
    double[] MinAnglesDeg,
    double[] MaxAnglesDeg);

/// <summary>
/// 6DoF manipulator kinematics.
/// Constructor now accepts (robotName, toolName, verbose).
/// - Robot MDH parameters are looked up in _mdhTable
/// - Tool definition is looked up in _toolTable
/// - Forward() only takes joint angles q
/// Tool format: [tx,ty,tz, zx,zy,zz] where (zx,zy,zz) is the tool Z-axis (unit) in flange frame.
/// </summary>
public class Manipulator6DoF
{
    public const string DefaultRobot = "MiRobot";
    public const string DefaultTool = "null";
    private const int JointCount = 6;

    // Internal registries (can be extended / replaced later by file/db loading).
    private static readonly object _sync = new();

    private static readonly Dictionary<string, MDHParameters> _mdhTable =
        new(StringComparer.OrdinalIgnoreCase)
        {
            [DefaultRobot] = new MDHParameters(
                Alpha:  [0, Math.PI / 2, Math.PI, -Math.PI / 2, Math.PI / 2, Math.PI / 2],
                A:      [0, 29.69, 108, 20, 0, 0],
                D:      [127, 0, 0, 168.98, 0, 24.29],
                Offset: [0, Math.PI / 2, 0, 0, -Math.PI / 2, 0],
                MinAnglesDeg: [-170, -125, -155, -180, -120, -180],
                MaxAnglesDeg: [ 170,  125,  155,  180,  120,  180 ]
            )
        };

    private static readonly Dictionary<string, double[]> _toolTable =
        new(StringComparer.OrdinalIgnoreCase)
        {
            // Default tool: origin, aligned with flange Z
            [DefaultTool] = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            // Example tool: offset + 45° about +Y
            ["tool0"]     = [1.41421356, 0.0, 2.41421356, 0.70710678, 0.0, 0.70710678]
        };

    // Instance data
    private readonly double[] _alpha;
    private readonly double[] _a;
    private readonly double[] _d;
    private readonly double[] _offset;
    private readonly double[] _minDeg;
    private readonly double[] _maxDeg;
    private double[] _tool; // [tx,ty,tz,zx,zy,zz]

    private readonly int _verbose;

    public string RobotName { get; }
    public string ToolName { get; private set; }

    public Manipulator6DoF(string robotName = DefaultRobot, string toolName = DefaultTool, int verbose = 0)
    {
        _verbose = verbose;
        RobotName = string.IsNullOrWhiteSpace(robotName) ? DefaultRobot : robotName;
        ToolName  = string.IsNullOrWhiteSpace(toolName)  ? DefaultTool  : toolName;

        if (!TryGetRobotInternal(RobotName, out var mdh))
            throw new ArgumentException($"Robot definition '{RobotName}' not found.", nameof(robotName));
        if (!TryGetToolInternal(ToolName, out var toolDef))
            throw new ArgumentException($"Tool definition '{ToolName}' not found.", nameof(toolName));

        _alpha  = (double[])mdh.Alpha.Clone();
        _a      = (double[])mdh.A.Clone();
        _d      = (double[])mdh.D.Clone();
        _offset = (double[])mdh.Offset.Clone();
        _minDeg = (double[])mdh.MinAnglesDeg.Clone();
        _maxDeg = (double[])mdh.MaxAnglesDeg.Clone();

        _tool = (double[])toolDef.Clone();

        ValidateArrayLengths();
    }

    // ----- Public registry management (future: replace with file/db loaders) -----

    public static void RegisterRobot(string name, MDHParameters mdh, bool overwrite = false)
    {
        if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("Robot name cannot be empty.", nameof(name));
        ValidateMdh(mdh);
        lock (_sync)
        {
            if (_mdhTable.ContainsKey(name) && !overwrite)
                throw new InvalidOperationException($"Robot '{name}' already exists (overwrite=false).");
            _mdhTable[name] = CloneMdh(mdh);
        }
    }

    public static void RegisterTool(string name, double[] tool, bool overwrite = false)
    {
        if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("Tool name cannot be empty.", nameof(name));
        ValidateTool(tool);
        lock (_sync)
        {
            if (_toolTable.ContainsKey(name) && !overwrite)
                throw new InvalidOperationException($"Tool '{name}' already exists (overwrite=false).");
            _toolTable[name] = (double[])tool.Clone();
        }
    }

    public void SetTool(string toolName)
    {
        if (!TryGetToolInternal(toolName, out var t))
            throw new ArgumentException($"Tool '{toolName}' not found.", nameof(toolName));
        _tool = (double[])t.Clone();
        ToolName = toolName;
    }

    public static string[] GetRobotNames()
    {
        lock (_sync)
        {
            var arr = new string[_mdhTable.Count];
            _mdhTable.Keys.CopyTo(arr, 0);
            return arr;
        }
    }

    public static string[] GetToolNames()
    {
        lock (_sync)
        {
            var arr = new string[_toolTable.Count];
            _toolTable.Keys.CopyTo(arr, 0);
            return arr;
        }
    }

    public (double[] MinDeg, double[] MaxDeg) GetJointLimits() =>
        ((double[])_minDeg.Clone(), (double[])_maxDeg.Clone());

    // ----- Forward Kinematics (uses constructor-selected tool) -----

    public KinematicsResult Forward(double[] q)
    {
        if (q is not { Length: JointCount }) throw new ArgumentException("q must have length 6", nameof(q));

        // (Optional) joint limit check (silent if within)
        for (var i = 0; i < JointCount; i++)
        {
            var deg = q[i] * 180.0 / Math.PI;
            if (deg < _minDeg[i] - 1e-9 || deg > _maxDeg[i] + 1e-9)
                throw new ArgumentOutOfRangeException(nameof(q), $"Joint {i+1} angle {deg:F3}° outside limits [{_minDeg[i]}, {_maxDeg[i]}]°");
        }

        // Unpack tool
        double tx = _tool[0], ty = _tool[1], tz = _tool[2];
        double zx = _tool[3], zy = _tool[4], zz = _tool[5];

        var zn = Math.Sqrt(zx * zx + zy * zy + zz * zz);
        if (zn < 1e-12) throw new InvalidOperationException("Tool Z-axis must be non-zero.");
        if (Math.Abs(zn - 1.0) > 1e-6)
        {
            zx /= zn; zy /= zn; zz /= zn;
        }

        var origins = new ColumnMeasureUtility.CMLib.Vec3[JointCount + 1];
        var axes    = new ColumnMeasureUtility.CMLib.Mat3[JointCount + 1];

        origins[0] = new ColumnMeasureUtility.CMLib.Vec3(0,0,0);
        axes[0] = new ColumnMeasureUtility.CMLib.Mat3(
            new ColumnMeasureUtility.CMLib.Vec3(1,0,0),
            new ColumnMeasureUtility.CMLib.Vec3(0,1,0),
            new ColumnMeasureUtility.CMLib.Vec3(0,0,1)
        );

        for (var i = 0; i < JointCount; i++)
        {
            var theta = q[i] + _offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(_alpha[i]), sa = Math.Sin(_alpha[i]);

            var Xp = axes[i].Xaxis;
            var Yp = axes[i].Yaxis;
            var Zp = axes[i].Zaxis;

            var Xi = new ColumnMeasureUtility.CMLib.Vec3(
                ct*Xp.X + st*(ca*Yp.X + sa*Zp.X),
                ct*Xp.Y + st*(ca*Yp.Y + sa*Zp.Y),
                ct*Xp.Z + st*(ca*Yp.Z + sa*Zp.Z)
            );
            var Yi = new ColumnMeasureUtility.CMLib.Vec3(
                -st*Xp.X + ct*(ca*Yp.X + sa*Zp.X),
                -st*Xp.Y + ct*(ca*Yp.Y + sa*Zp.Y),
                -st*Xp.Z + ct*(ca*Yp.Z + sa*Zp.Z)
            );
            var Zi = new ColumnMeasureUtility.CMLib.Vec3(
                -sa*Yp.X + ca*Zp.X,
                -sa*Yp.Y + ca*Zp.Y,
                -sa*Yp.Z + ca*Zp.Z
            );

            axes[i+1] = new ColumnMeasureUtility.CMLib.Mat3(Xi, Yi, Zi);
            origins[i+1] = origins[i] + _a[i]*Xp + _d[i]*Zi;
        }

        var flangePos = origins[JointCount];
        var flangeRot = axes[JointCount];

        // Tool Z axis (world)
        var Zt_world =
            zx * flangeRot.Xaxis +
            zy * flangeRot.Yaxis +
            zz * flangeRot.Zaxis;
        var znw = Math.Sqrt(Zt_world.X*Zt_world.X + Zt_world.Y*Zt_world.Y + Zt_world.Z*Zt_world.Z);
        Zt_world = new ColumnMeasureUtility.CMLib.Vec3(Zt_world.X/znw, Zt_world.Y/znw, Zt_world.Z/znw);

        // Build tool frame (roll = 0) using Up = Yf primarily
        var up = flangeRot.Yaxis;
        var Xt_world = ColumnMeasureUtility.CMLib.Cross(up, Zt_world);
        var xn = ColumnMeasureUtility.CMLib.Norm(Xt_world);
        if (xn < 1e-8)
        {
            up = flangeRot.Xaxis;
            Xt_world = ColumnMeasureUtility.CMLib.Cross(up, Zt_world);
            xn = ColumnMeasureUtility.CMLib.Norm(Xt_world);
            if (xn < 1e-8)
                throw new InvalidOperationException("Degenerate tool frame.");
        }
        Xt_world = new ColumnMeasureUtility.CMLib.Vec3(Xt_world.X/xn, Xt_world.Y/xn, Xt_world.Z/xn);
        var Yt_world = ColumnMeasureUtility.CMLib.Cross(Zt_world, Xt_world);

        var toolRot = new ColumnMeasureUtility.CMLib.Mat3(Xt_world, Yt_world, Zt_world);

        var toolPos = flangePos
                      + tx * flangeRot.Xaxis
                      + ty * flangeRot.Yaxis
                      + tz * flangeRot.Zaxis;

        // Decompose flange & relative rotation
        (var flangeRz, var flangeRy, var flangeRx) = ColumnMeasureUtility.CMLib.DecomposeZYX(flangeRot);
        var R_rel = ColumnMeasureUtility.CMLib.Multiply(ColumnMeasureUtility.CMLib.Transpose(flangeRot), toolRot);
        (var toolRz, var toolRy, var toolRx) = ColumnMeasureUtility.CMLib.DecomposeZYX(R_rel);

        if (_verbose >= 1)
        {
            Console.WriteLine($"[FK] Robot={RobotName} Tool={ToolName}");
            Console.WriteLine($"FlangePos: {flangePos}  ToolPos: {toolPos}");
        }
        if (_verbose >= 2)
        {
            Console.WriteLine($"Flange Axes: X{flangeRot.Xaxis} Y{flangeRot.Yaxis} Z{flangeRot.Zaxis}");
            Console.WriteLine($"Tool   Axes: X{Xt_world} Y{Yt_world} Z{Zt_world}");
            Console.WriteLine($"Tool Rel Angles (Z,Y,X): ({toolRz},{toolRy},{toolRx})");
        }

        return new KinematicsResult
        {
            FlangePos = flangePos,
            FlangeRot = flangeRot,
            EndEffectorPos = toolPos,
            EndEffectorRot = toolRot,
            ToolDirectionLocal = new ColumnMeasureUtility.CMLib.Vec3(zx, zy, zz),
            ToolDirectionWorld = Zt_world,
            FlangeRz = flangeRz,
            FlangeRy = flangeRy,
            FlangeRx = flangeRx,
            ToolRz = toolRz,
            ToolRy = toolRy,
            ToolRx = toolRx,
            Origins = origins,
            Axes = axes
        };
    }

    // ----- Utility / math -----

    private static bool TryGetRobotInternal(string name, out MDHParameters mdh)
    {
        lock (_sync)
        {
            if (_mdhTable.TryGetValue(name, out var p))
            {
                mdh = CloneMdh(p);
                return true;
            }
        }
        mdh = default!;
        return false;
    }

    private static bool TryGetToolInternal(string name, out double[] tool)
    {
        lock (_sync)
        {
            if (_toolTable.TryGetValue(name, out var t))
            {
                tool = (double[])t.Clone();
                return true;
            }
        }
        tool = Array.Empty<double>();
        return false;
    }

    private static void ValidateMdh(MDHParameters mdh)
    {
        if (mdh.Alpha.Length != JointCount ||
            mdh.A.Length != JointCount ||
            mdh.D.Length != JointCount ||
            mdh.Offset.Length != JointCount ||
            mdh.MinAnglesDeg.Length != JointCount ||
            mdh.MaxAnglesDeg.Length != JointCount)
            throw new ArgumentException("Each MDH array must have length 6.");
    }

    private static void ValidateTool(double[] tool)
    {
        if (tool.Length != 6)
            throw new ArgumentException("Tool must have length 6: [tx,ty,tz,zx,zy,zz]");
    }

    private void ValidateArrayLengths()
    {
        if (_alpha.Length != JointCount ||
            _a.Length != JointCount ||
            _d.Length != JointCount ||
            _offset.Length != JointCount)
            throw new InvalidOperationException("Internal MDH arrays invalid length.");
    }

    private static MDHParameters CloneMdh(MDHParameters p) =>
        new(
            (double[])p.Alpha.Clone(),
            (double[])p.A.Clone(),
            (double[])p.D.Clone(),
            (double[])p.Offset.Clone(),
            (double[])p.MinAnglesDeg.Clone(),
            (double[])p.MaxAnglesDeg.Clone()
        );
}
