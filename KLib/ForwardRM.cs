using System;
using ColumnMeasureUtility;

namespace ForwarRM;

public struct KinematicsResult
{
    public CMLib.Vec3 EndEffectorPos;
    public CMLib.Mat3 EndEffectorRot;
    public CMLib.Vec3[] Origins;
    public CMLib.Mat3[] Axes;
}

public static class Manipulator6DoF
{
    // Forward KinematicsCM: returns a result object
    public static KinematicsResult Forward(double[] q, double[] tool, int verbose = 0)
    {
        if (q.Length != 6) throw new ArgumentException("q must have 6 elements");
        if (tool.Length != 6) throw new ArgumentException("tool must have 6 elements");

        double[] alpha = { 0, Math.PI / 2, Math.PI, -Math.PI / 2, Math.PI / 2, Math.PI / 2 };
        double[] a = { 0, 29.69, 108, 20, 0, 0 };
        double[] d = { 127, 0, 0, 168.98, 0, 24.29 };
        double[] offset = { 0, Math.PI / 2, 0, 0, -Math.PI / 2, 0 };

        var origins = new CMLib.Vec3[7];
        var axes = new CMLib.Mat3[7];

        origins[0] = new CMLib.Vec3(0, 0, 0);
        axes[0] = new CMLib.Mat3(new CMLib.Vec3(1, 0, 0), new CMLib.Vec3(0, 1, 0), new CMLib.Vec3(0, 0, 1));

        for (var i = 0; i < 6; i++)
        {
            var theta = q[i] + offset[i];
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha[i]), sa = Math.Sin(alpha[i]);

            var Xprev = axes[i].Xaxis;
            var Yprev = axes[i].Yaxis;
            var Zprev = axes[i].Zaxis;

            var Xi = new CMLib.Vec3(
                ct * Xprev.X + st * (ca * Yprev.X + sa * Zprev.X),
                ct * Xprev.Y + st * (ca * Yprev.Y + sa * Zprev.Y),
                ct * Xprev.Z + st * (ca * Yprev.Z + sa * Zprev.Z)
            );

            var Yi = new CMLib.Vec3(
                -st * Xprev.X + ct * (ca * Yprev.X + sa * Zprev.X),
                -st * Xprev.Y + ct * (ca * Yprev.Y + sa * Zprev.Y),
                -st * Xprev.Z + ct * (ca * Yprev.Z + sa * Zprev.Z)
            );

            var Zi = new CMLib.Vec3(
                -sa * Yprev.X + ca * Zprev.X,
                -sa * Yprev.Y + ca * Zprev.Y,
                -sa * Yprev.Z + ca * Zprev.Z
            );

            axes[i + 1] = new CMLib.Mat3(Xi, Yi, Zi);
            origins[i + 1] = origins[i] + a[i] * Xprev + d[i] * Zi;
        }

        // Tool transformation
        double xt = tool[0], yt = tool[1], zt = tool[2];
        double rx = tool[3], ry = tool[4], rz = tool[5];

        double cx = Math.Cos(rx), sx = Math.Sin(rx);
        double cy = Math.Cos(ry), sy = Math.Sin(ry);
        double cz = Math.Cos(rz), sz = Math.Sin(rz);

        var Rtool = new CMLib.Mat3(
            new CMLib.Vec3(cz * cy, sz * cy, -sy),
            new CMLib.Vec3(cz * sy * sx - sz * cx, sz * sy * sx + cz * cx, cy * sx),
            new CMLib.Vec3(cz * sy * cx + sz * sx, sz * sy * cx - cz * sx, cy * cx)
        );

        var eePos = origins[6] + xt * axes[6].Xaxis + yt * axes[6].Yaxis + zt * axes[6].Zaxis;

        var Rprev = axes[6];
        var eeRot = new CMLib.Mat3(
            new CMLib.Vec3(
                Rtool.Xaxis.X * Rprev.Xaxis.X + Rtool.Yaxis.X * Rprev.Yaxis.X + Rtool.Zaxis.X * Rprev.Zaxis.X,
                Rtool.Xaxis.Y * Rprev.Xaxis.X + Rtool.Yaxis.Y * Rprev.Yaxis.X + Rtool.Zaxis.Y * Rprev.Zaxis.X,
                Rtool.Xaxis.Z * Rprev.Xaxis.X + Rtool.Yaxis.Z * Rprev.Yaxis.X + Rtool.Zaxis.Z * Rprev.Zaxis.X
            ),
            new CMLib.Vec3(
                Rtool.Xaxis.X * Rprev.Xaxis.Y + Rtool.Yaxis.X * Rprev.Yaxis.Y + Rtool.Zaxis.X * Rprev.Zaxis.Y,
                Rtool.Xaxis.Y * Rprev.Xaxis.Y + Rtool.Yaxis.Y * Rprev.Yaxis.Y + Rtool.Zaxis.Y * Rprev.Zaxis.Y,
                Rtool.Xaxis.Z * Rprev.Xaxis.Y + Rtool.Yaxis.Z * Rprev.Yaxis.Y + Rtool.Zaxis.Z * Rprev.Zaxis.Y
            ),
            new CMLib.Vec3(
                Rtool.Xaxis.X * Rprev.Xaxis.Z + Rtool.Yaxis.X * Rprev.Yaxis.Z + Rtool.Zaxis.X * Rprev.Zaxis.Z,
                Rtool.Xaxis.Y * Rprev.Xaxis.Z + Rtool.Yaxis.Y * Rprev.Yaxis.Z + Rtool.Zaxis.Y * Rprev.Zaxis.Z,
                Rtool.Xaxis.Z * Rprev.Xaxis.Z + Rtool.Yaxis.Z * Rprev.Yaxis.Z + Rtool.Zaxis.Z * Rprev.Zaxis.Z
            )
        );

        if (verbose >= 1)
        {
            Console.WriteLine("---- Origins (absolute coords) ----");
            for (var i = 0; i <= 6; i++)
                Console.WriteLine($"O{i}: {origins[i]}");
        }

        if (verbose >= 2)
        {
            Console.WriteLine("---- Local axes in absolute coordinates ----");
            for (var i = 1; i <= 6; i++)
                Console.WriteLine($"Frame {i}: X{axes[i].Xaxis} Y{axes[i].Yaxis} Z{axes[i].Zaxis}");
        }

        return new KinematicsResult
        {
            EndEffectorPos = eePos,
            EndEffectorRot = eeRot,
            Origins = origins,
            Axes = axes
        };
    }
}
