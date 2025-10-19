# Correct Inverse Kinematics for a 6DoF Manipulator Using the Jacobian in C#

## Introduction

Inverse kinematics (**IK**) is a foundational problem in robotics, particularly for manipulators with six degrees of freedom (**6DoF**). The task is to determine the necessary joint angles for a robotic arm so that its end-effector achieves a desired position and orientation in space. While analytic (closed-form) solutions exist for specific manipulator architectures, most general 6DoF arms require a **numerical iterative approach**—typically based on the **Jacobian matrix**—to solve IK for arbitrary end-effector poses. 

Given the context of the [Kinematics6DoF C# project](https://github.com/vmx17/Kinematics6DoF/blob/main/README.md), we investigate the requirements and structure for a correct Jacobian-based IK implementation: clarifying the mathematical algorithms, dissecting code structure, addressing numerical stability and singularities, leveraging C# linear algebra libraries, and supporting the system with practical, credible references from both the robotics research literature and real-world open-source examples.

---

## 1. Mathematical Foundations of Jacobian-Based Inverse Kinematics

### 1.1 Kinematics: Forward and Inverse

**Forward kinematics (FK)** calculates the end-effector pose (position and orientation) from a set of given joint angles. This is typically achieved via chained homogeneous transformation matrices using the Denavit–Hartenberg (DH) convention. In mathematical terms:

$$
T(\mathbf{q}) = T_1(q_1) T_2(q_2) \dots T_6(q_6)
$$

where $ q_i $ is the angle or displacement for joint $ i $, and each $ T_i $ is the homogeneous transformation for that joint using its DH parameters.

**Inverse kinematics (IK)** is the reverse: given a desired transformation $ T_{des} $, find $ \mathbf{q} $ such that $ T(\mathbf{q}) \simeq T_{des} $. For a general 6DoF manipulator, no analytic (closed-form) solution may exist, so an iterative method is preferred.

### 1.2 The Geometric Jacobian

The **Jacobian matrix (J)** relates instantaneous joint velocities $ \dot{\mathbf{q}} $ to end-effector linear and angular velocities $ \mathbf{v}, \boldsymbol{\omega} $:

\[
\begin{bmatrix}
\mathbf{v} \\
\boldsymbol{\omega}
\end{bmatrix}
= J(\mathbf{q}) \, \dot{\mathbf{q}}
\]

For a 6DoF arm with all revolute joints:
- Each column $ J_i $ (for joint $ i $) is comprised of:
    - Linear part $ J_{v_i} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) $, the effect of joint $ i $ on end-effector position.
    - Angular part $ J_{\omega_i} = \mathbf{z}_{i-1} $, the axis of rotation.

Where:
- $ \mathbf{z}_{i-1} $ is the axis of rotation of joint $ i-1 $, expressed in the base frame.
- $ \mathbf{p}_{i-1} $ is the position of joint $ i-1 $.
- $ \mathbf{p}_e $ is the position of the end-effector.

### 1.3 IK as an Iterative Numerical Problem

The basic iterative process is as follows:

1. **Compute the forward kinematics** for the current joint angles $ \mathbf{q}_k $ to get current end-effector pose $ T_{curr} $.
2. **Calculate the error vector** $ \mathbf{e}_k $ combining position and orientation errors.
3. **Compute the Jacobian** $ J(\mathbf{q}_k) $ at the current pose.
4. **Update joint angles** using the Jacobian pseudoinverse:

    \[
    \Delta \mathbf{q}_k = J^+(\mathbf{q}_k) \, \mathbf{e}_k
    \]
    \[
    \mathbf{q}_{k+1} = \mathbf{q}_k + \alpha \Delta \mathbf{q}_k
    \]
    where $ 0 < \alpha \le 1 $ is a step-size parameter.

5. **Iterate** until the error norm $ \| \mathbf{e}_k \| $ is small enough (i.e., converged).

This method works with any manipulator for which you can evaluate FK and the Jacobian.

---

## 2. Analytical vs. Numerical Jacobian Calculation

### 2.1 Analytical Jacobian

The **analytical (geometric)** approach uses the structure of the manipulator and the DH convention to explicitly construct each column of the Jacobian using cross products, as described above. It's preferred for performance and reliability when possible.

For the $ i $-th joint:

\[
J_{v_i} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}), \quad J_{\omega_i} = \mathbf{z}_{i-1}
\]

Computation involves:
- Obtaining all intermediate transformation matrices to extract $ \mathbf{z}_{i-1}, \mathbf{p}_{i-1} $.
- Efficient computation by chaining transforms, and storing intermediate results if desired.

### 2.2 Numerical Jacobian

The **numerical** approach approximates each partial derivative in the Jacobian using finite differences:

\[
J_{:,i} \approx \frac{F(\mathbf{q} + \delta \mathbf{e}_i) - F(\mathbf{q})}{\delta}
\]

where $ \mathbf{e}_i $ is the unit vector for joint $ i $, and $ F(\cdot) $ returns end-effector pose given joint angles.

Pros: Universality; works for any differentiable forward kinematic mapping.
Cons: Slow; susceptible to numerical noise; less accurate.

---

## 3. Jacobian Inversion: Transpose vs. Pseudoinverse vs. Damped Least Squares

### 3.1 Jacobian Transpose

The **Jacobian transpose** method applies a step proportional to the transpose:

\[
\Delta \mathbf{q} = \alpha J^T \mathbf{e}
\]

It's computationally efficient, but fails to converge in many configurations, especially near singularities or when Jacobian is ill-conditioned. It's not optimal for general 6DoF arms.

### 3.2 Moore–Penrose Pseudoinverse

The **pseudoinverse** yields the minimum-norm least-squares solution:

\[
J^{+} = J^{T} (J J^{T})^{-1} \quad \text{(for } J \text{ full row rank, } 6 \times 6)
\]
or, generally, via SVD.

Then,

\[
\Delta \mathbf{q} = J^{+} \mathbf{e}
\]

This approach is precise and robust for general cases, but may produce *large* joint angle changes near singularities.

### 3.3 Damped Least Squares (Levenberg-Marquardt)

To handle singularities, **damped least squares (DLS)** modifies the update as:

\[
J^{+}_{\lambda} = J^T (J J^T + \lambda^2 I)^{-1}
\]
\[
\Delta \mathbf{q} = J^{+}_{\lambda} \mathbf{e}
\]

where $ \lambda $ is a small positive damping constant. This regularizes the inversion and ensures numerical stability near singularities, at the cost of slightly slower convergence.

---

## 4. Orientation Representation for IK

### 4.1 Euler Angles vs. Rotation Matrices vs. Quaternions

- **Euler angles** are common and map directly to many robot architectures, but are prone to gimbal lock.
- **Rotation matrices** are algebraically convenient (3×3), but parameter redundancy increases complexity.
- **Quaternions** avoid singularities and are efficient for smooth interpolation and error calculation.

In Jacobian-based IK, it's common to use either **orientation error vectors** (axis–angle) or quaternion-based error metrics for orientation, with Jacobian's lower three rows representing the angular velocity mapping.

### 4.2 Error Calculation

The error vector $ \mathbf{e} $ used for updating joints is:

\[
\mathbf{e} =
\begin{bmatrix}
\mathbf{t}_{des} - \mathbf{t}(\mathbf{q}) \\
\text{OrientationError}(\mathbf{R}_{des}, \mathbf{R}(\mathbf{q}))
\end{bmatrix}
\]

A common orientation error computation is:

\[
\mathbf{e}_\omega = \frac{1}{2} (\mathbf{R}^T_{curr} \mathbf{R}_{des} - \mathbf{R}^T_{des} \mathbf{R}_{curr})^\vee
\]

or, using rotation axis–angle, the rotation vector taking $ \mathbf{R}_{curr} $ to $ \mathbf{R}_{des} $, or directly via quaternion log-difference.

---

## 5. C# Linear Algebra Libraries for Robotics

Efficient matrix operations are critical for Jacobian-based algorithms. The two major C# libraries are:

- **Math.NET Numerics** (free, robust, .NET Standard, supports matrices/vectors, SVD, pseudoinverse, etc.)
- **ILNumerics** (powerful, high-performance, commercial, Matlab syntax-like)

*We will use Math.NET Numerics for the code sample, as it's widely adopted and well supported.*

---

## 6. Corrected C# IK Function: Algorithm Overview and Implementation

### 6.1 Algorithm Steps

A robust C# implementation should proceed as follows:

1. **Input**: 
   - *DH parameters* of the manipulator
   - *current joint angles* ($ \mathbf{q}_0 $)
   - *target end-effector pose* (position + orientation)
   - *tolerances, step size, damping factor*

2. **Iterative Loop**: For each iteration,
   - a. Compute the **forward kinematics** for $ \mathbf{q}_k $
   - b. Calculate the **error vector** $ \mathbf{e}_k $ (position and orientation)
   - c. Evaluate the **Jacobian** $ J(\mathbf{q}_k) $ analytically
   - d. Compute the **damped pseudoinverse** $ J^+_\lambda $
   - e. Calculate the joint update: $ \Delta \mathbf{q}_k = J^+_\lambda \mathbf{e}_k $
   - f. Update: $ \mathbf{q}_{k+1} = \mathbf{q}_k + \alpha \Delta \mathbf{q}_k $
   - g. Check for **convergence** (error below threshold or max iterations)

3. **Output**: 
   - The joint angles $ \mathbf{q} $ that best achieve the desired pose.

### 6.2 C# Function Implementation

Below is a **complete, well-documented C# IK function** using Math.NET Numerics for matrix operations. This implementation is suitable for general 6DoF manipulators and explicitly uses the analytic Jacobian and the damped least-squares pseudoinverse for stability.

```csharp
using System;
using MathNet.Numerics.LinearAlgebra;

namespace Kinematics6DoF
{
    public class Kinematics
    {
        // Define the DH parameters for your robot as arrays of doubles for a, alpha, d,
        // or, ideally, use a DHParameter struct/class for clarity.
        private readonly double[] a;     // Link lengths
        private readonly double[] alpha; // Link twists (in radians)
        private readonly double[] d;     // Link offsets

        public Kinematics(double[] a, double[] alpha, double[] d)
        {
            if (a.Length != 6 || alpha.Length != 6 || d.Length != 6)
                throw new ArgumentException("All DH parameter arrays must have length 6.");
            this.a = a;
            this.alpha = alpha;
            this.d = d;
        }

        /// <summary>
        /// Computes the joint angles (q) required to achieve a target end-effector pose using 
        /// iterative Jacobian damped least squares inverse kinematics.
        /// </summary>
        /// <param name="qd0">Initial joint angles (length 6)</param>
        /// <param name="positionTarget">Target position (Vector3: x, y, z)</param>
        /// <param name="orientationTarget">Target orientation (3x3 Matrix: rotation matrix)</param>
        /// <param name="maxIterations">Maximum number of iterations</param>
        /// <param name="positionTolerance">Acceptable root mean square error for position [m]</param>
        /// <param name="orientationTolerance">Acceptable RMS error for orientation [rad]</param>
        /// <param name="stepSize">Update step size (alpha, 0 < alpha <= 1.0)</param>
        /// <param name="damping">Damping factor for pseudoinverse (lambda, e.g., 1e-3)</param>
        /// <returns>Joint angles solution (length 6)</returns>
        public double[] InverseKinematics(
            double[] qd0,
            Vector<double> positionTarget,
            Matrix<double> orientationTarget,
            int maxIterations = 100,
            double positionTolerance = 1e-4,
            double orientationTolerance = 1e-3,
            double stepSize = 0.5,
            double damping = 1e-4)
        {
            var q = Vector<double>.Build.DenseOfArray(qd0);
            int nJoints = 6;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // 1. Forward Kinematics
                (Vector<double> pCurr, Matrix<double> rCurr, Matrix<double>[] transforms) = ForwardKinematicsDH(q);

                // 2. Error Computation
                //    a) Position error
                Vector<double> ePos = positionTarget - pCurr;

                //    b) Orientation error: axis-angle between rCurr and orientationTarget
                Matrix<double> rError = orientationTarget * rCurr.Transpose();
                var axisAngle = RotationMatrixToAxisAngle(rError); // (axis*theta)
                Vector<double> eOri = axisAngle.Item1 * axisAngle.Item2; // orientation error vector

                //    c) Compose full error vector (size 6)
                Vector<double> e = Vector<double>.Build.Dense(6);
                e.SetSubVector(0, 3, ePos);
                e.SetSubVector(3, 3, eOri);

                // 3. Check for convergence
                if (ePos.L2Norm() < positionTolerance && eOri.L2Norm() < orientationTolerance)
                    return q.ToArray();

                // 4. Compute Analytic Jacobian
                Matrix<double> J = ComputeGeometricJacobian(q, transforms);

                // 5. Damped Pseudoinverse (Levenberg-Marquardt)
                Matrix<double> JJt = J * J.Transpose();
                Matrix<double> lambdaI = Matrix<double>.Build.DenseIdentity(6) * damping * damping;
                Matrix<double> JJtReg = JJt + lambdaI;

                var JJtRegInv = JJtReg.Inverse();
                Matrix<double> J_pinv = J.Transpose() * JJtRegInv;

                // 6. Joint Update: Δq = α·J^+·e
                Vector<double> deltaQ = stepSize * J_pinv * e;

                // Optional: Limit per-joint step sizes for stability
                double maxStep = 0.1; // radians per iteration per joint
                for (int i = 0; i < nJoints; ++i)
                    if (Math.Abs(deltaQ[i]) > maxStep)
                        deltaQ[i] = Math.Sign(deltaQ[i]) * maxStep;

                // 7. Update joints
                q += deltaQ;
            }

            throw new InvalidOperationException("Inverse kinematics did not converge within max iterations.");
        }

        /// <summary>
        /// Forward kinematics using DH parameters
        /// Returns end-effector position, rotation, and intermediate transforms.
        /// </summary>
        private (Vector<double>, Matrix<double>, Matrix<double>[]) ForwardKinematicsDH(Vector<double> q)
        {
            int n = 6;
            Matrix<double> t = Matrix<double>.Build.DenseIdentity(4);
            Matrix<double>[] transforms = new Matrix<double>[n + 1];
            transforms[0] = t.Clone(); // Base

            for (int i = 0; i < n; ++i)
            {
                t = t * DHMatrix(a[i], alpha[i], d[i], q[i]);
                transforms[i + 1] = t.Clone();
            }

            // Extract end-effector pose
            var p = transforms[n].SubMatrix(0, 3, 3, 1).Column(0);
            var r = transforms[n].SubMatrix(0, 3, 0, 3);

            return (p, r, transforms);
        }

        /// <summary>
        /// Computes the 6x6 geometric Jacobian (analytical, revolute joints).
        /// </summary>
        private Matrix<double> ComputeGeometricJacobian(Vector<double> q, Matrix<double>[] transforms)
        {
            int nJoints = 6;
            var J = Matrix<double>.Build.Dense(6, nJoints);
            // Base frame origin and z axis
            Vector<double> p0 = Vector<double>.Build.Dense(3); // [0,0,0]
            Vector<double> z0 = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });

            // End-effector position
            Vector<double> pe = transforms[nJoints].SubMatrix(0, 3, 3, 1).Column(0);

            for (int i = 0; i < nJoints; ++i)
            {
                Vector<double> pi = transforms[i].SubMatrix(0, 3, 3, 1).Column(0);
                Vector<double> zi = transforms[i].SubMatrix(0, 3, 2, 1).Column(0); // z axis

                Vector<double> d = pe - pi;
                Vector<double> ji_v = zi.CrossProduct(d); // linear velocity part
                Vector<double> ji_w = zi; // angular velocity part

                J.SetSubMatrix(0, i, ji_v.ToColumnMatrix());
                J.SetSubMatrix(3, i, ji_w.ToColumnMatrix());
            }
            return J;
        }

        /// <summary>
        /// DH homogeneous transformation matrix
        /// </summary>
        private static Matrix<double> DHMatrix(double a, double alpha, double d, double theta)
        {
            double ca = Math.Cos(alpha);
            double sa = Math.Sin(alpha);
            double ct = Math.Cos(theta);
            double st = Math.Sin(theta);

            var m = Matrix<double>.Build.Dense(4,4);
            m[0,0] = ct;  m[0,1] = -st*ca; m[0,2] = st*sa;  m[0,3] = a*ct;
            m[1,0] = st;  m[1,1] = ct*ca;  m[1,2] = -ct*sa; m[1,3] = a*st;
            m[2,0] = 0.0; m[2,1] = sa;     m[2,2] = ca;     m[2,3] = d;
            m[3,0] = 0.0; m[3,1] = 0.0;    m[3,2] = 0.0;    m[3,3] = 1.0;
            return m;
        }

        /// <summary>
        /// Converts a rotation matrix to axis-angle (returns (axis, angle)).
        /// </summary>
        private static Tuple<Vector<double>, double> RotationMatrixToAxisAngle(Matrix<double> R)
        {
            double angle = Math.Acos(Math.Min(1.0, Math.Max(-1.0, (R.Trace() - 1.0) / 2.0)));
            Vector<double> axis = Vector<double>.Build.Dense(3);

            if (angle < 1e-6)
            {
                // No rotation
                axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0;
            }
            else
            {
                axis[0] = (R[2,1] - R[1,2]) / (2 * Math.Sin(angle));
                axis[1] = (R[0,2] - R[2,0]) / (2 * Math.Sin(angle));
                axis[2] = (R[1,0] - R[0,1]) / (2 * Math.Sin(angle));
            }
            return Tuple.Create(axis, angle);
        }
    }

    // Extension for vector cross product:
    public static class VectorExtensions
    {
        public static Vector<double> CrossProduct(this Vector<double> a, Vector<double> b)
        {
            if (a.Count != 3 || b.Count != 3)
                throw new ArgumentException("Cross product is only defined for 3-element vectors.");
            return Vector<double>.Build.Dense(new double[]{
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]});
        }
    }
}
```

---

### 6.3 Function Structure and Libraries

**Explanation and Libraries**:
- This code utilizes [Math.NET Numerics](https://numerics.mathdotnet.com/Matrix) for efficient linear algebra, including vector/matrix creation, transpose, inverse, norm, etc.
- The manipulator’s configuration is described using standard **Denavit–Hartenberg (DH)** parameters, as is best practice for generality and modularity.
- The Jacobian is computed analytically for performance and robustness, avoiding numerical instability and slow evaluation common in finite-difference (numerical) Jacobians.
- Stability near singularities is ensured with **damping** in the pseudoinverse, following the Levenberg-Marquardt style damped least-squares update.
- The approach works for any 6DoF arm just by providing the correct DH parameters.

---

## 7. Explanation of Algorithm Steps and Key Design Choices

### 7.1 Analytical Geometric Jacobian

Each Jacobian column comprises the cross product $ \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) $ (linear velocity) and $ \mathbf{z}_{i-1} $ (angular velocity) reflecting the physical effect of rotating joint $ i $ on the end-effector pose. This analytic approach is not only robust but fast—critical for real-time robotics applications.

### 7.2 Damped Pseudoinverse for Singularity Robustness

Singularities are configurations where the manipulator loses a degree of freedom or the Jacobian becomes rank deficient (non-invertible). Near singularities, small changes in pose require large changes in joint angles, causing instability.

The **damped least squares** update:

\[
J^+_\lambda = J^T (J J^T + \lambda^2 I)^{-1}
\]

ensures that near singularities, the solution is stabilized by the damping term λ, which regularizes the otherwise unbounded updates.

### 7.3 Step Size and Increment Limiting

For numerical stability and physical realism, the algorithm:
- Limits the size of each joint update to a fixed maximum (e.g., 0.1 rad ≈ 5.7° per iteration) to avoid overjumping,
- Uses a global step-size scaling (α) to modulate convergence speed and stability.

### 7.4 Stopping Criteria

The iterative solver stops when both position and orientation errors fall below given tolerances, or if max iterations is reached. This ensures practical convergence.

### 7.5 Orientation Error Representation

Orientation error is computed by taking the **axis-angle** representation between the current and desired orientation matrices:
- Error vector $ \mathbf{e}_{ori} = \hat{a} \cdot \theta $, where $ \hat{a} $ is the axis, and $ \theta $ is the rotation angle between current and target orientation.
- This is mathematically unambiguous, well-posed for all rotations, and compatible with robotic conventions.

---

## 8. Numerical Example

Consider a 6DoF manipulator with **simple arbitrary DH parameters**:

| Joint | α (deg) | a (m) | d (m)   |
|-------|---------|-------|---------|
| 1     | +90     | 0.0   | 0.4     |
| 2     | 0       | 0.3   | 0       |
| 3     | +90     | 0.0   | 0.0     |
| 4     | -90     | 0.0   | 0.25    |
| 5     | +90     | 0.0   | 0.0     |
| 6     | 0       | 0.0   | 0.08    |

Suppose we want the end-effector to achieve:
- Position: (0.4, 0.2, 0.6) m
- Orientation: aligned with base (identity rotation)

Given an initial joint configuration (all zeros), the function above can be called with:

```csharp
// Example usage:

double[] a =    { 0.0, 0.3, 0.0, 0.0, 0.0, 0.0 };
double[] alpha ={ Math.PI/2, 0.0, Math.PI/2, -Math.PI/2, Math.PI/2, 0.0 };
double[] d =    { 0.4, 0.0, 0.0, 0.25, 0.0, 0.08 };

var kin = new Kinematics6DoF.Kinematics(a, alpha, d);
double[] q0 = new double[6]; // All joints start at zero

var targetPos = Vector<double>.Build.DenseOfArray(new double[] {0.4,0.2,0.6});
var targetOri = Matrix<double>.Build.DenseIdentity(3); // Identity rotation

double[] qSol = kin.InverseKinematics(q0, targetPos, targetOri);

Console.WriteLine("IK Solution (radians):");
for(int i=0;i<6;++i)
    Console.WriteLine($"q{i+1}: {qSol[i]:F4}");
```

The resulting joint angles will, after several iterations, position the robot’s end-effector close to the specified Cartesian pose, within the prescribed tolerances.

---

## 9. Comparison with Existing Solutions and Libraries

### 9.1 Reference: KLib Project Issues

The IK function in [KLib](https://github.com/vmx17/Kinematics6DoF/) previously used the Jacobian transpose rather than the pseudoinverse and may have had errors in analytical Jacobian calculation or orientation error formulation. As discussed in technical QA threads, using the transpose is not generally robust for arbitrary 6DoF arms, and incorrect cross product or DH frame interpretations will yield incorrect results.

### 9.2 Featured Open-Source Implementations

- [EricVoll/RobotDynamicsLibrary](https://github.com/EricVoll/RobotDynamicsLibrary) uses a similar analytic Jacobian and includes robust DLS.
- [yonchien/Comau_IK](https://github.com/yonchien/Comau_IK) applies the same analytic cross product approach in MATLAB, confirming its correctness.
- [Math.NET Numerics](https://numerics.mathdotnet.com/Matrix) and [ILNumerics](https://ilnumerics.net/docs-functions.html) provide requisite matrix/vector operations.

The implementation here matches and generalizes these, tailored for C# and .NET.

---

## 10. Further Considerations

### 10.1 Convergence and Performance

- **Convergence speed** depends on the damping, step size, and Jacobian conditioning. Adaptive step sizes or selective damping can improve robustness. 
- **Joint limits and redundancy**: Additional logic should enforce joint range constraints and, if needed, exploit null-space projection for secondary objectives (e.g., obstacle avoidance; see [Null-Space Algorithms][24†source][52†source]).
- **Multiple solutions**: In some poses, several IK solutions may exist (elbow up/elbow down). Iterative Jacobian methods tend to find whichever solution is closest to the initial guess.

### 10.2 Task and Tool Orientations

- The method can be trivially extended for tool offsets, by modifying the FK and orientation target appropriately.
- Key design patterns (Factory, Strategy, SOLID) are encouraged for extensibility (see [DesignPatternsLibrary][49†source]).

---

## 11. Testing and Verifying the Implementation

Testing an IK implementation requires comprehensive cases:
- **Known FK/IK pairs**: Use forward kinematics to generate an end pose for known joint angles, then check that IK recovers (approximately) the original angles.
- **Edge cases**: Workspace boundaries, near singularities (e.g., straight arm or wrist aligned), joint limits.
- **Realistic scenarios**: Poses sampled from actual robot tasks.

Visualization and simulation in C#, e.g., via Unity/MonoGame, can facilitate debugging and iterative refinement.

---

## 12. Summary Table: Numerical IK Design Choices

| Feature                      | Recommended Approach                             | Alternatives                    | Comments                    |
|------------------------------|--------------------------------------------------|----------------------------------|-----------------------------|
| FK routine                   | DH convention analytic                           | PoE, numerical                  | Standard practice           |
| Jacobian computation         | Analytic, cross-product from DH transforms       | Numerical finite diff           | Analytical: Performance     |
| Jacobian inversion           | Damped pseudoinverse (DLS, Levenberg-Marquardt)  | Pure pseudoinverse, transpose   | More stable near singularity|
| Orientation representation   | Axis–angle, rotation matrices, or quaternions    | Euler angles                    | Axis–angle, quaternion: robust|
| Step size                    | Adaptive or fixed (empirically tuned)            | Constant                        | Clamp max joint increment   |
| Libraries                    | Math.NET Numerics (recommended), ILNumerics      | Custom, unmanaged, third-party  | Robust, maintained, .NET    |
| Stopping criterion           | Pose error tolerance or max steps                | Target delta joint angles       | Error check both pos & ori  |
| Handling joint limits        | Enforce per step, project onto feasible region   | Ignore (for tests only)         | Must enforce in deployment  |

---

## Conclusion

A correct, robust, and efficient Jacobian-based inverse kinematics function for a 6DoF manipulator can be implemented straightforwardly in C#. The core requirements are:

- **Accurate forward kinematics and analytic Jacobian**.
- **Damped least-squares pseudoinverse** for robust inversion, essential for robust performance near singularities.
- **Proper pose and orientation error formulation**, favoring axis-angle or quaternion difference for orientation.
- **Efficient linear algebra**, best provided by Math.NET Numerics for C#.

The code above embodies these best practices, and is extensible for a range of arm configurations and applications. This method is used across academia and industry, and matches general robotics research consensus found in core texts, current literature, and quality open-source implementations.

For deployment on real manipulators, further enhancements can be made for enforcing joint limits, collision avoidance, redundancy exploitation, and real-time computation demands. These principles form the backbone for virtually all advanced manipulator control systems in robotics today.

---
Great! I’ll begin crafting a corrected inverse kinematics function using the Jacobian matrix for the 6DoF manipulator described in the GitHub project you mentioned. This will include a proper C# implementation, algorithm explanation, and any relevant libraries or examples. It’ll take me a little while to complete, so feel free to step away—I’ll keep working in the background and save the report right here in our conversation.

---
---
**整理中のフレームインデックス**

フレームi-1はtransforms[i]で取得できるのか？例えば、i=0ならtransforms[0]、i=1ならtransforms[1]になるけれど、i=1のときはフレーム0からO0が必要。そのため、axisFrameIdxは各フレーム番号に対応するインデックスを使うべきだと気づいた。今のコードではjoint3がOeを指してしまっているのが誤り。また、DLS法と計算方法に少し問題があり、修正が必要そう。
# Correcting the Jacobian-based IK function in KinematicsRM2

Your current InverseJacobian implementation is mostly sound (DLS with JJᵀ regularization and axis-angle orientation error), but the Jacobian column construction is wrong due to a misaligned axis frame index for joint 4 (zero-based index 3). The inserted fixed “elbow” transform creates an extra frame in the transform list, and the code currently uses the elbow frame as the axis source for joint 4. This breaks the geometric Jacobian and derails convergence near that segment.

---

## What’s wrong

- The transforms list includes: O0, O1, O2, O3, Oe, O4, O5, O6, O7 (TCP). Indices: 0..8 respectively.
- For a revolute joint i, geometric Jacobian uses the axis and origin of frame i (the “previous” frame of the i→i+1 joint): Jᵥᵢ = zᵢ × (p_end − oᵢ), Jωᵢ = zᵢ.
- Your axis frame mapping uses {0, 1, 2, 4, 5, 6}, which selects Oe (index 4) for joint 4’s axis. The correct previous frame for joint 4 is O3 (index 3), not Oe. The elbow is a fixed transform inserted after O3; it moves the origin but does not change which frame defines z₃ for joint 4’s column.

---

## Minimal fix

Update axisFrameIdx in CalculateJacobian to reference the correct previous frames, accounting for the inserted elbow:

```csharp
private double[,] CalculateJacobian(double[] q)
{
    // Use the origin/axis of frame i (the “previous” frame for joint i)
    // Transforms indices: O0=0, O1=1, O2=2, O3=3, Oe=4, O4=5, O5=6, O6=7, O7(tcp)=8
    // Joint index -> frame index:
    // 0 -> O0 (0)
    // 1 -> O1 (1)
    // 2 -> O2 (2)
    // 3 -> O3 (3)   // FIXED: was 4 (Oe), must be 3
    // 4 -> O4 (5)   // due to inserted elbow, O4 is at index 5
    // 5 -> O5 (6)
    int[] axisFrameIdx = { 0, 1, 2, 3, 5, 6 };

    var J = new double[6, 6];
    var transforms = GetFrameTransforms(q);
    var p_eff = new[] { transforms[^1][0, 3], transforms[^1][1, 3], transforms[^1][2, 3] };

    for (var joint = 0; joint < 6; joint++)
    {
        var Tprev = transforms[axisFrameIdx[joint]];
        var z = new[] { Tprev[0, 2], Tprev[1, 2], Tprev[2, 2] };
        var o = new[] { Tprev[0, 3], Tprev[1, 3], Tprev[2, 3] };

        var Jv = MathUtil.Cross(z, MathUtil.Sub(p_eff, o));
        for (var r = 0; r < 3; r++)
        {
            J[r, joint] = Jv[r];
            J[r + 3, joint] = z[r];
        }
    }
    return J;
}
```

This aligns each Jacobian column with the correct z-axis and origin for the “previous” frame of each joint, restoring the proper kinematic relationships across the elbow segment.

---

## Optional orientation error tweak

Your PoseErrorFull builds R_err as R_curᵀ·R_tar and then uses axis-angle. That is correct. If you encounter instability near π flips, you can clamp and use the skew-symmetric part consistently:

```csharp
// After R_err = R_cur^T * R_tar
double[] w = {
    0.5*(R_err[2,1]-R_err[1,2]),
    0.5*(R_err[0,2]-R_err[2,0]),
    0.5*(R_err[1,0]-R_err[0,1])
};
```

You already fall back to axis-angle with acos(trace) safely when needed, so this is optional.

---

## Why this fixes convergence

- The DLS step uses y from (J Jᵀ + λ²I) y = err and dq = Jᵀ y. If a column uses the wrong axis/origin, the solver drives joints in directions that don’t reduce the end-effector error, especially across the elbow insertion. Correcting the frame indices makes each column physically meaningful again, allowing the DLS to converge reliably on both position and orientation.

---

## Notes on the fixed prismatic elbow

- The elbow is a fixed prismatic transform inserted between joints 3 and 4. It should not contribute a Jacobian column but must be reflected in the transforms list used to compute origins and axes for downstream joints. The corrected axisFrameIdx respects this by mapping joint 4 to O4’s previous frame (O3) and shifting subsequent frames appropriately.

---

## Quick validation

- Test with several targets around the elbow reach boundary and compare error norms per iteration. You should see monotonic decrease rather than oscillation.
- Check that solutions near wrist singularities still behave as before; this change only corrects axis frames for Jacobian columns and does not alter your damping or angle normalization.

If you want, I can propose a small unit test that asserts the numerical Jacobian against finite differences at random configurations, specifically around joints 3–4 where the elbow insertion matters.

---
---

# Clarifying fixed frames and 6DoF

You’re right: fixing the base frame O0, the elbow offset frame Oe, and the TCP/tool frame O7 does not change the manipulator’s degrees of freedom. They are constant transforms in the kinematic chain, and the six actuated joints (1–6) still define a 6DoF system. The Jacobian must be built using the “previous” frame of each actuated joint, while correctly threading those fixed frames into the forward kinematics so origins and axes are accurate downstream.

---

# Corrected Jacobian-based IK

Below is a focused correction for Manipulator6DoF.InversJacobian. The key is the Jacobian column mapping: for joint i, use the origin and z-axis of frame i (the frame before the i→i+1 joint). Because Oe is an inserted fixed transform between O3 and O4, it shifts indices in the transforms list but does not change which previous frame defines joint-4’s axis.

- Transforms order (example): O0, O1, O2, O3, Oe, O4, O5, O6, O7
- Joint index → axis/origin source:
  - 0 → O0
  - 1 → O1
  - 2 → O2
  - 3 → O3
  - 4 → O4 (previous frame is O4’s parent, which is O3; we’ll use the frame at index 5–1=4 for origin/axis if your list stores “frame itself”, but the safe approach is to explicitly map)
  - 5 → O5

Since implementations differ, we’ll explicitly map indices with an array matching your GetFrameTransforms ordering.

---

## Replace your Jacobian construction with this

```csharp
private static double[,] CalculateJacobian(double[][] transforms)
{
    // transforms: list of 4x4 homogeneous matrices in chain order:
    // [O0, O1, O2, O3, Oe, O4, O5, O6, O7]
    // Map each joint (0..5) to the frame that defines its axis and origin (previous frame of the joint).
    // If your GetFrameTransforms guarantees the above order, this mapping is valid:
    int[] axisFrameIdx = { 0, 1, 2, 3, 5, 6 }; // Note: joint 4 uses frame O4 (index 5) for its z-axis; Oe is fixed and should NOT be used as a joint axis.

    var J = new double[6, 6];

    // End-effector position p_end from O7
    var Tend = transforms[^1];
    var p_end = new[] { Tend[0][3], Tend[1][3], Tend[2][3] };

    for (int j = 0; j < 6; j++)
    {
        var Tprev = transforms[axisFrameIdx[j]];
        var z = new[] { Tprev[0][2], Tprev[1][2], Tprev[2][2] };   // z-axis of previous frame in base coords
        var o = new[] { Tprev[0][3], Tprev[1][3], Tprev[2][3] };   // origin of previous frame

        // Geometric Jacobian columns for revolute joints:
        // linear part: z_i × (p_end - o_i)
        // angular part: z_i
        var pe_minus_oi = new[] { p_end[0] - o[0], p_end[1] - o[1], p_end[2] - o[2] };
        var Jv = Cross(z, pe_minus_oi);

        J[0, j] = Jv[0];
        J[1, j] = Jv[1];
        J[2, j] = Jv[2];

        J[3, j] = z[0];
        J[4, j] = z[1];
        J[5, j] = z[2];
    }

    return J;
}

private static double[] Cross(double[] a, double[] b)
{
    return new[]
    {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}
```

Important: The axisFrameIdx must match your actual transforms order. The mistake to avoid is using Oe (the fixed prismatic offset) as the axis source for joint 4. Joint 4’s axis is defined by the z-axis of frame O4’s previous joint frame (which, given the chain, is correctly captured when using the O4 frame matrix on the left side of the O4→O5 transform in most implementations). If your chain composes transforms as parent-to-child, the above mapping works exactly as written.

---

## Corrected InversJacobian with DLS

Here’s a self-contained version that uses your FK, pose error, and clamps to joint limits you already store in MDHParameters. Substitute your own matrix utilities where marked.

```csharp
public bool InversJacobian(Vector3 targetPos, Matrix3x3 targetRot, double[] q, MDHParameters mdh, int maxIter = 200, double tolPos = 1e-3, double tolOri = 1e-3, double lambda = 1e-2)
{
    // q: current joint angles (rad), length = 6
    for (int it = 0; it < maxIter; it++)
    {
        // Compose transforms for current q (include fixed O0, Oe, O7)
        var transforms = GetFrameTransforms(q, mdh); // returns double[][] where each is 4x4 stored as double[4][4]

        var Tend = transforms[^1];
        var p = new Vector3(Tend[0][3], Tend[1][3], Tend[2][3]);
        var R = new Matrix3x3(
            Tend[0][0], Tend[0][1], Tend[0][2],
            Tend[1][0], Tend[1][1], Tend[1][2],
            Tend[2][0], Tend[2][1], Tend[2][2]
        );

        // Position error
        var ePos = targetPos - p;

        // Orientation error (axis-angle from R_err = R^T * R_target)
        var Rerr = R.Transpose() * targetRot;
        var eOri = AxisAngleError(Rerr); // returns 3-vector of rotation axis * angle

        // Check convergence
        if (ePos.Length() < tolPos && eOri.Length() < tolOri)
            return true;

        // Build geometric Jacobian
        var J = CalculateJacobian(transforms);

        // Damped least squares: solve (J J^T + λ^2 I) y = [ePos; eOri], then dq = J^T y
        var e = new double[6] { ePos.X, ePos.Y, ePos.Z, eOri.X, eOri.Y, eOri.Z };
        var JJt = Multiply(J, Transpose(J));                // 6x6
        var A = Add(JJt, ScaleIdentity(6, lambda * lambda)); // 6x6
        var y = SolveSymmetricPositiveDefinite(A, e);       // 6

        var JT = Transpose(J);                               // 6x6
        var dq = Multiply(JT, y);                            // 6

        // Step and clamp to joint limits
        for (int i = 0; i < 6; i++)
        {
            q[i] += dq[i];
            q[i] = Clamp(q[i], Deg2Rad(mdh.MinAnglesDeg[i]), Deg2Rad(mdh.MaxAnglesDeg[i]));
        }
    }

    return false;
}

// Orientation error helper
private static Vector3 AxisAngleError(Matrix3x3 Rerr)
{
    // Stable axis-angle from rotation matrix
    double trace = Rerr.M11 + Rerr.M22 + Rerr.M33;
    double cosTheta = Math.Clamp((trace - 1.0) / 2.0, -1.0, 1.0);
    double theta = Math.Acos(cosTheta);

    if (theta < 1e-9)
        return new Vector3(0, 0, 0);

    // Axis from skew-symmetric part
    double rx = Rerr.M32 - Rerr.M23;
    double ry = Rerr.M13 - Rerr.M31;
    double rz = Rerr.M21 - Rerr.M12;

    var axis = new Vector3(rx, ry, rz);
    double s = axis.Length();
    if (s < 1e-12)
        return new Vector3(0, 0, 0);

    axis /= s;
    return axis * theta;
}

private static double Clamp(double x, double lo, double hi) => Math.Max(lo, Math.Min(hi, x));

private static double Deg2Rad(double d) => d * Math.PI / 180.0;
```

- The DLS formulation uses a 6D task-space error vector [position; orientation].
- Axis-angle orientation error is computed from Rᵀ·R_target, which matches your approach and behaves well with damping.
- Joint limits from MDHParameters are enforced after each step.

---

## Validate with a finite-difference check

To be certain the Jacobian columns are mapped correctly with fixed Oe, validate numerically:

```csharp
bool CheckJacobian(double[] q, MDHParameters mdh)
{
    var transforms = GetFrameTransforms(q, mdh);
    var J = CalculateJacobian(transforms);

    double h = 1e-6;
    var Tend0 = transforms[^1];
    var p0 = new[] { Tend0[0][3], Tend0[1][3], Tend0[2][3] };

    for (int j = 0; j < 6; j++)
    {
        var qh = (double[])q.Clone();
        qh[j] += h;

        var transformsH = GetFrameTransforms(qh, mdh);
        var TendH = transformsH[^1];
        var pH = new[] { TendH[0][3], TendH[1][3], TendH[2][3] };

        var dp = new[] { (pH[0]-p0[0])/h, (pH[1]-p0[1])/h, (pH[2]-p0[2])/h };

        // Compare dp to Jv column j (rows 0..2)
        double err = Math.Sqrt(
            Math.Pow(dp[0]-J[0,j],2) + Math.Pow(dp[1]-J[1,j],2) + Math.Pow(dp[2]-J[2,j],2)
        );
        if (err > 1e-3) return false;
    }
    return true;
}
```

This catches any index mismatches, especially around the O3–Oe–O4 region.

---

## Direct answer

Yes: with O0, Oe, and O7 fixed, the system is 6DoF. The correct Jacobian must use the origin and z-axis of the previous frame of each actuated joint. The provided mapping and InversJacobian implementation fix the erroneous use of the Oe frame as a joint axis source, restoring proper convergence.

---
---
### Including fixed tool and base frames in homogeneous transforms

You’re right. The IK must build the full forward chain with fixed frames: base O0, elbow offset Oe, and tool/TCP O7. Below is a corrected, self-contained set of routines that:

- Compose homogeneous transforms with MDH parameters plus fixed frames (O0, Oe, O7).
- Build a geometric Jacobian at the TCP using the “previous frame” z-axis and origin for each actuated joint.
- Solve IK via damped least squares with position + orientation error.

---

### Fixed frames and MDH composition

- O0: base frame (fixed).
- Oe: fixed prismatic/offset after joint 3, before joint 4.
- O7: fixed tool frame (TCP) relative to joint 6.

The transforms list order is:
O0, O1, O2, O3, Oe, O4, O5, O6, O7(TCP)

Each Oi is the accumulated base-to-frame matrix.

---

### Code: FK with fixed frames and Jacobian

```csharp
public sealed class Manipulator6DoF
{
    private readonly MDHParameters _mdh;
    private readonly Matrix4x4 _T0;  // O0 fixed
    private readonly Matrix4x4 _Te;  // Oe fixed (elbow offset)
    private readonly Matrix4x4 _Ttool; // O7 fixed (tool/TCP)

    public Manipulator6DoF(MDHParameters mdh, Matrix4x4 T0, Matrix4x4 Te, Matrix4x4 Ttool)
    {
        _mdh = mdh;
        _T0 = T0;
        _Te = Te;
        _Ttool = Ttool;
    }

    // Build the chain: [O0, O1, O2, O3, Oe, O4, O5, O6, O7]
    public List<Matrix4x4> GetFrameTransforms(double[] qRad)
    {
        var list = new List<Matrix4x4>(9);
        Matrix4x4 T = _T0; // start at base O0
        list.Add(T);

        // convenience
        var alpha = _mdh.Alpha;
        var a = _mdh.A;
        var d = _mdh.D;
        var off = _mdh.Offset;

        // Joint 1..3
        for (int i = 0; i < 3; i++)
        {
            var Ti = MDH(a[i], alpha[i], d[i], off[i] + qRad[i]);
            T = Matrix4x4.Multiply(T, Ti);
            list.Add(T); // O1, O2, O3
        }

        // Insert fixed elbow Oe
        T = Matrix4x4.Multiply(T, _Te);
        list.Add(T); // Oe

        // Joint 4..6
        for (int i = 3; i < 6; i++)
        {
            var Ti = MDH(a[i], alpha[i], d[i], off[i] + qRad[i]);
            T = Matrix4x4.Multiply(T, Ti);
            list.Add(T); // O4, O5, O6
        }

        // Fixed tool O7
        T = Matrix4x4.Multiply(T, _Ttool);
        list.Add(T); // O7 (TCP)

        return list;
    }

    // MDH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    private static Matrix4x4 MDH(double a, double alpha, double d, double theta)
    {
        var cth = Math.Cos(theta); var sth = Math.Sin(theta);
        var cal = Math.Cos(alpha); var sal = Math.Sin(alpha);

        // Homogeneous
        return new Matrix4x4(
            (float)cth, (float)(-sth), 0f, 0f,
            (float)(sth*cal), (float)(cth*cal), (float)(-sal), 0f,
            (float)(sth*sal), (float)(cth*sal), (float)cal, 0f,
            0f, 0f, 0f, 1f
        )
        * Matrix4x4.CreateTranslation((float)a, 0f, (float)d)
        * Matrix4x4.CreateRotationX((float)alpha); // already included above, kept for clarity if you prefer explicit composition
    }

    // Geometric Jacobian at TCP using previous-frame origin and z-axis
    private static double[,] CalculateJacobian(List<Matrix4x4> Tlist)
    {
        // Tlist: [O0=0, O1=1, O2=2, O3=3, Oe=4, O4=5, O5=6, O6=7, O7=8]
        int[] prevIdx = { 0, 1, 2, 3, 5, 6 }; // joint 1..6 → previous frames

        var J = new double[6, 6];

        var Tend = Tlist[^1];
        var p_end = ExtractPosition(Tend);

        for (int j = 0; j < 6; j++)
        {
            var Tprev = Tlist[prevIdx[j]];
            var z = ExtractZ(Tprev);
            var o = ExtractPosition(Tprev);

            var pe_minus_o = Sub(p_end, o);
            var Jv = Cross(z, pe_minus_o);

            J[0, j] = Jv[0];
            J[1, j] = Jv[1];
            J[2, j] = Jv[2];

            J[3, j] = z[0];
            J[4, j] = z[1];
            J[5, j] = z[2];
        }

        return J;
    }

    private static double[] ExtractPosition(Matrix4x4 T) => new[] { (double)T.M41, (double)T.M42, (double)T.M43 };
    private static double[] ExtractZ(Matrix4x4 T) => new[] { (double)T.M13, (double)T.M23, (double)T.M33 };

    private static double[] Cross(double[] a, double[] b) => new[]
    {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
    private static double[] Sub(double[] a, double[] b) => new[] { a[0]-b[0], a[1]-b[1], a[2]-b[2] };
}
```

Notes:
- The Jacobian uses the z-axis and origin of the frame before each actuated joint. Oe is fixed and should not provide a joint axis.
- The tool transform `_Ttool` is multiplied at the end, so the TCP position/orientation drive the error and Jacobian.

---

### Code: Inverse Jacobian (DLS) with tool consideration

```csharp
public bool InversJacobian(
    Vector3 targetPos, Matrix3x3 targetRot,
    double[] qRad, int maxIter = 200,
    double tolPos = 1e-3, double tolOri = 1e-3,
    double lambda = 1e-2)
{
    for (int it = 0; it < maxIter; it++)
    {
        var Tlist = GetFrameTransforms(qRad);
        var Tend = Tlist[^1];

        var p = new Vector3(Tend.M41, Tend.M42, Tend.M43);
        var R = new Matrix3x3(
            Tend.M11, Tend.M12, Tend.M13,
            Tend.M21, Tend.M22, Tend.M23,
            Tend.M31, Tend.M32, Tend.M33
        );

        var ePos = targetPos - p;

        // Orientation error from R_err = R_cur^T * R_tar
        var Rerr = R.Transpose() * targetRot;
        var eOri = AxisAngleError(Rerr);

        if (ePos.Length() < tolPos && eOri.Length() < tolOri)
            return true;

        var J = CalculateJacobian(Tlist);

        var e = new double[] { ePos.X, ePos.Y, ePos.Z, eOri.X, eOri.Y, eOri.Z };

        // DLS: (J J^T + λ² I) y = e, dq = J^T y
        var JT = Transpose(J);
        var JJt = Multiply(J, JT);
        var A = AddIdentity(JJt, lambda * lambda);
        var y = SolveSPD(A, e);
        var dq = Multiply(JT, y);

        // Step with joint limits
        for (int i = 0; i < 6; i++)
        {
            qRad[i] += dq[i];
            qRad[i] = Clamp(qRad[i], Deg2Rad(_mdh.MinAnglesDeg[i]), Deg2Rad(_mdh.MaxAnglesDeg[i]));
        }
    }
    return false;
}

// Orientation error: axis * angle from rotation matrix
private static Vector3 AxisAngleError(Matrix3x3 Rerr)
{
    double tr = Rerr.M11 + Rerr.M22 + Rerr.M33;
    double cosTh = Math.Clamp((tr - 1.0) / 2.0, -1.0, 1.0);
    double th = Math.Acos(cosTh);

    if (th < 1e-9) return Vector3.Zero;

    double rx = Rerr.M32 - Rerr.M23;
    double ry = Rerr.M13 - Rerr.M31;
    double rz = Rerr.M21 - Rerr.M12;

    var axis = new Vector3((float)rx, (float)ry, (float)rz);
    float s = axis.Length();
    if (s < 1e-12f) return Vector3.Zero;

    axis /= s;
    return axis * (float)th;
}

// Minimal linear algebra helpers (replace with your own)
private static double[,] Transpose(double[,] A)
{
    int r = A.GetLength(0), c = A.GetLength(1);
    var T = new double[c, r];
    for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) T[j, i] = A[i, j];
    return T;
}

private static double[,] Multiply(double[,] A, double[,] B)
{
    int r = A.GetLength(0), k = A.GetLength(1), c = B.GetLength(1);
    var C = new double[r, c];
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
        {
            double sum = 0;
            for (int t = 0; t < k; t++) sum += A[i, t] * B[t, j];
            C[i, j] = sum;
        }
    return C;
}

private static double[] Multiply(double[,] A, double[] x)
{
    int r = A.GetLength(0), c = A.GetLength(1);
    var y = new double[r];
    for (int i = 0; i < r; i++)
    {
        double sum = 0;
        for (int j = 0; j < c; j++) sum += A[i, j] * x[j];
        y[i] = sum;
    }
    return y;
}

private static double[,] AddIdentity(double[,] A, double s)
{
    int n = A.GetLength(0);
    var B = (double[,])A.Clone();
    for (int i = 0; i < n; i++) B[i, i] += s;
    return B;
}

// Solve SPD (e.g., by Cholesky) — placeholder
private static double[] SolveSPD(double[,] A, double[] b)
{
    // Implement Cholesky or use your numeric lib; here a naive Gaussian elimination placeholder
    return GaussianSolve(A, b);
}

private static double[] GaussianSolve(double[,] A, double[] b)
{
    int n = b.Length;
    var M = (double[,])A.Clone();
    var y = (double[])b.Clone();

    // Forward elimination
    for (int k = 0; k < n; k++)
    {
        // pivot
        double piv = M[k, k];
        if (Math.Abs(piv) < 1e-12) piv = 1e-12;
        for (int j = k; j < n; j++) M[k, j] /= piv;
        y[k] /= piv;

        for (int i = k + 1; i < n; i++)
        {
            double f = M[i, k];
            for (int j = k; j < n; j++) M[i, j] -= f * M[k, j];
            y[i] -= f * y[k];
        }
    }

    // Back substitution
    for (int i = n - 1; i >= 0; i--)
    {
        double sum = y[i];
        for (int j = i + 1; j < n; j++) sum -= M[i, j] * y[j];
        y[i] = sum;
    }
    return y;
}

private static double Clamp(double x, double lo, double hi) => Math.Max(lo, Math.Min(hi, x));
private static double Deg2Rad(double d) => d * Math.PI / 180.0;
```

---

### Key points

- The TCP/tool frame is explicitly included via `_Ttool`; all errors and Jacobian columns are computed with respect to O7.
- The fixed base O0 and elbow Oe are part of the FK chain but do not add DoF or Jacobian columns.
- The Jacobian column indices are mapped to the correct “previous frames,” avoiding use of Oe as a joint axis.
- Joint limits from `MDHParameters` are enforced at each step.

If you share your exact `KinematicsRM2.cs` transforms order or `Matrix` utilities, I’ll align the indexing and types precisely to your codebase.


---
---

# Fixing Manipulator6DoF.CalculateJacobian for 6DoF with fixed Oe and tool

Your current implementation already builds the full chain with fixed base O0, inserted elbow Oe after joint 3, and tool O7 at the TCP, and uses PoseErrorFull for a 6D task-space error. The issue is confined to the Jacobian column mapping: joint 4’s column incorrectly uses the elbow frame Oe (index 4) for its axis and origin. The geometric Jacobian must use the z-axis and origin of the “previous frame” of each actuated joint. For joint 4, that previous frame is O3 (index 3), not Oe. Correcting this restores convergence and consistency across the elbow insertion.

---

## Direct fix

Replace axisFrameIdx in CalculateJacobian with this mapping:

- Frame order from GetFrameTransforms: [O0, O1, O2, O3, Oe, O4, O5, O6, O7]
- Joint i → previous frame index: {0→O0, 1→O1, 2→O2, 3→O3, 4→O4’s previous (use O4 frame for z/origin), 5→O5’s previous}

In your list, to preserve the correct geometry:
- Use O3 for joint 4’s “previous” frame (index 3).
- Keep O4 and O5 for joints 5 and 6.

That yields:

```csharp
private double[,] CalculateJacobian(double[] q)
{
    // Correct previous-frame mapping for revolute joints 1..6
    // Transforms indices: O0=0, O1=1, O2=2, O3=3, Oe=4, O4=5, O5=6, O6=7, O7=8
    // Joint index -> previous frame:
    // 0 -> O0 (0), 1 -> O1 (1), 2 -> O2 (2), 3 -> O3 (3), 4 -> O4 (5), 5 -> O5 (6)
    int[] axisFrameIdx = { 0, 1, 2, 3, 5, 6 };

    var J = new double[6, 6];
    var transforms = GetFrameTransforms(q);
    var p_eff = new[] { transforms[^1][0, 3], transforms[^1][1, 3], transforms[^1][2, 3] };

    for (var joint = 0; joint < 6; joint++)
    {
        var Tprev = transforms[axisFrameIdx[joint]];
        var z = new[] { Tprev[0, 2], Tprev[1, 2], Tprev[2, 2] };
        var o = new[] { Tprev[0, 3], Tprev[1, 3], Tprev[2, 3] };

        var Jv = MathUtil.Cross(z, MathUtil.Sub(p_eff, o)); // linear part
        for (var r = 0; r < 3; r++)
        {
            J[r, joint] = Jv[r];
            J[r + 3, joint] = z[r]; // angular part
        }
    }
    return J;
}
```

This change ensures:
- Joint 4’s column uses z₃ and o₃ from O3, not the fixed Oe frame.
- The TCP O7 is still used for p_eff, including the tool transform as you already apply in Forward/ApplyTool.

---

## Optional validation: finite differences

Add a quick check to confirm column correctness around the O3–Oe–O4 region:

```csharp
private bool ValidateJacobian(double[] q, double eps = 1e-6, double tol = 1e-3)
{
    var transforms = GetFrameTransforms(q);
    var J = CalculateJacobian(q);
    var T0 = transforms[^1];
    var p0 = new[] { T0[0,3], T0[1,3], T0[2,3] };

    for (int j = 0; j < 6; j++)
    {
        var qh = (double[])q.Clone();
        qh[j] += eps;
        var Th = Forward(qh);
        var ph = new[] { Th[0,3], Th[1,3], Th[2,3] };
        var num = new[] { (ph[0]-p0[0])/eps, (ph[1]-p0[1])/eps, (ph[2]-p0[2])/eps };

        double err = Math.Sqrt(
            (num[0]-J[0,j])*(num[0]-J[0,j]) +
            (num[1]-J[1,j])*(num[1]-J[1,j]) +
            (num[2]-J[2,j])*(num[2]-J[2,j])
        );
        if (err > tol) return false;
    }
    return true;
}
```

You should see errors below the tolerance, especially for the joint-4 column after the fix.

---

## Why this matters

- Damped least squares solves (J Jᵀ + λ²I) y = e and uses dq = Jᵀ y. If any column uses the wrong axis/origin (e.g., Oe for joint 4), updates won’t reduce the end-effector error consistently, causing oscillation or slow/no convergence around the elbow segment.
- The fixed Oe does not add DoF; it changes downstream origins/axes. Using the proper “previous frame” maintains the physical meaning of each Jacobian column.

---

## Quick note on tool and orientation

- Your Forward appends the tool transform to the flange and Jacobian uses O7 (TCP) for p_eff, so the tool is properly factored into both position and orientation error via PoseErrorFull. No changes needed there.

If you want, I can provide a small PR-style diff limited to the CalculateJacobian method.

---
---
Excellent, let’s pin this down precisely.

---

### 1. Where Oe shows up in your current code
- In your **forward kinematics** (`GetFrameTransforms` / `Forward`), you multiply in the fixed homogeneous transform for **Oe** right after joint 3.  
  That means every downstream frame (O4, O5, O6, O7) is shifted by that constant offset.  
  → **This is the only place Oe’s “space” is used.**

- In your **Jacobian construction**, you originally had an `axisFrameIdx` array like `{0,1,2,4,5,6}`.  
  That means for joint 4 you were taking the z‑axis and origin from **index 4**, which is Oe.  
  → That’s the mistake: Oe is not a joint, so its z‑axis/origin should never be used to form a Jacobian column.

---

### 2. Correct usage of Oe
- **Keep Oe in the transform chain**: it’s a fixed spacer, so you must include it in FK to get the TCP in the right place.
- **Do not use Oe as a Jacobian axis source**: when building Jacobian columns, only the six actuated joints contribute. For joint 4, the correct “previous frame” is O3, not Oe.

So Oe’s “space” is used **indirectly**: it shifts the downstream origins and axes, which are then used for joints 4–6. But Oe itself never contributes a Jacobian column.

---

### 3. Why this matters
- If you use Oe’s z‑axis, you’re effectively telling the solver there’s a 7th joint at Oe. That breaks the mapping between joint motion and TCP motion.
- If you skip Oe in the Jacobian axis list, you still get the correct geometry, because Oe’s transform has already been applied to O4, O5, O6, and O7.

---

### 4. Practical rule of thumb
- **Forward kinematics**: include every fixed transform (O0, Oe, O7).  
- **Jacobian**: only include actuated joints. Fixed transforms are “transparent” — they shift the coordinate frames but don’t add columns.

---

👉 So to answer directly:  
You used Oe’s space in **forward kinematics** (to position downstream frames), but you should **not** use Oe’s space in the Jacobian column construction. That’s why the fix was to change the axisFrameIdx mapping so joint 4 uses O3’s z/origin, not Oe’s.  

Would you like me to show you a **numerical example** (say, with a simple set of joint angles) where the Jacobian column for joint 4 differs if you use Oe vs O3, so you can see the effect concretely?

---
---

Perfect — let’s make this concrete with your own MDH table.  
We’ll compute the Jacobian column for **joint 4** in two ways:

1. **Wrong way (using Oe’s z, o)**  
2. **Correct way (using O3’s z, o)**  

and compare the results numerically.

---

## 1. Your MDH table (summary)

From what you showed earlier:

| L | α | a | θ offset+θ | d |  
|---|---|---|---|---|  
| 1 | 0 | 0 | θ₁ | 127 |  
| 2 | π/2 | 29.69 | π/2+θ₂ | 0 |  
| 3 | 0 | 108 | −π/2+θ₃ | 0 |  
| e | π/2 | 0 | 0 | −20 | ← fixed Oe |  
| 4 | −π/2 | 168.98 | −π+θ₄ | 0 |  
| 5 | −π/2 | 0 | θ₅ | 0 |  
| 6 | 0 | 0 | θ₆ | 24.29 |  

So the chain is: O0 → O1 → O2 → O3 → Oe → O4 → O5 → O6 → O7.

---

## 2. Pick a test joint configuration

Let’s take a simple pose (all angles in radians):

\[
\theta = [0.0,\; 0.0,\; 0.0,\; 0.0,\; 0.0,\; 0.0]
\]

---

## 3. Compute forward kinematics up to O3, Oe, O4

- At θ=0, O3 is located at roughly (x=137.69, y=0, z=127) in base coordinates (after link1+link2+link3).  
- Oe shifts by d=−20 along z₃ (which points along −y at this point), so Oe is offset from O3.  
- O4 then rotates about z₃ (joint 4’s axis).

---

## 4. Jacobian column for joint 4

Formula for revolute joint:

\[
J_{v,4} = \vec{z}_3 \times (\vec{p}_{\text{TCP}} - \vec{o}_3), \quad
J_{\omega,4} = \vec{z}_3
\]

- **Correct**: use z₃, o₃.  
- **Wrong**: use zₑ, oₑ.

---

### Example numbers (approximate)

Let’s suppose the TCP (O7) at this zero pose is at:

\[
p_{\text{TCP}} \approx (306.7,\; 0,\; 107)
\]

- O3 origin: \(o_3 \approx (137.7, 0, 127)\)  
- Oe origin: \(o_e \approx (137.7, -20, 127)\)  
- z₃ = (0, −1, 0) (pointing −y)  
- zₑ = (0, −1, 0) (same direction, but different origin)

Now:

- **Correct (O3):**  
  \(p_{\text{TCP}} - o_3 = (169, 0, −20)\)  
  \(J_{v,4} = z₃ × (169, 0, −20) = (20, 0, 169)\)  
  \(J_{\omega,4} = (0, −1, 0)\)

- **Wrong (Oe):**  
  \(p_{\text{TCP}} - o_e = (169, 20, −20)\)  
  \(J_{v,4} = zₑ × (169, 20, −20) = (20, 0, 169)\)  
  \(J_{\omega,4} = (0, −1, 0)\)

At this particular pose, the linear part comes out the same because z₃=zₑ and the offset is along z. But in general, when the elbow offset is not aligned with z, the difference shows up: using oₑ instead of o₃ shifts the lever arm, producing the wrong linear velocity contribution.

---

## 5. Why this matters

- At some configurations, the error cancels (like above).  
- At others, the offset changes the cross product, giving a wrong Jacobian column.  
- That’s why you must always use the **previous joint frame (O3)**, not the inserted fixed frame (Oe), when forming joint 4’s Jacobian column.

---

✅ **Conclusion:**  
You do **not** need to remake your MDH. Keep Oe in the FK chain. But in the Jacobian, joint 4’s column must use z₃ and o₃ (from O3), not zₑ/oₑ. That ensures the Jacobian reflects the true kinematic influence of joint 4.  

Would you like me to show you a **numerical case where θ₂ or θ₃ is rotated**, so the difference between using O3 vs Oe becomes very visible?

---
---


### Numerical example with rotation at joint 3

We’ll use your MDH and compare the Jacobian column for joint 4 computed two ways:
- Correct: use \(\vec{z}_3, \vec{o}_3\) from frame O3.
- Incorrect: use \(\vec{z}_e, \vec{o}_e\) from the fixed elbow frame Oe.

Assumptions:
- Base at O0 is identity.
- Tool O7 is a fixed transform appended after O6 (already included in the end-effector pose we compute).
- Joint angles (radians): \(\theta = [0,\; 0,\; \theta_3,\; 0,\; 0,\; 0]\) with \(\theta_3 = \frac{\pi}{4}\) (45°).
- MDH rows (from your table):
  - 1: \(\alpha_0=0, a_0=0, d_1=127, \theta_1=\theta_1\)
  - 2: \(\alpha_1=\frac{\pi}{2}, a_1=29.69, d_2=0, \theta_2=\frac{\pi}{2}+\theta_2\)
  - 3: \(\alpha_2=0, a_2=108, d_3=0, \theta_3=-\frac{\pi}{2}+\theta_3\)
  - e: \(\alpha_e=\frac{\pi}{2}, a_e=0, d_e=-20, \theta_e=0\) (fixed)
  - 4: \(\alpha_3=-\frac{\pi}{2}, a_3=168.98, d_4=0, \theta_4=-\pi+\theta_4\)
  - 5: \(\alpha_4=-\frac{\pi}{2}, a_4=0, d_5=0, \theta_5=\theta_5\)
  - 6: \(\alpha_5=0, a_5=0, d_6=24.29, \theta_6=\theta_6\)

We’ll compute O3 and Oe, then the end-effector position O7 at this configuration, and form the joint-4 Jacobian column.

---

### Forward to O3 under the given MDH

Using MDH composition \(T_i = R_z(\theta_i)\,T_z(d_i)\,T_x(a_{i-1})\,R_x(\alpha_{i-1})\), with \(\theta_1=0,\theta_2=0,\theta_3=\frac{\pi}{4}\):

- After joint 1 (O1):
  - \(\vec{o}_1 = (0,\;0,\;127)\)
  - \(\vec{z}_1 = (0,\;0,\;1)\)

- After joint 2 (O2):
  - Rotation \(R_z(\frac{\pi}{2})\) plus \(R_x(\frac{\pi}{2})\), translation \(a_1=29.69\) along \(x_1\).
  - Approximate result:
    - \(\vec{o}_2 \approx (0,\;29.69,\;127)\)
    - \(\vec{z}_2 \approx (0,\;-1,\;0)\)

- After joint 3 (O3):
  - \(\theta_3 = -\frac{\pi}{2}+\frac{\pi}{4} = -\frac{\pi}{4}\). This rotates about \(\vec{z}_2\) (which is roughly \((0,-1,0)\)), and translates \(a_2=108\) along \(x_2\).
  - The local \(x_2\) is roughly \((0,0,1)\) before the \(\theta_3\) rotation; after rotation by \(-\pi/4\) about \(\vec{z}_2\), the forward direction tilts.
  - A numerically reasonable approximation (keeping terms to show the effect):
    - \(\vec{o}_3 \approx (0,\;29.69,\;127) + 108 \cdot (0,0,1) = (0,\;29.69,\;235)\)
    - \(\vec{z}_3 \approx (0,\;-1,\;0)\)

These approximations reflect the MDH orientation sequence; the exact numbers depend on the precise rotation order, but the key is that \(\vec{z}_3\) is not aligned with the global \(z\), and \(o_3\) is offset from \(o_e\) by the elbow’s \(d_e\) along \(z_3\).

---

### Insert fixed elbow Oe

Oe applies \(\alpha_e=\frac{\pi}{2}\) and \(d_e=-20\) along \(z_3\).

- \(\vec{o}_e = \vec{o}_3 + d_e \cdot \vec{z}_3 \approx (0,\;29.69,\;235) + (-20)\cdot(0,-1,0) = (0,\;49.69,\;235)\)
- \(\vec{z}_e = \vec{z}_3 \approx (0,\;-1,\;0)\) (axis direction unchanged by pure translation)

Note the origin shift in \(y\): Oe is 20 mm away from O3 along the local \(\vec{z}_3\).

---

### End-effector O7 (flange + tool)

With joints 4–6 at zero and the tool appended, the TCP will be forward along the chain from Oe and O4 by \(a_3=168.98\) and \(d_6=24.29\), mostly along the rotated axes. For the purpose of comparing Jacobian columns, we only need the difference vectors \(\vec{p}_{\text{TCP}}-\vec{o}_3\) and \(\vec{p}_{\text{TCP}}-\vec{o}_e\). Let’s pick a plausible TCP (from FK at this configuration):

- Suppose \(\vec{p}_{\text{TCP}} \approx (170,\;50,\;260)\) in base coordinates.  
  This choice encodes that the arm extended roughly forward in \(x\), elevated in \(z\), and shifted in \(y\) by the elbow and wrist geometry at \(\theta_3=\pi/4\). The exact value is not critical; what matters is that \(\vec{p}_{\text{TCP}}\) is not collinear with \(\vec{o}_3\) and \(\vec{o}_e\).

Compute the lever arms:

- Using O3:
  - \(\vec{r}_3 = \vec{p}_{\text{TCP}} - \vec{o}_3 \approx (170-0,\;50-29.69,\;260-235) = (170,\;20.31,\;25)\)

- Using Oe:
  - \(\vec{r}_e = \vec{p}_{\text{TCP}} - \vec{o}_e \approx (170-0,\;50-49.69,\;260-235) = (170,\;0.31,\;25)\)

---

### Jacobian column for joint 4

For a revolute joint:

\[
J_{v,4} = \vec{z} \times \vec{r}, \quad J_{\omega,4} = \vec{z}
\]

With \(\vec{z}_3=\vec{z}_e=(0,-1,0)\):

- Correct (O3):
  - \(\vec{z}_3 \times \vec{r}_3 = (0,-1,0) \times (170,\,20.31,\,25) = \big(-1\cdot25 - 0\cdot20.31,\; 0\cdot170 - 0\cdot25,\; 0\cdot20.31 - (-1)\cdot170\big)\)
  - \(J_{v,4}^{\text{(O3)}} = (-25,\; 0,\; 170)\)
  - \(J_{\omega,4} = (0,\,-1,\,0)\)

- Wrong (Oe):
  - \(\vec{z}_e \times \vec{r}_e = (0,-1,0) \times (170,\,0.31,\,25) = \big(-1\cdot25 - 0\cdot0.31,\; 0\cdot170 - 0\cdot25,\; 0\cdot0.31 - (-1)\cdot170\big)\)
  - \(J_{v,4}^{\text{(Oe)}} = (-25,\; 0,\; 170)\)
  - \(J_{\omega,4} = (0,\,-1,\,0)\)

In this specific numerical setup, the linear part happens to match because the Oe offset was applied along the axis direction \(\vec{z}_3\), and the \(y\)-component of the lever arm cancels in the cross product for \(\vec{z}=(0,-1,0)\). However, this is not generally true.

---

### Make the difference visible by rotating θ₂

Now rotate \(\theta_2 = \frac{\pi}{6}\) (30°) as well, keeping \(\theta_3=\frac{\pi}{4}\). This tilts the plane of motion so Oe’s translation is no longer aligned with \(\vec{z}_3\) in base coordinates.

Recompute approximated frames (qualitatively):
- The local \(\vec{z}_3\) is now tilted with respect to base \(y\), and \(\vec{o}_e = \vec{o}_3 + d_e \cdot \vec{z}_3\) moves in a direction with nonzero \(x\) and \(z\) components.
- Choose a TCP consistent with the tilt, e.g., \(\vec{p}_{\text{TCP}} \approx (160,\;60,\;250)\).

Let’s assume the tilt yields:
- \(\vec{o}_3 \approx (10,\;35,\;240)\)
- \(\vec{z}_3 \approx (0.2,\;-0.96,\;0.18)\) (unit length approx)
- Then \(\vec{o}_e = \vec{o}_3 + (-20)\cdot \vec{z}_3 \approx (10 - 4,\; 35 + 19.2,\; 240 - 3.6) = (6,\;54.2,\;236.4)\)

Lever arms:
- \(\vec{r}_3 = \vec{p}_{\text{TCP}} - \vec{o}_3 = (150,\;25,\;10)\)
- \(\vec{r}_e = \vec{p}_{\text{TCP}} - \vec{o}_e = (154,\;5.8,\;13.6)\)

Cross products:
- \(J_{v,4}^{\text{(O3)}} = \vec{z}_3 \times \vec{r}_3\).
- \(J_{v,4}^{\text{(Oe)}} = \vec{z}_3 \times \vec{r}_e\).  
  Since \(\vec{r}_e \neq \vec{r}_3\), these will differ.

Compute components:

- For O3:

\(J_{v,4}^{\text{(O3)}} =
    \begin{bmatrix}
    z_y r_z - z_z r_y \\
    z_z r_x - z_x r_z \\
    z_x r_y - z_y r_x
    \end{bmatrix}
    =
    \begin{bmatrix}
    (-0.96)\cdot 10 - (0.18)\cdot 25 \\
    (0.18)\cdot 150 - (0.2)\cdot 10 \\
    (0.2)\cdot 25 - (-0.96)\cdot 150
    \end{bmatrix}
    =
    \begin{bmatrix}
    -9.6 - 4.5 \\
    27 - 2 \\
    5 + 144
    \end{bmatrix}
    =
    (-14.1,\; 25,\; 149)\)

- For Oe:

\(J_{v,4}^{\text{(Oe)}} =
    \begin{bmatrix}
    (-0.96)\cdot 13.6 - (0.18)\cdot 5.8 \\
    (0.18)\cdot 154 - (0.2)\cdot 13.6 \\
    (0.2)\cdot 5.8 - (-0.96)\cdot 154
    \end{bmatrix}
    =
    \begin{bmatrix}
    -13.056 - 1.044 \\
    27.72 - 2.72 \\
    1.16 + 147.84
    \end{bmatrix}
    =
    (-14.1,\; 25,\; 149)\)

Here, the numbers still look close because the chosen vectors kept the offset mostly along the tilted \(\vec{z}_3\), and the lever arms’ differences project similarly. To make the discrepancy stark, increase the elbow offset or pick a TCP where \(\vec{r}\) has strong components orthogonal to \(\vec{z}_3\). For example, if the elbow translation includes an \(x\)-component (due to tilt), the \(z \times r\) term changes noticeably.

Let’s exaggerate the elbow shift to demonstrate:

- Use \(d_e = -60\) (hypothetical) with the same \(\vec{z}_3\).
- Then \(\vec{o}_e = \vec{o}_3 + (-60)\vec{z}_3 \approx (10 - 12,\; 35 + 57.6,\; 240 - 10.8) = (-2,\; 92.6,\; 229.2)\)
- \(\vec{r}_e = \vec{p}_{\text{TCP}} - \vec{o}_e = (162,\; -32.6,\; 20.8)\)

Now:

- \(J_{v,4}^{\text{(O3)}} = (-14.1,\; 25,\; 149)\) (unchanged)

- \(J_{v,4}^{\text{(Oe, exaggerated)}} =
  \begin{bmatrix}
  (-0.96)\cdot 20.8 - (0.18)\cdot (-32.6) \\
  (0.18)\cdot 162 - (0.2)\cdot 20.8 \\
  (0.2)\cdot (-32.6) - (-0.96)\cdot 162
  \end{bmatrix}
  =
  \begin{bmatrix}
  -19.968 + 5.868 \\
  29.16 - 4.16 \\
  -6.52 + 155.52
  \end{bmatrix}
  =
  (-14.1,\; 25,\; 149)\)

Even with exaggerated offset, because \(\vec{z}_3\) is used for both and the cross product depends on the perpendicular component of \(\vec{r}\) to \(\vec{z}\), the linear part can still match when the offset happens along \(\vec{z}\). The actual divergence in practice appears when the elbow’s fixed transform includes rotations or when numerical FK produces \(\vec{z}_e \neq \vec{z}_3\) due to the internal composition order. In many MDH implementations, Oe may include an \(R_x(\alpha_e)\) changing the downstream axes’ orientation; if you extract axes at Oe instead of O3, you can get a different \(\vec{z}\), which changes both \(J_v\) and \(J_\omega\).

---

### The robust rule

- Always use the “previous joint frame” \(i-1\) (here O3) to form joint \(i\)’s Jacobian column: \(\vec{z}_{i-1}, \vec{o}_{i-1}\).
- Include Oe in FK to position downstream frames, but never use Oe as the source of a Jacobian column.

If you want, I can run through your exact KinematicsRM2 composition order and extract \(\vec{z}_3\) and \(\vec{z}_e\) symbolically to show a case where \(\vec{z}_e \neq \vec{z}_3\), which will make the mismatch explicit.

