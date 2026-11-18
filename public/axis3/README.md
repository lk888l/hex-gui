# Axis 3 test assembly

Source supplied by the operator: `smb://nas.local/home/tmp/总装URDF`.
Copied on 2026-11-17 without altering the URDF or STL files.
`assembly.urdf` is the source `urdf/总装URDF.urdf`, renamed only for its local URL.
URDF SHA-256: `7506d795221b4a493958016e9544423e55eae65ed821648b4fc24b6c54dbacfc`.

This is a **single movable joint assembly**, not a full three-axis robot:
`base_link → Link1 (revolute) → Link2 (fixed) → Link3 (fixed)`.
The application treats its sole movable joint `Link1` as the physical arm's third axis.
The CAN motor ID is selected separately; it is not inferred from the axis number.

The Rust backend embeds this URDF at build time, and the 3D viewer loads the same
file with package `总装URDF` mapped to `/axis3/`. No NAS connection is required at
runtime. Rebuild after replacing the model; do not replace only the web assets of
an installed binary, which would disagree with its embedded dynamics.

The moving mass is 0.9591861162559 kg, including all three downstream links and
excluding the fixed base. Its combined COM in the `Link1` joint frame is
`[0.0895058061489, -0.00296823407027, 0.0179109966815] m`. Fixed-joint rotations and
translations are applied before summing the masses' first moments. The joint axis
is normalized. Inertial origins locate each link's COM in its own link frame, as in
the [ROS URDF inertial specification](https://github.com/ros2/ros2_documentation/blob/rolling/source/Capabilities/Simulation/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.rst).

With base gravity `[0, 0, -9.80665] m/s²`, the holding torque is approximately
`G(q) = 0.027920404163 sin(q) + 0.841911506859 cos(q) Nm`.
Full-angle peak is 0.842374343359 Nm. URDF zero therefore requires nonzero torque;
it is **not** a vertically downward pendulum reference.

Joint limits are `[0, 3.14] rad`. The URDF's zero effort/velocity fields are CAD
export placeholders, not useful drive limits; the app uses explicit torque,
velocity and temperature settings. The model contains no extra payload. Its base
and the full arm's upstream joints must stay fixed during the single-axis test.
