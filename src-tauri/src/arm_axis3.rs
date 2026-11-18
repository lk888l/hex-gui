//! The supplied axis-3 assembly has ONE moving joint (Link1), followed by
//! fixed Link2/Link3. Fold all downstream masses in the moving joint frame.
//! G(q) = -axis · (R(axis,q) * mass_moment × gravity_in_joint_frame).
use anyhow::{anyhow, bail, ensure, Result};
use serde::{Deserialize, Serialize};
use std::{collections::HashSet, sync::OnceLock};

pub const URDF: &str = include_str!("../../public/axis3/assembly.urdf");
type V3 = [f64; 3];
type M3 = [[f64; 3]; 3];
const IDENTITY: M3 = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Axis3Calibration {
    pub joint_angle_rad: f64,
    pub encoder_direction: i8,
}
impl Axis3Calibration {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            matches!(self.encoder_direction, -1 | 1),
            "axis-3 encoder direction must be +1 or -1"
        );
        let m = model()?;
        ensure!(
            self.joint_angle_rad.is_finite()
                && self.joint_angle_rad >= m.lower
                && self.joint_angle_rad <= m.upper,
            "axis-3 reference angle must be within URDF limits [{:.4}, {:.4}] rad",
            m.lower,
            m.upper
        );
        Ok(())
    }
    pub fn joint_angle(&self, encoder: f64, reference: f64) -> f64 {
        self.joint_angle_rad + f64::from(self.encoder_direction) * (encoder - reference)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Axis3GravityConfig {
    /// Gravity expressed in the assembly's base_link frame, not the world.
    pub gravity_base_m_s2: V3,
}
impl Axis3GravityConfig {
    pub fn validate(&self) -> Result<()> {
        let g = self.gravity_base_m_s2;
        ensure!(
            g.iter().all(|v| v.is_finite()) && (dot(g, g).sqrt() - 9.80665).abs() < 0.02,
            "axis-3 base gravity must have magnitude 9.80665 m/s² (±0.02)"
        );
        model()?;
        Ok(())
    }
    pub fn coefficients(&self) -> Result<[f64; 2]> {
        let m = model()?;
        Ok([
            m.torque(std::f64::consts::FRAC_PI_2, self.gravity_base_m_s2),
            m.torque(0., self.gravity_base_m_s2),
        ])
    }
    pub fn peak_nm(&self) -> Result<f64> {
        let [a, b] = self.coefficients()?;
        Ok(a.hypot(b))
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Axis3ModelInfo {
    pub robot_name: String,
    pub joint_name: String,
    pub moving_links: Vec<String>,
    pub mass_kg: f64,
    pub com_joint_m: V3,
    pub axis_joint: V3,
    pub lower_rad: f64,
    pub upper_rad: f64,
    pub sin_nm: f64,
    pub cos_nm: f64,
    pub peak_nm: f64,
}

pub fn model_info(config: &Axis3GravityConfig) -> Result<Axis3ModelInfo> {
    config.validate()?;
    let m = model()?;
    let [sin_nm, cos_nm] = config.coefficients()?;
    Ok(Axis3ModelInfo {
        robot_name: m.robot_name.clone(),
        joint_name: m.joint_name.clone(),
        moving_links: m.moving_links.clone(),
        mass_kg: m.mass,
        com_joint_m: scale(m.moment, 1. / m.mass),
        axis_joint: m.axis,
        lower_rad: m.lower,
        upper_rad: m.upper,
        sin_nm,
        cos_nm,
        peak_nm: sin_nm.hypot(cos_nm),
    })
}

pub fn angle_in_limits(angle: f64) -> bool {
    model().is_ok_and(|m| angle.is_finite() && angle >= m.lower && angle <= m.upper)
}

struct Model {
    robot_name: String,
    joint_name: String,
    moving_links: Vec<String>,
    mass: f64,
    moment: V3,
    axis: V3,
    joint_rotation: M3,
    lower: f64,
    upper: f64,
}
fn model() -> Result<&'static Model> {
    static MODEL: OnceLock<Result<Model, String>> = OnceLock::new();
    MODEL
        .get_or_init(|| Model::parse(URDF).map_err(|e| e.to_string()))
        .as_ref()
        .map_err(|e| anyhow!("{e}"))
}
impl Model {
    fn parse(xml: &str) -> Result<Self> {
        let robot = urdf_rs::read_from_string(xml)?;
        let moving: Vec<_> = robot
            .joints
            .iter()
            .filter(|j| j.joint_type != urdf_rs::JointType::Fixed)
            .collect();
        ensure!(
            moving.len() == 1,
            "axis-3 test requires exactly one moving joint"
        );
        let j = moving[0];
        ensure!(
            j.joint_type == urdf_rs::JointType::Revolute && j.mimic.is_none(),
            "axis-3 test requires one independent revolute joint"
        );
        ensure!(
            !robot.joints.iter().any(|p| p.child.link == j.parent.link),
            "axis-3 moving joint must attach directly to the base"
        );
        ensure!(
            j.limit.lower.is_finite() && j.limit.upper.is_finite() && j.limit.lower < j.limit.upper,
            "invalid axis-3 joint limits"
        );
        let axis = j.axis.xyz.0;
        let norm = dot(axis, axis).sqrt();
        ensure!(norm.is_finite() && norm > 1e-9, "invalid axis-3 joint axis");
        let mut model = Self {
            robot_name: robot.name.clone(),
            joint_name: j.name.clone(),
            moving_links: vec![],
            mass: 0.,
            moment: [0.; 3],
            axis: scale(axis, 1. / norm),
            joint_rotation: rotation(j.origin.rpy.0),
            lower: j.limit.lower,
            upper: j.limit.upper,
        };
        ensure!(
            j.origin.rpy.iter().all(|v| v.is_finite()),
            "invalid axis-3 origin rotation"
        );
        let mut seen = HashSet::new();
        let mut pending = vec![(j.child.link.as_str(), IDENTITY, [0.; 3])];
        while let Some((name, rot, pos)) = pending.pop() {
            ensure!(seen.insert(name), "cycle or repeated child in axis-3 URDF");
            let link = robot
                .links
                .iter()
                .find(|l| l.name == name)
                .ok_or_else(|| anyhow!("missing link {name}"))?;
            let mass = link.inertial.mass.value;
            let com = link.inertial.origin.xyz.0;
            ensure!(
                mass.is_finite() && mass >= 0. && com.iter().all(|v| v.is_finite()),
                "invalid mass/COM on {name}"
            );
            model.mass += mass;
            model.moment = add(model.moment, scale(add(pos, mv(rot, com)), mass));
            model.moving_links.push(name.to_owned());
            for child in robot.joints.iter().filter(|c| c.parent.link == name) {
                if child.joint_type != urdf_rs::JointType::Fixed {
                    bail!("moving downstream joint {}", child.name);
                }
                ensure!(
                    child
                        .origin
                        .xyz
                        .iter()
                        .chain(child.origin.rpy.iter())
                        .all(|v| v.is_finite()),
                    "invalid fixed joint transform"
                );
                pending.push((
                    child.child.link.as_str(),
                    mm(rot, rotation(child.origin.rpy.0)),
                    add(pos, mv(rot, child.origin.xyz.0)),
                ));
            }
        }
        ensure!(
            model.mass > 0. && model.mass.is_finite() && model.moment.iter().all(|v| v.is_finite()),
            "axis-3 assembly needs positive finite mass"
        );
        ensure!(
            seen.len() + 1 == robot.links.len()
                && robot.links.iter().any(|l| l.name == j.parent.link),
            "disconnected axis-3 URDF links"
        );
        Ok(model)
    }
    fn torque(&self, q: f64, gravity_base: V3) -> f64 {
        let g = mv(transpose(self.joint_rotation), gravity_base);
        -dot(self.axis, cross(rotate(self.axis, q, self.moment), g))
    }
}
fn add(a: V3, b: V3) -> V3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn scale(a: V3, k: f64) -> V3 {
    a.map(|v| v * k)
}
fn dot(a: V3, b: V3) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn mv(m: M3, v: V3) -> V3 {
    m.map(|row| dot(row, v))
}
fn transpose(m: M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}
fn mm(a: M3, b: M3) -> M3 {
    a.map(|row| transpose(b).map(|col| dot(row, col)))
}
fn rotation([r, p, y]: V3) -> M3 {
    let (sr, cr) = r.sin_cos();
    let (sp, cp) = p.sin_cos();
    let (sy, cy) = y.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}
fn rotate(axis: V3, q: f64, v: V3) -> V3 {
    add(
        add(scale(v, q.cos()), scale(cross(axis, v), q.sin())),
        scale(axis, dot(axis, v) * (1. - q.cos())),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn assembly_includes_all_fixed_children_but_not_stationary_base() {
        let m = model().unwrap();
        assert_eq!(m.joint_name, "Link1");
        assert_eq!(m.moving_links, ["Link1", "Link2", "Link3"]);
        assert!(
            (m.mass - (0.214060387105929 + 0.152730310890769 + 0.592395418259202)).abs() < 1e-12
        );
        assert_eq!((m.lower, m.upper), (0., 3.14));
        // Independent URDF library folds the fixed subtree in the same child frame.
        let mut robot = urdf_rs::read_from_string(URDF).unwrap();
        robot.links.retain(|l| l.name != "base_link");
        robot.joints.retain(|j| j.name != "Link1");
        let xml = urdf_rs::write_to_string(&robot).unwrap();
        let (mass, com) = hex_arm_dynamics::lumped_inertial_from_urdf_string(&xml).unwrap();
        assert!((f64::from(mass) - m.mass).abs() < 1e-6);
        for i in 0..3 {
            assert!((f64::from(com[i]) - m.moment[i] / m.mass).abs() < 1e-6);
        }
    }
    #[test]
    fn torque_matches_potential_energy_gradient_for_tilted_mount_and_all_angles() {
        let m = model().unwrap();
        for gravity in [[0., 0., -9.80665], [9.80665, 0., 0.], [2., -5., -8.]] {
            let potential = |q| -dot(gravity, mv(m.joint_rotation, rotate(m.axis, q, m.moment)));
            for q in [0., 0.3, 1.2, 2., 3.14] {
                let derivative = (potential(q + 1e-6) - potential(q - 1e-6)) / 2e-6;
                assert!((derivative - m.torque(q, gravity)).abs() < 1e-8);
                let reconstructed = m.torque(std::f64::consts::FRAC_PI_2, gravity) * q.sin()
                    + m.torque(0., gravity) * q.cos();
                assert!((reconstructed - derivative).abs() < 1e-8);
            }
        }
    }
    #[test]
    fn rejects_invalid_calibration_gravity_and_unsupported_models() {
        for direction in [0, 2] {
            assert!(Axis3Calibration {
                joint_angle_rad: 0.,
                encoder_direction: direction
            }
            .validate()
            .is_err());
        }
        for q in [-0.01, 3.15, f64::NAN] {
            assert!(Axis3Calibration {
                joint_angle_rad: q,
                encoder_direction: 1
            }
            .validate()
            .is_err());
        }
        for g in [[0.; 3], [f64::NAN, 0., 0.], [0., 0., -100.]] {
            assert!(Axis3GravityConfig {
                gravity_base_m_s2: g
            }
            .validate()
            .is_err());
        }
        assert!(Model::parse(&URDF.replace("type=\"fixed\"", "type=\"continuous\"")).is_err());
        assert!(!angle_in_limits(f64::NAN));
    }
}
