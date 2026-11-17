//! One latest-value channel per pull-cord run. The wheel loop reads it directly;
//! a delayed observer cannot queue old movement or attach to a replacement run.

use super::Phase;
use std::time::{Duration, Instant};
use tokio::sync::watch;

const MOTION_TIMEOUT: Duration = Duration::from_millis(150);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionPermit {
    Unavailable,
    Ready,
}

#[derive(Debug, Clone, Copy)]
pub struct CordMotion {
    pub permit: MotionPermit,
    pub at: Instant,
    /// Preserve rehome/failure even if readiness returns before a reader runs.
    pub invalidation_epoch: u64,
    pub length_m: Option<f64>,
    pub speed_m_s: Option<f64>,
    pub retract_positive: bool,
}

pub type CordMotionReceiver = watch::Receiver<CordMotion>;

/// Sole writer, owned by the pull-cord task. Dropping the task closes the link.
pub(crate) struct MotionPublisher(watch::Sender<CordMotion>);

impl MotionPublisher {
    pub fn new() -> (Self, CordMotionReceiver) {
        let (tx, rx) = watch::channel(CordMotion {
            permit: MotionPermit::Unavailable,
            at: Instant::now(),
            invalidation_epoch: 0,
            length_m: None,
            speed_m_s: None,
            retract_positive: true,
        });
        (Self(tx), rx)
    }

    pub fn publish(
        &self,
        phase: Phase,
        length_m: Option<f64>,
        speed_m_s: Option<f64>,
        retract_positive: bool,
    ) {
        let permit = if phase != Phase::Running {
            MotionPermit::Unavailable
        } else {
            MotionPermit::Ready
        };
        let previous = *self.0.borrow();
        self.0.send_replace(CordMotion {
            permit,
            at: Instant::now(),
            invalidation_epoch: previous.invalidation_epoch.wrapping_add(u64::from(
                permit == MotionPermit::Unavailable && previous.permit != MotionPermit::Unavailable,
            )),
            length_m,
            speed_m_s,
            retract_positive,
        });
    }

    pub fn unavailable(&self) {
        self.publish(Phase::Idle, None, None, true);
    }
}

/// Copy under the channel's short read lock; no guard crosses an await.
pub fn read_motion(rx: &CordMotionReceiver, now: Instant) -> Result<CordMotion, String> {
    if rx.has_changed().is_err() {
        return Err("pull-cord motion publisher stopped".into());
    }
    let motion = *rx.borrow();
    if now.saturating_duration_since(motion.at) > MOTION_TIMEOUT {
        return Err("pull-cord motion publisher expired".into());
    }
    if motion.permit == MotionPermit::Unavailable {
        return Err("pull-cord control stopped, homing or faulted".into());
    }
    if motion.length_m.is_none_or(|x| !x.is_finite())
        || motion.speed_m_s.is_none_or(|x| !x.is_finite())
    {
        return Err("invalid pull-cord motion feedback".into());
    }
    Ok(motion)
}
