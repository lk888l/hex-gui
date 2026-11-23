//! Planar endpoint feedback in the cord outlet frame. +X follows the calibrated
//! centre line outwards and +Y points to its left. Distances assume a taut cord.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use serde::{Deserialize, Serialize};

const TRACE_INTERVAL_US: u64 = 40_000;
const TRACE_SPACING_M: f64 = 0.001;
const TRACE_CAPACITY: usize = 1_500;
static NEXT_FRAME_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug, Clone, Copy, Serialize)]
pub struct CordEndpoint {
    /// Changes on startup, rehoming or a change of direction centre.
    pub frame_id: u64,
    /// Increases when feedback resumes after a gap in the same frame.
    pub segment: u64,
    /// Monotonic microseconds since this pull-cord run started.
    pub time_us: u64,
    /// Encoder centre used to map this sample into the outlet frame.
    pub center_deg: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub length_m: f64,
    /// Counterclockwise from +X.
    pub bearing_rad: f64,
    /// Positive for payout, negative for retraction.
    pub radial_speed_m_s: f64,
    /// [vx, vy] relative to the outlet, once two distinct direction frames exist.
    pub velocity_m_s: Option<[f64; 2]>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TrajectoryCursor {
    frame_id: u64,
    seq: u64,
}

#[derive(Serialize)]
pub struct TrajectoryUpdate {
    pub cursor: TrajectoryCursor,
    pub latest: Option<CordEndpoint>,
    pub retained_points: usize,
    pub points: Vec<CordEndpoint>,
}

struct Trace {
    cursor: TrajectoryCursor,
    latest: Option<(CordEndpoint, Instant)>,
    points: VecDeque<CordEndpoint>,
    cleared_at: Option<Instant>,
}

impl Trace {
    fn new() -> Self {
        Self {
            cursor: TrajectoryCursor {
                frame_id: NEXT_FRAME_ID.fetch_add(1, Ordering::Relaxed),
                seq: 0,
            },
            latest: None,
            points: VecDeque::with_capacity(TRACE_CAPACITY),
            cleared_at: None,
        }
    }

    fn latest(&self, now: Instant) -> Option<CordEndpoint> {
        self.latest
            .filter(|(_, until)| now <= *until)
            .map(|(p, _)| p)
    }

    fn push_point(&mut self, point: CordEndpoint) {
        if self.points.len() == TRACE_CAPACITY {
            self.points.pop_front();
        }
        self.points.push_back(point);
        self.cursor.seq += 1;
    }
}

/// Endpoint feedback and plot history from one motor run.
#[derive(Clone)]
pub struct CordTrajectory {
    trace: Arc<Mutex<Trace>>,
    stop: Arc<AtomicBool>,
    rehome: Arc<AtomicBool>,
}

impl CordTrajectory {
    pub fn latest(&self) -> Option<CordEndpoint> {
        self.live_sample(&self.trace.lock().unwrap())
    }

    /// Clear the displayed frame's plot and return the cursor for its next samples.
    pub fn clear_history(&self, displayed: TrajectoryCursor) -> Option<TrajectoryCursor> {
        let mut trace = self.trace.lock().unwrap();
        if displayed.frame_id != trace.cursor.frame_id {
            return None;
        }
        trace.cleared_at = Some(Instant::now());
        trace.points.clear();
        // A stopped recorder also needs to notify incremental readers of the clear.
        trace.cursor.seq += 1;
        Some(trace.cursor)
    }

    fn live_sample(&self, trace: &Trace) -> Option<CordEndpoint> {
        if self.stop.load(Ordering::Acquire) || self.rehome.load(Ordering::Acquire) {
            return None;
        }
        trace.latest(Instant::now())
    }

    /// Returns new plot samples, or the retained window on the first read or
    /// after a frame change. The live sample is available at the motor loop rate.
    pub fn since(&self, after: Option<TrajectoryCursor>) -> TrajectoryUpdate {
        let trace = self.trace.lock().unwrap();
        let len = trace.points.len();
        let oldest = trace.cursor.seq - len as u64;
        let skip = after
            .filter(|c| c.frame_id == trace.cursor.frame_id && c.seq <= trace.cursor.seq)
            .map_or(0, |c| c.seq.saturating_sub(oldest) as usize);
        TrajectoryUpdate {
            cursor: trace.cursor,
            latest: self.live_sample(&trace),
            retained_points: len,
            points: trace.points.iter().skip(skip).copied().collect(),
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct DirectionSample {
    pub offset_deg: f64,
    pub center_deg: f64,
    pub at: Instant,
    pub valid_until: Instant,
}

struct PreviousDirection {
    sample: DirectionSample,
    omega_rad_s: Option<f64>,
    valid_until: Instant,
}

pub(crate) struct TrajectoryRecorder {
    reader: CordTrajectory,
    started: Instant,
    center_deg: Option<f64>,
    segment: u64,
    previous: Option<PreviousDirection>,
}

impl TrajectoryRecorder {
    pub fn new(
        started: Instant,
        stop: Arc<AtomicBool>,
        rehome: Arc<AtomicBool>,
    ) -> (Self, CordTrajectory) {
        let reader = CordTrajectory {
            trace: Arc::new(Mutex::new(Trace::new())),
            stop,
            rehome,
        };
        let recorder = Self {
            reader: reader.clone(),
            started,
            center_deg: None,
            segment: 0,
            previous: None,
        };
        (recorder, reader)
    }

    pub fn reset_frame(&mut self) {
        self.previous = None;
        self.center_deg = None;
        self.segment = 0;
        *self.reader.trace.lock().unwrap() = Trace::new();
    }

    pub fn gap(&mut self) {
        if self.previous.take().is_some() {
            let mut trace = self.reader.trace.lock().unwrap();
            if let Some((last, _)) = trace.latest.take() {
                if trace
                    .points
                    .back()
                    .is_some_and(|p| p.time_us != last.time_us)
                {
                    trace.push_point(last);
                }
            }
        }
    }

    pub fn record(
        &mut self,
        now: Instant,
        length_m: Option<f64>,
        length_at: Instant,
        radial_speed_m_s: f64,
        motor_valid_until: Instant,
        direction: Option<DirectionSample>,
    ) {
        let (Some(length_m), Some(direction)) = (length_m, direction) else {
            self.gap();
            return;
        };
        let valid_until = motor_valid_until.min(direction.valid_until);
        if now > valid_until || !length_m.is_finite() || !radial_speed_m_s.is_finite() {
            self.gap();
            return;
        }
        if self
            .center_deg
            .is_some_and(|old| old != direction.center_deg)
        {
            self.reset_frame();
        }
        self.center_deg = Some(direction.center_deg);

        let previous = self
            .previous
            .as_ref()
            .filter(|p| now <= p.valid_until && direction.at >= p.sample.at);
        let omega = match previous {
            Some(p) if direction.at == p.sample.at => p.omega_rad_s,
            Some(p) => {
                let dt = direction.at.duration_since(p.sample.at).as_secs_f64();
                Some(
                    -crate::signed_degrees(direction.offset_deg - p.sample.offset_deg).to_radians()
                        / dt,
                )
            }
            None => {
                self.gap();
                self.segment += 1;
                None
            }
        };
        self.previous = Some(PreviousDirection {
            sample: direction,
            omega_rad_s: omega,
            valid_until,
        });

        // Anchor every estimate to the homed multi-turn measurement. Speed
        // supplies only the distance since that measurement, bounding drift.
        let length_m = (length_m
            + radial_speed_m_s * now.saturating_duration_since(length_at).as_secs_f64())
        .max(0.0);
        let bearing_rad = crate::signed_degrees(-direction.offset_deg).to_radians();
        let (sin, cos) = bearing_rad.sin_cos();
        let mut trace = self.reader.trace.lock().unwrap();
        let point = CordEndpoint {
            frame_id: trace.cursor.frame_id,
            segment: self.segment,
            time_us: now.duration_since(self.started).as_micros() as u64,
            center_deg: direction.center_deg,
            x_m: length_m * cos,
            y_m: length_m * sin,
            length_m,
            bearing_rad,
            radial_speed_m_s,
            velocity_m_s: omega.map(|w| {
                [
                    radial_speed_m_s * cos - length_m * w * sin,
                    radial_speed_m_s * sin + length_m * w * cos,
                ]
            }),
        };
        trace.latest = Some((point, valid_until));
        // A CAN transaction can finish publishing an earlier sample after a clear.
        if trace.cleared_at.is_some_and(|cutoff| now < cutoff) {
            return;
        }
        if trace.points.back().is_none_or(|p| {
            p.segment != point.segment
                || (point.time_us - p.time_us >= TRACE_INTERVAL_US
                    && (point.x_m - p.x_m).hypot(point.y_m - p.y_m) >= TRACE_SPACING_M)
        }) {
            trace.push_point(point);
        }
    }
}

impl Drop for TrajectoryRecorder {
    fn drop(&mut self) {
        self.gap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    const TTL: Duration = Duration::from_millis(150);

    fn new_trace(t: Instant) -> (TrajectoryRecorder, CordTrajectory) {
        TrajectoryRecorder::new(t, Default::default(), Default::default())
    }

    fn direction(at: Instant, offset_deg: f64) -> DirectionSample {
        DirectionSample {
            at,
            offset_deg,
            center_deg: 275.0,
            valid_until: at + TTL,
        }
    }

    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
    }

    #[test]
    fn measured_speed_bridges_drum_reads_and_home_bounds_retraction() {
        let start = Instant::now();
        let (mut recorder, reader) = new_trace(start);
        recorder.record(
            start,
            None,
            start,
            0.0,
            start + TTL,
            Some(direction(start, 0.0)),
        );
        assert!(reader.latest().is_none());
        recorder.record(
            start,
            Some(1.0),
            start,
            0.0,
            start + TTL,
            Some(direction(start, 0.0)),
        );
        assert!(reader.latest().unwrap().velocity_m_s.is_none());
        let t = start + Duration::from_millis(20);
        recorder.record(t, Some(1.0), start, 0.5, t + TTL, Some(direction(t, 0.0)));
        let p = reader.latest().unwrap();
        near(p.x_m, 1.01);
        near(p.y_m, 0.0);
        assert_eq!(p.velocity_m_s, Some([0.5, 0.0]));
        let t = t + Duration::from_millis(20);
        recorder.record(t, Some(0.8), t, -1.0, t + TTL, Some(direction(t, 0.0)));
        let p = reader.latest().unwrap();
        near(p.x_m, 0.8);
        assert_eq!(p.velocity_m_s, Some([-1.0, 0.0]));
        recorder.record(t, Some(-0.002), t, 0.0, t + TTL, Some(direction(t, 0.0)));
        near(reader.latest().unwrap().length_m, 0.0);
    }

    #[test]
    fn fixed_radius_sweep_uses_wrapped_angle_and_distinct_encoder_frame_times() {
        let t = Instant::now();
        let (mut recorder, reader) = new_trace(t);
        let t1 = t + Duration::from_millis(40);
        recorder.record(t, Some(2.0), t, 0.0, t + TTL, Some(direction(t, 179.0)));
        // A clockwise 2-degree turn crossing the signed-angle boundary, with
        // the second encoder frame arriving 10 ms before the motor loop reads it.
        let d = direction(t + Duration::from_millis(30), -179.0);
        recorder.record(t1, Some(2.0), t1, 0.0, t1 + TTL, Some(d));
        let p = reader.latest().unwrap();
        let angle = 179.0_f64.to_radians();
        near(p.x_m, 2.0 * angle.cos());
        near(p.y_m, 2.0 * angle.sin());
        let tangent = 2.0 * (-2.0_f64).to_radians() / 0.030;
        let [vx, vy] = p.velocity_m_s.unwrap();
        near(vx, -tangent * angle.sin());
        near(vy, tangent * angle.cos());
        recorder.record(
            t1 + Duration::from_millis(5),
            Some(2.0),
            t1,
            0.0,
            t1 + TTL,
            Some(d),
        );
        assert_eq!(reader.latest().unwrap().velocity_m_s, p.velocity_m_s);
        assert_eq!(reader.since(None).points.len(), 2);
    }

    #[test]
    fn gaps_expiry_and_frame_changes_never_bridge_history_or_velocity() {
        let t = Instant::now();
        let (mut recorder, reader) = new_trace(t);
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(direction(t, 90.0)));
        let first = reader.since(None);
        near(first.latest.unwrap().y_m, -1.0);
        assert!(reader
            .trace
            .lock()
            .unwrap()
            .latest(t + TTL + Duration::from_micros(1))
            .is_none());
        assert!(reader.since(Some(first.cursor)).points.is_empty());
        let t = t + Duration::from_millis(10);
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, None);
        assert!(reader.latest().is_none());
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(direction(t, -90.0)));
        let recovered = reader.since(Some(first.cursor));
        assert_eq!(recovered.points.len(), 1);
        let p = recovered.latest.unwrap();
        assert_ne!(p.segment, first.latest.unwrap().segment);
        assert!(p.velocity_m_s.is_none());
        near(p.y_m, 1.0);

        // Expiry also starts a segment if no consumer observed the outage.
        let t = t + TTL + Duration::from_millis(1);
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(direction(t, 0.0)));
        assert_ne!(reader.latest().unwrap().segment, p.segment);
        let d = DirectionSample {
            center_deg: 30.0,
            ..direction(t, 0.0)
        };
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(d));
        let recentered = reader.since(Some(recovered.cursor));
        assert_ne!(recentered.cursor.frame_id, first.cursor.frame_id);
        assert_eq!(recentered.retained_points, 1);
        near(recentered.latest.unwrap().center_deg, 30.0);
        assert!(recentered.latest.unwrap().velocity_m_s.is_none());
        recorder.reset_frame();
        assert!(reader.latest().is_none());
        assert!(reader.since(None).points.is_empty());
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(d));
        drop(recorder);
        assert!(reader.latest().is_none());
        assert_eq!(reader.since(None).points.len(), 1);
    }

    #[test]
    fn clear_preserves_live_feedback_and_resumes_plotting_only_new_samples() {
        let start = Instant::now() - Duration::from_millis(50);
        let (mut recorder, reader) = new_trace(start);
        recorder.record(
            start,
            Some(1.0),
            start,
            0.0,
            start + TTL,
            Some(direction(start, 0.0)),
        );
        let t = start + Duration::from_millis(40);
        recorder.record(t, Some(1.1), t, 0.5, t + TTL, Some(direction(t, 5.0)));
        let old = reader.since(None);
        assert_eq!(old.points.len(), 2);
        let before = old.latest.unwrap();
        let cleared = reader.clear_history(old.cursor).unwrap();
        let empty = reader.since(Some(old.cursor));
        assert_eq!(empty.retained_points, 0);
        assert!(empty.points.is_empty());
        assert_eq!(cleared.frame_id, before.frame_id);
        assert!(cleared.seq > old.cursor.seq);
        let live = empty.latest.unwrap();
        assert_eq!(live.time_us, before.time_us);
        assert_eq!(live.velocity_m_s, before.velocity_m_s);

        let cutoff = reader.trace.lock().unwrap().cleared_at.unwrap();
        let late = cutoff - Duration::from_micros(1);
        recorder.record(
            late,
            Some(1.2),
            late,
            0.5,
            late + TTL,
            Some(direction(late, 6.0)),
        );
        assert!(reader.since(None).points.is_empty());
        let next = cutoff + Duration::from_millis(5);
        recorder.record(
            next,
            Some(1.3),
            next,
            0.5,
            next + TTL,
            Some(direction(next, 7.0)),
        );
        let resumed = reader.since(Some(cleared));
        assert_eq!(resumed.points.len(), 1);
        assert_eq!(resumed.points[0].segment, before.segment);
        assert!(resumed.points[0].velocity_m_s.is_some());
        near(resumed.points[0].length_m, 1.3);
        assert_eq!(reader.since(Some(old.cursor)).points.len(), 1);

        reader.clear_history(resumed.cursor).unwrap();
        recorder.gap();
        assert!(reader.since(None).points.is_empty());
    }

    #[test]
    fn clear_targets_the_displayed_frame_and_works_after_stop() {
        let t = Instant::now();
        let (mut recorder, reader) = new_trace(t);
        let old = reader.since(None).cursor;
        recorder.reset_frame();
        recorder.record(t, Some(1.0), t, 0.0, t + TTL, Some(direction(t, 0.0)));
        drop(recorder);
        assert!(reader.clear_history(old).is_none());
        let retained = reader.since(None);
        assert_eq!(retained.points.len(), 1);
        assert!(retained.latest.is_none());
        reader.clear_history(retained.cursor).unwrap();
        assert!(reader.since(None).points.is_empty());
        assert_eq!(reader.since(Some(retained.cursor)).retained_points, 0);
    }

    #[test]
    fn a_slow_plot_reader_can_resume_from_the_bounded_window() {
        let start = Instant::now();
        let (mut recorder, reader) = new_trace(start);
        let initial = reader.since(None).cursor;
        for i in 0..=TRACE_CAPACITY {
            let t = start + Duration::from_micros(i as u64 * TRACE_INTERVAL_US);
            let length = 1.0 + i as f64 * 0.002;
            recorder.record(t, Some(length), t, 0.05, t + TTL, Some(direction(t, 0.0)));
        }
        let update = reader.since(Some(initial));
        assert_eq!(update.points.len(), TRACE_CAPACITY);
        assert_eq!(update.points[0].time_us, TRACE_INTERVAL_US);
        let end = update.points.last().unwrap();
        let held = start + Duration::from_micros(end.time_us + TRACE_INTERVAL_US);
        recorder.record(
            held,
            Some(end.length_m),
            held,
            0.0,
            held + TTL,
            Some(direction(held, 0.0)),
        );
        assert!(reader.since(Some(update.cursor)).points.is_empty());
        let final_at = held + Duration::from_millis(5);
        recorder.record(
            final_at,
            Some(end.length_m + 0.0005),
            final_at,
            0.0,
            final_at + TTL,
            Some(direction(final_at, 0.0)),
        );
        recorder.gap();
        let tail = reader.since(Some(update.cursor));
        assert!(tail.latest.is_none());
        assert_eq!(tail.points.len(), 1);
        near(tail.points[0].length_m, end.length_m + 0.0005);
    }
}
