import type { El05ExperimentData, El05TelemetrySample } from "./el05Api";

/** Historical analysis must be recomputed after fixing the recorded feedback direction. */
export const LEGACY_VELOCITY_ANALYSIS_NOTE = "反馈速度方向已由原始 CAN 帧修正；旧历史曲线可能已抽样，指标按当前保留样本重新计算，旧分析已废弃。";

function feedbackDirection(sample: El05TelemetrySample): "wire" | "position" | null {
  if (!Number.isInteger(sample.can_id) || sample.can_id < 0 || !Number.isFinite(sample.velocity_rad_s)) return null;
  const frameType = (sample.can_id >>> 24) & 0x1f;
  if (frameType !== 2 && frameType !== 24) return null;
  if (!Array.isArray(sample.payload) || sample.payload.length !== 8
    || sample.payload.some(byte => !Number.isInteger(byte) || byte < 0 || byte > 255)) return null;
  const encoded = sample.payload[2] * 256 + sample.payload[3];
  const wireVelocity = encoded * 100 / 65535 - 50;
  if (Math.abs(sample.velocity_rad_s - wireVelocity) < 1e-8) return "wire";
  if (Math.abs(sample.velocity_rad_s + wireVelocity) < 1e-8) return "position";
  return null;
}

/**
 * Fix legacy feedback only when every sample has matching original type-2/type-24 evidence.
 * Commands, independently read mechVel, position derivatives, and wire bytes retain their values.
 */
export function normalizeExperimentFeedbackVelocity(data: El05ExperimentData): { data: El05ExperimentData; changed: boolean } {
  if (data.feedback_velocity_sign === -1 || !data.samples.length) return { data, changed: false };
  const calibration = data.status.static_friction;
  const eventSamples = calibration
    ? [calibration.last_stationary_sample, calibration.onset_sample, calibration.confirmation_sample].filter((sample): sample is El05TelemetrySample => sample != null)
    : [];
  const samples = [...data.samples, ...eventSamples];
  const directions = samples.map(feedbackDirection);
  if (directions.some(direction => direction === null)) return { data, changed: false };
  const corrected = (sample: El05TelemetrySample): El05TelemetrySample => feedbackDirection(sample) === "wire"
    ? { ...sample, velocity_rad_s: -sample.velocity_rad_s } : sample;
  return { data: { ...data, feedback_velocity_sign: -1,
    samples: data.samples.map(corrected),
    status: calibration ? { ...data.status, static_friction: { ...calibration,
      last_stationary_sample: calibration.last_stationary_sample ? corrected(calibration.last_stationary_sample) : null,
      onset_sample: calibration.onset_sample ? corrected(calibration.onset_sample) : null,
      confirmation_sample: calibration.confirmation_sample ? corrected(calibration.confirmation_sample) : null,
    } } : data.status,
  }, changed: true };
}
