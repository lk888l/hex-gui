import type { El05ExperimentData, El05StaticFrictionPass, El05TelemetrySample } from "./el05Api";

/** Historical analysis must be recomputed after fixing the recorded feedback direction. */
export const LEGACY_VELOCITY_ANALYSIS_NOTE = "反馈速度方向已统一；旧历史曲线可能已抽样，指标按当前保留样本重新计算，旧分析已废弃。";

function feedbackDirection(sample: El05TelemetrySample, data: El05ExperimentData): "wire" | "inverted" | null {
  if (!Number.isInteger(sample.can_id) || sample.can_id < 0 || !Number.isFinite(sample.velocity_rad_s)) return null;
  if (!Array.isArray(sample.payload) || sample.payload.length !== 8
    || sample.payload.some(byte => !Number.isInteger(byte) || byte < 0 || byte > 255)) return null;
  const standardMit = data.protocol === "StandardMit";
  if (standardMit) {
    if (sample.can_id > 0x7ff || (data.motor_id != null && sample.payload[0] !== data.motor_id)) return null;
  } else {
    const frameType = (sample.can_id >>> 24) & 0x1f;
    if (frameType !== 2 && frameType !== 24) return null;
  }
  const encoded = standardMit ? sample.payload[3] * 16 + (sample.payload[4] >> 4) : sample.payload[2] * 256 + sample.payload[3];
  const wireVelocity = encoded * 100 / (standardMit ? 4095 : 65535) - 50;
  if (Math.abs(sample.velocity_rad_s - wireVelocity) < 1e-8) return "wire";
  if (Math.abs(sample.velocity_rad_s + wireVelocity) < 1e-8) return "inverted";
  return null;
}

/**
 * Restore direct (+1) feedback using an explicit old multiplier, or matching
 * original type-2/type-24 evidence for every sample in an unmarked recording.
 * Commands, independently read mechVel, position derivatives, and wire bytes retain their values.
 */
export function normalizeExperimentFeedbackVelocity(data: El05ExperimentData): { data: El05ExperimentData; changed: boolean } {
  if (data.feedback_velocity_sign === 1 || !data.samples.length) return { data, changed: false };
  const calibration = data.status.static_friction;
  const eventSamples = calibration
    ? [...(calibration.completed_directions ?? []), calibration].flatMap(pass => [pass.last_stationary_sample, pass.onset_sample, pass.confirmation_sample]).filter((sample): sample is El05TelemetrySample => sample != null)
    : [];
  const samples = [...data.samples, ...eventSamples];
  const invertedRecording = data.feedback_velocity_sign === -1;
  if (samples.some(sample => !Number.isFinite(sample.velocity_rad_s))
    || (!invertedRecording && samples.some(sample => feedbackDirection(sample, data) === null))) return { data, changed: false };
  const corrected = (sample: El05TelemetrySample): El05TelemetrySample => invertedRecording || feedbackDirection(sample, data) === "inverted"
    ? { ...sample, velocity_rad_s: -sample.velocity_rad_s } : sample;
  const correctedPass = (pass: El05StaticFrictionPass): El05StaticFrictionPass => ({ ...pass,
    last_stationary_sample: pass.last_stationary_sample ? corrected(pass.last_stationary_sample) : null,
    onset_sample: pass.onset_sample ? corrected(pass.onset_sample) : null,
    confirmation_sample: pass.confirmation_sample ? corrected(pass.confirmation_sample) : null,
  });
  return { data: { ...data, feedback_velocity_sign: 1,
    samples: data.samples.map(corrected),
    status: calibration ? { ...data.status, static_friction: { ...correctedPass(calibration),
      ...(calibration.completed_directions ? { completed_directions: calibration.completed_directions.map(correctedPass) } : {}),
    } } : data.status,
  }, changed: true };
}
