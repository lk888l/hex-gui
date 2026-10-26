import type { El05ExperimentData, El05TelemetrySample } from "./el05Api";

/** Legacy logs predate standard MIT support and retain their 16-bit scale. */
export function feedbackVelocityBits(data: Pick<El05ExperimentData, "protocol" | "feedback_velocity_bits">): 12 | 16 {
  return data.feedback_velocity_bits ?? (data.protocol === "StandardMit" ? 12 : 16);
}

export function feedbackVelocityResolution(data: Pick<El05ExperimentData, "protocol" | "feedback_velocity_bits">): number {
  return 100 / (2 ** feedbackVelocityBits(data) - 1);
}

/** Standard MIT clones may hold raw mode bits at zero even while moving. */
export function controlFeedbackEligible(sample: El05TelemetrySample, data: Pick<El05ExperimentData, "protocol">): boolean {
  if (data.protocol !== "StandardMit") return sample.status_code === 2;
  return sample.fault_bits === 0 && sample.target?.kind === "Mit"
    && sample.target_sequence > 0 && sample.target_age_ms != null
    && Number.isFinite(sample.target_age_ms) && sample.target_age_ms >= 0 && sample.target_age_ms <= 250;
}
