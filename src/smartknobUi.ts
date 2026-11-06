import type { SmartKnobKind, SmartKnobProfile, SmartKnobTuning } from "./types";

/** Meow exposes one physical limit; legacy wire fields are derived, not edited. */
export function smartknobClampTuning(
  profile: Pick<SmartKnobProfile, "target" | "effortLimitMax">,
  tuning: SmartKnobTuning,
): SmartKnobTuning {
  if (profile.target.kind !== "meow") return { ...tuning };
  const peak = profile.effortLimitMax;
  const validPeak = Number.isFinite(peak) && peak > 0;
  const effortLimit = validPeak && Number.isFinite(tuning.effortLimit)
    ? Math.max(0, Math.min(tuning.effortLimit, peak)) : 0;
  return {
    ...tuning,
    effortLimit,
    maxOutputPermille: validPeak ? Math.floor(effortLimit / peak * 1000) : 0,
    frictionCompensation: effortLimit,
  };
}

/** Preserve session-wide safety limits when restoring a new motor's mode. */
export function smartknobModeTuning(
  kind: SmartKnobKind,
  current: SmartKnobTuning,
  mode: SmartKnobTuning,
): SmartKnobTuning {
  return kind === "meow" || kind === "damiao"
    ? { ...mode, effortLimit: current.effortLimit, maxOutputPermille: current.maxOutputPermille }
    : { ...mode };
}

/** Start within the selected motor's physical range and haptic defaults. */
export function smartknobInitialOutputLimits(
  profile: Pick<SmartKnobProfile, "target" | "effortUnit" | "effortLimitMax" | "maxOutputPermille">,
): Pick<SmartKnobTuning, "effortLimit" | "maxOutputPermille"> {
  const isDamiao = profile.target.kind === "damiao";
  return {
    effortLimit: Math.min(profile.effortLimitMax, isDamiao ? 0.12 : profile.effortUnit === "A" ? 0.45 : 2),
    maxOutputPermille: Math.min(profile.maxOutputPermille, isDamiao ? 400 : 700),
  };
}
