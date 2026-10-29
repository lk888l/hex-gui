import type { SmartKnobKind, SmartKnobProfile, SmartKnobTuning } from "./types";

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
