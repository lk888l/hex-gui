import type { SmartKnobKind, SmartKnobTuning } from "./types";

/** Preserve session-wide safety limits when restoring a new motor's mode. */
export function smartknobModeTuning(
  kind: SmartKnobKind,
  current: SmartKnobTuning,
  mode: SmartKnobTuning,
): SmartKnobTuning {
  return kind === "meow"
    ? { ...mode, effortLimit: current.effortLimit, maxOutputPermille: current.maxOutputPermille }
    : { ...mode };
}
