export const PP_POSITION_MIN_REV = -128;
export const PP_POSITION_MAX_REV_EXCLUSIVE = 128;

export type PpEndpoint = "A" | "B";

export interface EndpointAccuracyStats {
  samples: number;
  missed: number;
  last_encoder_rev: number | null;
  last_error_rev: number | null;
  mean_abs_error_rev: number | null;
  max_abs_error_rev: number | null;
}

export function emptyEndpointAccuracyStats(): EndpointAccuracyStats {
  return {
    samples: 0,
    missed: 0,
    last_encoder_rev: null,
    last_error_rev: null,
    mean_abs_error_rev: null,
    max_abs_error_rev: null,
  };
}

export function isRepresentablePpPosition(positionRev: number): boolean {
  return (
    Number.isFinite(positionRev) &&
    positionRev >= PP_POSITION_MIN_REV &&
    positionRev < PP_POSITION_MAX_REV_EXCLUSIVE
  );
}

export function positionErrorRev(
  encoderRev: number | null | undefined,
  targetRev: number | null | undefined,
): number | null {
  if (
    encoderRev == null ||
    targetRev == null ||
    !Number.isFinite(encoderRev) ||
    !Number.isFinite(targetRev)
  ) {
    return null;
  }
  return encoderRev - targetRev;
}

export function recordEndpointAccuracy(
  previous: EndpointAccuracyStats,
  encoderRev: number,
  targetRev: number,
): EndpointAccuracyStats {
  const errorRev = positionErrorRev(encoderRev, targetRev);
  if (errorRev == null) return { ...previous, missed: previous.missed + 1 };

  const absError = Math.abs(errorRev);
  const samples = previous.samples + 1;
  const previousMean = previous.mean_abs_error_rev ?? 0;
  return {
    ...previous,
    samples,
    last_encoder_rev: encoderRev,
    last_error_rev: errorRev,
    mean_abs_error_rev: previousMean + (absError - previousMean) / samples,
    max_abs_error_rev: Math.max(previous.max_abs_error_rev ?? 0, absError),
  };
}

export function recordMissedEndpointSample(
  previous: EndpointAccuracyStats,
): EndpointAccuracyStats {
  return { ...previous, missed: previous.missed + 1 };
}
