import assert from "node:assert/strict";
import test from "node:test";

import {
  emptyEndpointAccuracyStats,
  isRepresentablePpPosition,
  positionErrorRev,
  recordEndpointAccuracy,
  recordMissedEndpointSample,
} from "../src/ppAccuracy.ts";

test("PP endpoints follow the signed Q8.24 range", () => {
  assert.equal(isRepresentablePpPosition(-128), true);
  assert.equal(isRepresentablePpPosition(127.999_999), true);
  assert.equal(isRepresentablePpPosition(128), false);
  assert.equal(isRepresentablePpPosition(-128.000_001), false);
  assert.equal(isRepresentablePpPosition(Number.NaN), false);
});

test("encoder error is signed encoder minus target", () => {
  assert.equal(positionErrorRev(0.101, 0.1), 0.0010000000000000009);
  assert.equal(positionErrorRev(null, 0.1), null);
  assert.equal(positionErrorRev(0.1, Number.POSITIVE_INFINITY), null);
});

test("endpoint accuracy keeps last, mean absolute and worst error", () => {
  let stats = emptyEndpointAccuracyStats();
  stats = recordEndpointAccuracy(stats, 1.01, 1);
  stats = recordEndpointAccuracy(stats, 0.98, 1);

  assert.equal(stats.samples, 2);
  assert.equal(stats.last_encoder_rev, 0.98);
  assert.equal(stats.last_error_rev, -0.020000000000000018);
  assert.ok(Math.abs((stats.mean_abs_error_rev ?? 0) - 0.015) < 1e-12);
  assert.ok(Math.abs((stats.max_abs_error_rev ?? 0) - 0.02) < 1e-12);

  stats = recordMissedEndpointSample(stats);
  assert.equal(stats.missed, 1);
  assert.equal(stats.samples, 2);
});
