import { useMemo, type MutableRefObject } from "react";
import ReactECharts from "echarts-for-react";
import { useI18n, type I18nKey } from "../i18n";

/** One polled snapshot of the cord, kept in a ref so the chart redraw is
 *  decoupled from the poll loop. */
export interface CordSample {
  /** ms since page load (`performance.now`). */
  t: number;
  length_m: number | null;
  torque_limit_raw: number | null;
  global_torque_limit_raw: number | null;
  feedback_raw: number | null;
  speed_m_s: number | null;
  target_speed_m_s: number | null;
}

// Three stacked sub-plots sharing a rolling time axis: one ECharts instance,
// each signal auto-scaled so different units stay readable side by side.
const SIGNALS: { key: keyof CordSample; nameKey: I18nKey; color: string; axis: number }[] = [
  { key: "length_m", nameKey: "chartLength", color: "#4f8cff", axis: 0 },
  { key: "torque_limit_raw", nameKey: "chartTorqueLimit", color: "#f39c12", axis: 1 },
  { key: "global_torque_limit_raw", nameKey: "chartGlobalTorqueLimit", color: "#06a7b2", axis: 1 },
  { key: "feedback_raw", nameKey: "chartCurrentFeedback", color: "#ab7bff", axis: 1 },
  { key: "speed_m_s", nameKey: "chartSpeed", color: "#2ecc71", axis: 2 },
  { key: "target_speed_m_s", nameKey: "chartTargetSpeed", color: "#e76f51", axis: 2 },
];

// Minimum y span. A cord held still has millimetre-scale noise; without a floor
// `scale: true` would zoom until that noise filled the plot and a steady hold
// looked like violent motion.
const AXES: { nameKey: I18nKey; minSpan: number }[] = [
  { nameKey: "chartLength", minSpan: 0.1 },
  { nameKey: "chartRaw", minSpan: 10 },
  { nameKey: "chartSpeed", minSpan: 0.2 },
];

export function PullCordChart({
  samples,
  chartVersion,
  windowSec,
}: {
  samples: MutableRefObject<CordSample[]>;
  chartVersion: number;
  windowSec: number;
}) {
  const { t } = useI18n();
  const option = useMemo(() => {
    const buffer = samples.current;
    const lastT = buffer.length > 0 ? buffer[buffer.length - 1].t : performance.now();
    const cutoff = lastT - windowSec * 1000;

    const ranges: ({ min: number; max: number } | null)[] = AXES.map(() => null);
    const series = SIGNALS.map(signal => {
      const data: [number, number | null][] = [];
      let low = Infinity;
      let high = -Infinity;
      for (const sample of buffer) {
        if (sample.t < cutoff) continue;
        const value = sample[signal.key] as number | null;
        if (value == null || !Number.isFinite(value)) {
          data.push([(sample.t - lastT) / 1000, null]);
          continue;
        }
        data.push([(sample.t - lastT) / 1000, value]); // seconds ago (≤ 0)
        if (value < low) low = value;
        if (value > high) high = value;
      }
      if (Number.isFinite(low) && Number.isFinite(high)) {
        const previous = ranges[signal.axis];
        ranges[signal.axis] = { min: Math.min(previous?.min ?? low, low), max: Math.max(previous?.max ?? high, high) };
      }
      return {
        name: t(signal.nameKey),
        type: "line" as const,
        xAxisIndex: signal.axis,
        yAxisIndex: signal.axis,
        data,
        showSymbol: false,
        lineStyle: {
          width: signal.key === "global_torque_limit_raw" ? 2.5 : 1.5,
          type: signal.key === "torque_limit_raw" ? "dashed" as const : "solid" as const,
          color: signal.color,
        },
        itemStyle: { color: signal.color },
        animation: false,
      };
    });

    const gridTop = 15;
    const gridHeight = 18;
    const gridGap = 10;
    return {
      backgroundColor: "transparent",
      animation: false,
      legend: { data: [t("chartGlobalTorqueLimit"), t("chartTorqueLimit"), t("chartCurrentFeedback"), t("chartSpeed"), t("chartTargetSpeed")], textStyle: { color: "#888" } },
      tooltip: { trigger: "axis" as const },
      axisPointer: { link: [{ xAxisIndex: "all" }] },
      grid: AXES.map((_, index) => ({
        left: 64,
        right: 16,
        top: `${gridTop + index * (gridHeight + gridGap)}%`,
        height: `${gridHeight}%`,
      })),
      xAxis: AXES.map((_, index) => ({
        type: "value" as const,
        gridIndex: index,
        min: -windowSec,
        max: 0,
        axisLabel: {
          show: index === AXES.length - 1,
          formatter: (value: number) => `${value.toFixed(0)}s`,
        },
        splitLine: { show: false },
      })),
      yAxis: AXES.map((axis, index) => {
        const range = ranges[index];
        const minSpan = axis.minSpan;
        const bounds =
          range && range.max - range.min < minSpan
            ? {
                min: round4(range.min - minSpan / 2),
                max: round4(range.max + minSpan / 2),
              }
            : { scale: true };
        return {
          type: "value" as const,
          gridIndex: index,
          name: t(axis.nameKey),
          nameLocation: "end" as const,
          nameTextStyle: { align: "left" as const, padding: [0, 0, 0, -56] },
          axisLabel: { formatter: (value: number) => formatTick(value) },
          splitLine: { lineStyle: { opacity: 0.12 } },
          ...bounds,
        };
      }),
      series,
    };
    // `chartVersion` is the redraw trigger: `samples` is a ref, so React cannot
    // see its mutations.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [chartVersion, windowSec, t]);

  return (
    <ReactECharts
      option={option}
      notMerge
      lazyUpdate
      style={{ height: 340, width: "100%" }}
      opts={{ renderer: "canvas" }}
    />
  );
}

const round4 = (value: number) => Math.round(value * 1e4) / 1e4;

function formatTick(value: number): string {
  if (Math.abs(value) >= 100) return value.toFixed(0);
  if (Math.abs(value) >= 10) return value.toFixed(1);
  return value.toFixed(2);
}
