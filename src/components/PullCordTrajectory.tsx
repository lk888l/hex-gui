import { useMemo } from "react";
import { Button, Card, Descriptions, Space, Tag, Typography } from "antd";
import ReactECharts from "echarts-for-react";
import { useI18n } from "../i18n";
import type { CordEndpoint, Phase } from "../pullCordTypes";

export function PullCordTrajectory({ points, latest, phase, onClear, clearing }: {
  points: CordEndpoint[];
  latest: CordEndpoint | null;
  phase: Phase;
  onClear: () => void;
  clearing: boolean;
}) {
  const { lang } = useI18n();
  const tr = (zh: string, en: string) => lang === "zh" ? zh : en;
  const option = useMemo(() => {
    const path: ([number, number] | null)[] = [];
    let previous: CordEndpoint | undefined;
    let maxCoord = 0;
    const append = (p: CordEndpoint) => {
      if (previous && (p.frame_id !== previous.frame_id || p.segment !== previous.segment)) path.push(null);
      path.push([p.x_m, p.y_m]);
      maxCoord = Math.max(maxCoord, Math.abs(p.x_m), Math.abs(p.y_m));
      previous = p;
    };
    for (const p of points) append(p);
    if (latest && latest.time_us !== previous?.time_us) append(latest);
    const extent = Math.max(0.25, Math.ceil(maxCoord * 1.05 * 4) / 4);
    const axis = {
      type: "value", min: -extent, max: extent, interval: extent / 2,
      axisLine: { show: true, onZero: false },
      axisLabel: { color: "#aaa" },
      splitLine: { lineStyle: { color: "rgba(135,150,165,.18)" } },
      nameLocation: "middle", nameTextStyle: { color: "#bbb" },
    };
    return {
      animation: false,
      // Equal total margins on a square canvas preserve metres on both axes.
      grid: { left: 56, right: 16, top: 20, bottom: 52 },
      xAxis: { ...axis, name: "X (m)", nameGap: 27 },
      yAxis: { ...axis, name: "Y (m)", nameGap: 38 },
      series: [
        { type: "line", data: path, connectNulls: false, symbol: "circle", symbolSize: 2,
          itemStyle: { color: "#62c6c4" }, lineStyle: { color: "#62c6c4", width: 2 } },
        { type: "line", data: latest ? [[0, 0], [latest.x_m, latest.y_m]] : [], showSymbol: false,
          lineStyle: { color: "#a7b2bd", type: "dashed", width: 1 } },
        { type: "scatter", data: [[0, 0]], symbol: "rect", symbolSize: 7, itemStyle: { color: "#bbc4cc" } },
        { type: "scatter", data: latest ? [[latest.x_m, latest.y_m]] : [], symbolSize: 11,
          itemStyle: { color: "#ff895e", opacity: 1 } },
      ],
    };
  }, [points, latest]);

  const state = latest ? tr("实时测绘", "Tracking")
    : phase === "homing" ? tr("等待归零", "Homing")
    : phase === "running" ? tr("等待有效反馈", "Waiting for feedback")
    : points.length ? tr("已保留轨迹", "Recorded trace") : tr("等待启动拉线", "Waiting for cord startup");
  const pair = (values: [number, number] | null, unit: string) =>
    values ? `(${values[0].toFixed(3)}, ${values[1].toFixed(3)}) ${unit}` : "—";
  return <Card size="small" title={tr("拉线末端轨迹", "Cord endpoint trajectory")}
    extra={<Space size={8} wrap>
      <Tag color={latest ? "green" : "default"}>{state}</Tag>
      <Button size="small" onClick={onClear} loading={clearing} disabled={!points.length || clearing}>
        {tr("清除轨迹", "Clear trail")}
      </Button>
    </Space>}>
    <div className="cord-trajectory-layout">
      <div className="cord-trajectory-plot" role="img" aria-label={tr("出线口坐标系下的末端轨迹", "Endpoint path in the outlet frame")}>
        <ReactECharts option={option} lazyUpdate style={{ width: "100%", height: "100%" }} />
      </div>
      <Space direction="vertical" size={12}>
        <Typography.Text type="secondary">{tr(
          "以出线口为原点，+X 沿中心方向向外，+Y 向左。测绘时保持绳线拉直，并在同一平面移动手柄。",
          "The outlet is the origin. +X points out along the centre line and +Y to its left. Keep the cord taut and move the handle in the same plane."
        )}</Typography.Text>
        <Descriptions size="small" column={1}>
          <Descriptions.Item label={tr("末端坐标 (X, Y)", "Position (X, Y)")}>{pair(latest && [latest.x_m, latest.y_m], "m")}</Descriptions.Item>
          <Descriptions.Item label={tr("末端速度 (Vx, Vy)", "Velocity (Vx, Vy)")}>{pair(latest?.velocity_m_s ?? null, "m/s")}</Descriptions.Item>
          <Descriptions.Item label={tr("出线速度", "Payout speed")}>{latest ? `${latest.radial_speed_m_s.toFixed(3)} m/s` : "—"}</Descriptions.Item>
        </Descriptions>
        <div className="cord-trajectory-legend">
          <span><i className="outlet" />{tr("出线口", "Outlet")}</span>
          <span><i className="trail" />{tr("轨迹", "Trail")}</span>
          <span><i className="endpoint" />{tr("当前末端", "Live endpoint")}</span>
        </div>
      </Space>
    </div>
  </Card>;
}
