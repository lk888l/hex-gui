export type LingkongMotorState = "running" | "closed" | "unknown";

// V2.36 defines 0x00/0x10. MHF5010 with DHF50 was observed to use
// 0x30/0x20 for running/closed; other values remain unknown.
export function lingkongMotorState(state: number | undefined): LingkongMotorState {
  switch (state) {
    case 0x00:
    case 0x30:
      return "running";
    case 0x10:
    case 0x20:
      return "closed";
    default:
      return "unknown";
  }
}
