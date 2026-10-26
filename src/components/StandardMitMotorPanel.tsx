import { standardMitApi } from "../el05Api";
import { MotorCanWorkspace } from "./El05MotorPanel";

export function StandardMitMotorPanel() {
  return <MotorCanWorkspace protocol="StandardMit" api={standardMitApi} />;
}
