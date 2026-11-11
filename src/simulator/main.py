from __future__ import annotations

import argparse
import asyncio
import pathlib
import sys
from typing import Optional

ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
  sys.path.append(str(ROOT))

# Import via the package so running as a module or a script both work.
from simulator.agsg_controller import AgsgController, Config  # noqa: E402
from simulator.server import TelemetryServer  # noqa: E402


async def control_loop(server: TelemetryServer, interval: float = 0.1) -> None:
  """
  Simple placeholder control loop.
  - If we have a pose, command gentle forward motion.
  - If a sign is seen, rotate slowly toward it.
  """

  # Wait for a UI/WebSocket client before issuing control.
  await server.wait_connected()
  # Reset telemetry/odom on (re)connect to avoid stale state.
  server.state.reset()

  await server.send_log("Starting simulation controller")

  try:
    while True:
        log_msg = (
        f"distances={server.state.distances} "
        f"sign={server.state.sign} "
        f"heading={server.hal.heading.get_heading_deg()} "
        f"odom={server.state.odom}"
        )
        #await server.send_log(log_msg)

        await server.send_control(velocity=0.12)
        await asyncio.sleep(interval)
  except asyncio.CancelledError:
    raise


async def main(host: str, port: int, controller: str = "agsg", config: Optional[Config] = None) -> None:
  server = TelemetryServer(host=host, port=port)
  print(f"[controller] listening on ws://{host}:{port} ({controller})")
  brain = AgsgController(server, config) if controller == "agsg" else _Placeholder(server)
  await asyncio.gather(server.serve_forever(), brain.run())


class _Placeholder:
  """The original stand-in brain, kept behind --controller placeholder."""

  def __init__(self, server: TelemetryServer) -> None:
    self.server = server

  async def run(self) -> None:
    await control_loop(self.server)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="WebSocket simulation controller")
  parser.add_argument("--host", default="0.0.0.0", help="Host to bind the WS server (default: 0.0.0.0)")
  parser.add_argument("--port", type=int, default=8765, help="Port for the WS server (default: 8765)")
  parser.add_argument(
    "--controller",
    default="agsg",
    choices=["agsg", "placeholder"],
    help="Brain driving the car: the vendored AGSG firmware (default) or the old placeholder loop",
  )
  # Ackermann and frame knobs. Calibrate them on the real car (README, 'Bench calibration').
  parser.add_argument("--wheelbase-mm", type=float, default=70.0, help="Wheelbase in mm (default: 70)")
  parser.add_argument("--servo-gain", type=float, default=1.0, help="delta_eff = gain * servo_angle + offset")
  parser.add_argument("--servo-offset-deg", type=float, default=0.0, help="Servo centre offset in degrees")
  parser.add_argument(
    "--servo-sign",
    type=float,
    default=-1.0,
    choices=[1.0, -1.0],
    help="Direction for a positive servo angle (measured: -1 for the AGSG car)",
  )
  parser.add_argument(
    "--imu-sign",
    type=float,
    default=-1.0,
    choices=[1.0, -1.0],
    help="Firmware heading sign vs world heading. -1 keeps its odometry consistent (2-25 cm); +1 makes the camera POV work but mirrors the odometry - see FIRMWARE_NOTES.md",
  )
  parser.add_argument(
    "--imu-offset-deg",
    type=float,
    default=0.0,
    help="Heading offset of the IMU mounting (0 = zero at boot, +90 if the firmware expects 0 = north)",
  )
  parser.add_argument(
    "--actuator",
    default="target_speed",
    choices=["target_speed", "pwm"],
    help="Speed command from the cruise setpoint (default) or from the motor duty",
  )
  parser.add_argument("--pwm-cm-per-duty", type=float, default=150.0, help="Duty->speed constant for --actuator pwm")
  parser.add_argument("--verbose", action="store_true", help="Mirror firmware telemetry into the UI log")
  parser.add_argument(
    "--lockstep", action="store_true",
    help="Advance the world one physics step per firmware sleep (reproducible rounds)",
  )
  return parser.parse_args(argv)


if __name__ == "__main__":
  args = parse_args()
  config = Config(
    actuator=args.actuator,
    pwm_cm_per_duty=args.pwm_cm_per_duty,
    wheelbase_m=args.wheelbase_mm / 1000.0,
    servo_gain=args.servo_gain,
    servo_offset_deg=args.servo_offset_deg,
    servo_sign=args.servo_sign,
    imu_sign=args.imu_sign,
    imu_offset_deg=args.imu_offset_deg,
    verbose=args.verbose,
    lockstep=args.lockstep,
  )
  try:
    asyncio.run(main(args.host, args.port, args.controller, config))
  except KeyboardInterrupt:
    print("\n[controller] stopped")


