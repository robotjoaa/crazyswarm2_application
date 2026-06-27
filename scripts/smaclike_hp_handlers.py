"""
HP event handlers for smaclike_aviary.

Each handler is notified when a unit takes damage (on_hit), when it is
initialized at combat start (on_init), and when it dies (on_death).

To add a new feedback channel — a sound effect, a score topic, a logging
sink, etc. — create a subclass of HpEventHandler, implement the methods you
need, and append an instance to SmaclikeAviary._hp_event_handlers.
"""

from __future__ import annotations
from typing import TYPE_CHECKING

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

if TYPE_CHECKING:
    from smaclike.units.unit import Unit


# ─────────────────────────────────────────────────────────────────────────────
#  Base
# ─────────────────────────────────────────────────────────────────────────────

class HpEventHandler:
    """
    Base for HP event handlers.  Override the methods you need.

    Methods
    -------
    on_init(unit)
        Called once per unit when combat starts so handlers can set initial
        state (e.g. LED ring mode).
    on_hit(unit, shooter_unit, old_hp, new_hp)
        Called when `unit` takes a hit and its HP decreases.
    on_death(unit)
        Called immediately after HP reaches 0 (before exit-sequence starts).
    """

    def on_init(self, unit: 'Unit') -> None:
        pass

    def on_hit(self, unit: 'Unit', shooter_unit: 'Unit',
               old_hp: float, new_hp: float) -> None:
        pass

    def on_death(self, unit: 'Unit') -> None:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  RViz: transient hit-flash spheres
# ─────────────────────────────────────────────────────────────────────────────

class RvizHpHandler(HpEventHandler):
    """
    Publishes transient sphere markers on ``rviz/hp_events``.

    - on_hit  : small sphere whose colour fades green→red with HP fraction,
                visible for 0.5 s.
    - on_death: larger solid-red sphere, visible for 2 s.

    These complement the existing 1 Hz ``rviz/unit_info`` text markers which
    always show the current HP value.
    """

    _ID_LIMIT = 2000  # recycle marker IDs to avoid stale-marker accumulation

    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(
            MarkerArray, 'rviz/hp_events', node.get_qos_profile(10))
        self._next_id = 0

    # ── helpers ───────────────────────────────────────────────────────────────

    def _alloc_id(self) -> int:
        mid = self._next_id
        self._next_id = (self._next_id + 1) % self._ID_LIMIT
        return mid

    def _make_sphere(self, unit: 'Unit', scale: float,
                     color: ColorRGBA, lifetime_ns: int) -> Marker:
        pos = unit.get_pos()
        m = Marker()
        m.header.frame_id = "/world"
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "hp_events"
        m.id = self._alloc_id()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        if pos is not None:
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        m.lifetime.nanosec = lifetime_ns
        return m

    @staticmethod
    def _hp_color(unit: 'Unit') -> ColorRGBA:
        """Green (full) → red (empty) gradient based on current HP fraction."""
        hp_frac = max(0.0, min(1.0, unit.hp / unit.max_hp))
        return ColorRGBA(r=1.0 - hp_frac, g=hp_frac, b=0.0, a=0.85)

    def _publish(self, marker: Marker) -> None:
        arr = MarkerArray()
        arr.markers.append(marker)
        self._pub.publish(arr)

    # ── interface ─────────────────────────────────────────────────────────────

    def on_hit(self, unit: 'Unit', shooter_unit: 'Unit',
               old_hp: float, new_hp: float) -> None:
        m = self._make_sphere(unit, scale=0.25, color=self._hp_color(unit),
                              lifetime_ns=500_000_000)   # 0.5 s
        self._publish(m)

    def on_death(self, unit: 'Unit') -> None:
        m = self._make_sphere(unit, scale=0.50,
                              color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                              lifetime_ns=2_000_000_000)  # 2 s
        self._publish(m)


# ─────────────────────────────────────────────────────────────────────────────
#  Hardware: Crazyflie LED ring
# ─────────────────────────────────────────────────────────────────────────────

class LedHpHandler(HpEventHandler):
    """
    Controls the Crazyflie LED ring deck via the crazyflie_server parameter
    service (``/crazyflie_server/set_parameters``).

    Requires the LED ring deck to be physically attached.  The handler checks
    whether the service is ready before each call and silently skips otherwise,
    so it degrades gracefully in simulation / when no deck is present.

    Color scheme
    ------------
    - Ally  : green (100 % HP) → yellow → red   (0 % HP)
    - Enemy : blue  (100 % HP) → cyan  → white  (0 % HP)

    on_init  sets ``ring/effect = 7`` (solid-color mode) for the drone, then
    applies the initial color.  on_hit updates the color.  on_death turns the
    ring off (0, 0, 0).

    TODO: per-LED HP bar (N/12 LEDs lit)
    -------------------------------------
    ``ring/effect = 13`` (virtualMemEffect) reads ``ledringmem[12][2]`` —
    12 LEDs × 2 bytes RGB565.  One CRTP packet holds all 24 bytes.
    RGB565 encoding: byte0 = (R5<<3)|(G6>>3), byte1 = ((G6&7)<<5)|B5
    where R5=r8>>3, G6=g8>>2, B5=b8>>3.

    Implementation requires:
    1. ``writeLedRing(vector<uint8_t>)`` in Crazyflie.cpp (same pattern as
       uploadTrajectory but targeting MemoryTypeLED12 = 0x10).
    2. ``WriteLedRing.srv`` service exposed per-CF in crazyflie_server.cpp.
    3. ``uint8[] data`` field added to UserCommand.msg.
    4. ``set_led_ring`` relay handler in crazyswarm_app.cpp.
    5. Pattern generator here: ceil(12*hp_frac) LEDs lit, each at the
       HP-fraction color; remaining LEDs encoded as (0, 0) = off.
    """

    _RING_EFFECT_SOLID = 7

    def __init__(self, node, cf_prefix: str):
        self._node = node
        self._cf_prefix = cf_prefix          # e.g. "cf_"
        self._client = node.create_client(
            SetParameters, '/crazyflie_server/set_parameters')

    # ── helpers ───────────────────────────────────────────────────────────────

    def _ready(self) -> bool:
        return self._client.service_is_ready()

    def _param(self, ros_name: str, value: int) -> Parameter:
        pv = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        return Parameter(name=ros_name, value=pv)

    def _cf_param(self, cf_name: str, fw_param: str, value: int) -> Parameter:
        """Map a CF firmware parameter to its ROS parameter name."""
        return self._param(f'{cf_name}.params.{fw_param}', value)

    def _push(self, params: list) -> None:
        if not self._ready():
            return
        req = SetParameters.Request()
        req.parameters = params
        self._client.call_async(req)

    @staticmethod
    def _hp_to_rgb(unit: 'Unit') -> tuple[int, int, int]:
        from smaclike.util.faction import Faction
        hp_frac = max(0.0, min(1.0, unit.hp / unit.max_hp))
        if unit.faction == Faction.ALLY:
            # green → yellow → red
            if hp_frac > 0.5:
                r = int(2 * (1.0 - hp_frac) * 255)
                g, b = 255, 0
            else:
                r, g, b = 255, int(2 * hp_frac * 255), 0
        else:
            # blue → cyan → white
            if hp_frac > 0.5:
                r, g = 0, int(2 * (1.0 - hp_frac) * 255)
                b = 255
            else:
                r = int((1.0 - 2 * hp_frac) * 180)   # slight red tint at low HP
                g, b = 255, 255
        return r, g, b

    def _set_rgb(self, unit: 'Unit') -> None:
        cf_name = f'{self._cf_prefix}{unit.id + 1}'
        r, g, b = self._hp_to_rgb(unit)
        self._push([
            self._cf_param(cf_name, 'ring/solidRed',   r),
            self._cf_param(cf_name, 'ring/solidGreen', g),
            self._cf_param(cf_name, 'ring/solidBlue',  b),
        ])

    # ── interface ─────────────────────────────────────────────────────────────

    def on_init(self, unit: 'Unit') -> None:
        cf_name = f'{self._cf_prefix}{unit.id + 1}'
        # Switch to solid-color ring mode first, then paint initial HP color.
        self._push([self._cf_param(cf_name, 'ring/effect', self._RING_EFFECT_SOLID)])
        self._set_rgb(unit)

    def on_hit(self, unit: 'Unit', shooter_unit: 'Unit',
               old_hp: float, new_hp: float) -> None:
        self._set_rgb(unit)

    def on_death(self, unit: 'Unit') -> None:
        cf_name = f'{self._cf_prefix}{unit.id + 1}'
        self._push([
            self._cf_param(cf_name, 'ring/solidRed',   0),
            self._cf_param(cf_name, 'ring/solidGreen', 0),
            self._cf_param(cf_name, 'ring/solidBlue',  0),
        ])
