"""config + event_bus + world_model testleri."""
import asyncio

import pytest

from uav.config.config_manager import ConfigManager
from uav.core.types import Boundary, HSSZone, Position, UAVState
from uav.infra.event_bus import Event, EventBus, EventType
from uav.world_model.world_model import WorldModel

CONFIG = "config/default.yaml"


def test_config_dotted_get():
    cfg = ConfigManager(CONFIG)
    assert cfg.get_float("nav.planner.hss_buffer_m") == 40.0
    assert cfg.get_int("nav.planner.polygon_segments") == 12
    assert cfg.get_str("mavlink.pixhawk.connection").startswith("udp")
    assert cfg.get_float("nonexistent.key", 7.5) == 7.5
    assert cfg.has_key("safety.hss_buffer_m")
    assert not cfg.has_key("safety.does_not_exist")


def test_config_boundary_node():
    cfg = ConfigManager(CONFIG)
    verts = cfg.get_node("boundary.vertices")
    assert isinstance(verts, list) and len(verts) >= 3


@pytest.mark.asyncio
async def test_event_bus_async_handler():
    bus = EventBus()
    received = []

    async def handler(ev: Event):
        received.append(ev.payload)

    bus.subscribe(EventType.OWN_STATE_UPDATED, handler)
    await bus.start()
    bus.publish(EventType.OWN_STATE_UPDATED, 42)
    await asyncio.sleep(0.05)
    await bus.stop()
    assert received == [42]


@pytest.mark.asyncio
async def test_event_bus_sync_handler_and_unsubscribe():
    bus = EventBus()
    hits = []
    sid = bus.subscribe(EventType.MODE_CHANGED, lambda ev: hits.append(ev.payload))
    await bus.start()
    bus.publish(EventType.MODE_CHANGED, "a")
    await asyncio.sleep(0.02)
    bus.unsubscribe(sid)
    bus.publish(EventType.MODE_CHANGED, "b")
    await asyncio.sleep(0.02)
    await bus.stop()
    assert hits == ["a"]


def test_world_model_own_and_opponents():
    wm = WorldModel()
    s = UAVState(id=1, position=Position(37.8, -122.3, 100), timestamp_us=1)
    wm.set_own_state(s)
    assert wm.get_own_state().position.latitude == 37.8

    wm.update_opponent(UAVState(id=5, position=Position(37.81, -122.31, 100), timestamp_us=1))
    wm.update_opponent(UAVState(id=6, position=Position(37.9, -122.4, 100), timestamp_us=1))
    assert len(wm.get_all_opponents()) == 2
    nearest = wm.get_nearest_opponent(Position(37.8, -122.3, 100))
    assert nearest.id == 5
    wm.remove_opponent(5)
    assert wm.get_opponent(5) is None


def test_world_model_returns_copies():
    wm = WorldModel()
    s = UAVState(id=1, position=Position(1, 2, 3), timestamp_us=1)
    wm.set_own_state(s)
    got = wm.get_own_state()
    got.position.latitude = 999.0
    assert wm.get_own_state().position.latitude == 1.0  # ic durum bozulmadi


def test_world_model_hss_and_boundary():
    wm = WorldModel()
    wm.update_hss_zone(HSSZone(id=1, center=Position(0, 0), radius=100, active=True))
    wm.update_hss_zone(HSSZone(id=2, center=Position(0, 0), radius=100, active=False))
    assert len(wm.get_active_hss_zones()) == 1
    assert not wm.has_boundary()
    wm.set_boundary(Boundary(vertices=[Position(0, 0), Position(0, 1), Position(1, 1)]))
    assert wm.has_boundary()
