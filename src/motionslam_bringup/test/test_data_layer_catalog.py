#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from data_layer_types import (  # noqa: E402
    CATALOG_SCHEMA,
    DataLayerId,
    LayerCatalog,
    ViewState,
    build_catalog_snapshot,
    wrap_data_envelope,
)
from execution_feedback import SCHEMA as FEEDBACK_SCHEMA  # noqa: E402
from execution_feedback import ExecutionFeedbackInput, build_execution_feedback_v1  # noqa: E402
from subgraph_snapshot import LAYER_ID as SUBGRAPH_LAYER  # noqa: E402
from subgraph_snapshot import SubgraphBuildInput, build_sparse_subgraph_v1  # noqa: E402


class TestDataLayerCatalog(unittest.TestCase):
    def test_catalog_has_all_default_layers(self) -> None:
        catalog = LayerCatalog(session_id="s1", map_id="m1")
        snap = build_catalog_snapshot(catalog)
        self.assertEqual(snap["schema"], CATALOG_SCHEMA)
        layer_ids = {layer["layer_id"] for layer in snap["layers"]}
        self.assertIn(DataLayerId.TOPOLOGY_SUBGRAPH.value, layer_ids)
        self.assertIn(DataLayerId.EXEC_FEEDBACK.value, layer_ids)

    def test_context_generation_marks_stale(self) -> None:
        catalog = LayerCatalog()
        catalog.note_publish(
            DataLayerId.TOPOLOGY_SUBGRAPH.value,
            layer_version=1,
            context_generation=0,
        )
        layer = catalog.layers[DataLayerId.TOPOLOGY_SUBGRAPH.value]
        self.assertEqual(layer.view_state, ViewState.READY.value)
        catalog.bump_context_generation(1)
        self.assertEqual(layer.view_state, ViewState.STALE.value)

    def test_subgraph_has_layer_id(self) -> None:
        snap = build_sparse_subgraph_v1(
            SubgraphBuildInput(
                version=1,
                session_id="s",
                floor_id="floor_01",
                tile_id="main",
                map_id="m",
                trigger_reason="periodic",
                robot_x=0,
                robot_y=0,
                robot_z=0,
                robot_yaw=0,
            )
        )
        self.assertEqual(snap["layer_id"], SUBGRAPH_LAYER)
        self.assertEqual(snap["floor_id"], "floor_01")

    def test_execution_feedback_schema(self) -> None:
        fb = build_execution_feedback_v1(
            ExecutionFeedbackInput(
                session_id="s",
                floor_id="floor_01",
                tile_id="main",
                map_id="m",
                context_generation=2,
                event="subgoal_reached",
            )
        )
        self.assertEqual(fb["schema"], FEEDBACK_SCHEMA)
        self.assertEqual(fb["layer_id"], DataLayerId.EXEC_FEEDBACK.value)

    def test_envelope_wrap(self) -> None:
        inner = {"schema": "test.v1", "x": 1}
        env = wrap_data_envelope(
            layer_id=DataLayerId.TOPOLOGY_SUBGRAPH.value,
            payload_schema="test.v1",
            payload=inner,
            session_id="s",
            floor_id="floor_01",
            tile_id="main",
            map_id="m",
            context_generation=0,
            layer_version=1,
        )
        self.assertEqual(env["payload_schema"], "test.v1")
        self.assertEqual(env["payload"]["x"], 1)


if __name__ == "__main__":
    unittest.main()
