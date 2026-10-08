"""
Ch7.X 실습 — OmniGraph로 키보드 제어

Python API(og.Controller.edit)를 사용하여 Nova Carter AMR의
키보드 제어 Action Graph를 자동 생성한다.
GUI에서 Window > Visual Scripting > Action Graph로 동일한 결과를 확인할 수 있다.
"""

from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf, Sdf
from isaacsim.core.api.world import World

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

NUCLEUS = "omniverse://localhost/NVIDIA/Assets/Isaac/6.1"
CARTER_USD = f"{NUCLEUS}/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

stage = omni.usd.get_context().get_stage()
prim = stage.DefinePrim("/World/NovaCarter")
prim.GetReferences().AddReference(CARTER_USD)

xform = UsdGeom.Xformable(prim)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

world.reset()

print("=" * 70)
print("OmniGraph Action Graph 생성 중...")
print("=" * 70)

keys = og.Controller.Keys

try:
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("diff_ctrl", "omni.isaac.wheeled_robots.DifferentialController"),
                ("art_ctrl", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("diff_ctrl.inputs:maxLinearSpeed", 1.0),
                ("diff_ctrl.inputs:maxAngularSpeed", 1.0),
                ("diff_ctrl.inputs:wheelDistance", 0.41),
                ("diff_ctrl.inputs:wheelRadius", 0.08),
                ("art_ctrl.inputs:robotPath", "/World/NovaCarter"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "diff_ctrl.inputs:execIn"),
                ("diff_ctrl.outputs:execOut", "art_ctrl.inputs:execIn"),
                ("diff_ctrl.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
            ],
        },
    )

    print(f"Action Graph 생성 완료: 노드 {len(nodes)}개")
    for node in nodes:
        print(f"  - {node.get_prim_path()}")

except Exception as e:
    print(f"OmniGraph 생성 실패: {e}")
    print("노드 이름이 Isaac Sim 버전에 따라 다를 수 있습니다.")
    print("Action Graph 에디터에서 검색하여 정확한 이름을 확인하세요.")

print("\n" + "=" * 70)
print("OmniGraph 확인 방법:")
print("  1. Window > Visual Scripting > Action Graph 열기")
print("  2. /World/ActionGraph를 선택하면 노드 구성이 보임")
print("  3. 각 노드를 클릭하여 속성 확인")
print("=" * 70)

print("\n키보드 제어 GUI를 통해 확인:")
print("  - Isaac Utils > Common Omnigraphs > Differential Controller")
print("  - 로봇 경로와 바퀴 관절 이름 입력 후 Create")
print("  - Play 버튼을 누르고 W/A/S/D로 조종")

print("\n시뮬레이션을 시작합니다. GUI에서 Action Graph를 확인해 보세요.")

while app.is_running():
    world.step(render=True)

app.close()
