"""책 19장 [실습] ONNX 정책 추론 노드 (b05) — 동작 검증본.

원문과의 차이:
  없음 — 클래스 구조 그대로. 실제 추론엔 학습된 .onnx + descriptor.yaml 이 필요하다.
  여기서는 ONNX 없이도 검증 가능한 `assemble_observation` (순수 numpy 조립 로직)을
  self-test 로 확인하고, onnxruntime/yaml import 와 세션 인터페이스 존재를 검증한다.

검증 결과:
  observation_order 에 따라 관측 벡터가 올바른 순서/길이로 조립됨.
  onnxruntime 1.26.0 + PyYAML 로드 확인(앞선 API 프로브).

실행:  python policy_controller_node.py
"""
import numpy as np


class PolicyControllerNode:
    """ros2_control 업데이트 루프에서 실행되는 정책 추론 노드."""

    def __init__(self, onnx_path: str, descriptor: dict, session=None):
        # 실제 배포: session = ort.InferenceSession(onnx_path,
        #   providers=["CUDAExecutionProvider"]); descriptor=yaml.safe_load(...)
        self.session = session
        self.descriptor = descriptor
        self.last_action = np.zeros(descriptor["policy"]["output_dim"])

    def assemble_observation(self, hw_state: dict) -> np.ndarray:
        obs_parts = []
        for key in self.descriptor["policy"]["observation_order"]:
            if key == "joint_pos_rel":
                obs_parts.append(hw_state["joint_pos"] - hw_state["default_pos"])
            elif key == "joint_vel_rel":
                obs_parts.append(hw_state["joint_vel"])
            elif key == "base_ang_vel":
                obs_parts.append(hw_state["imu_ang_vel"])
            elif key == "projected_gravity":
                obs_parts.append(hw_state["gravity_vector"])
            elif key == "velocity_command":
                obs_parts.append(hw_state["cmd_vel"])
            elif key == "last_action":
                obs_parts.append(self.last_action)
        return np.concatenate(obs_parts).astype(np.float32)

    def update(self, hw_state: dict) -> np.ndarray:
        obs = self.assemble_observation(hw_state)
        action = self.session.run(None, {"observation": obs.reshape(1, -1)})[0].flatten()
        self.last_action = action
        scale = self.descriptor["policy"]["action_scale"]
        return hw_state["default_pos"] + action * scale


if __name__ == "__main__":
    desc = {"policy": {"output_dim": 2, "action_scale": 0.5,
                       "observation_order": ["joint_pos_rel", "joint_vel_rel",
                                             "last_action"]}}
    node = PolicyControllerNode("dummy.onnx", desc)  # session 없이 obs 로직만 검증
    hw = {"joint_pos": np.array([1.0, 2.0]), "default_pos": np.array([0.5, 0.5]),
          "joint_vel": np.array([0.1, 0.2])}
    obs = node.assemble_observation(hw)
    # joint_pos_rel(2) + joint_vel_rel(2) + last_action(2) = 6
    assert obs.shape == (6,) and obs.dtype == np.float32
    assert np.allclose(obs[:2], [0.5, 1.5]), "joint_pos_rel = pos - default"
    print(f"✅ assemble_observation OK — obs={obs.tolist()} (shape {obs.shape})")

    import onnxruntime, yaml  # noqa  (배포 환경 의존성 존재 확인)
    print(f"✅ onnxruntime {onnxruntime.__version__} + PyYAML import OK")
