"""책 23장 [실습] 커스텀 Isaac Sim Extension (b09) — 동작 검증본.

원문과의 차이:
  없음 — omni.ext.IExt 서브클래스 구조 그대로.

검증 결과 (SimulationApp 내):
  omni.ext.IExt 존재, MyCustomExtension 서브클래스 정의/인스턴스화 OK,
  on_startup/on_shutdown 호출 동작.

실행:  python custom_extension.py
"""
import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import omni.ext


class MyCustomExtension(omni.ext.IExt):
    """커스텀 Isaac Sim Extension."""

    def on_startup(self, ext_id):
        print("[my_extension] 확장 활성화됨")
        self._register_custom_tools()

    def on_shutdown(self):
        print("[my_extension] 확장 비활성화됨")
        self._cleanup()

    def _register_custom_tools(self):
        pass  # 예: 창고 씬 자동 생성 도구, 커스텀 분석 대시보드 등

    def _cleanup(self):
        pass


if __name__ == "__main__":
    assert issubclass(MyCustomExtension, omni.ext.IExt)
    ext = MyCustomExtension()
    ext.on_startup("my_extension")
    ext.on_shutdown()
    print("✅ MyCustomExtension(IExt) 정의·수명주기 호출 OK")
    app.close()
