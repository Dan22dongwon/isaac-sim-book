"""책 18장 [실습] 렌더링 랜덤화 (b03) — 정리본.

원문과의 차이:
  없음 — Replicator 랜덤화 패턴 그대로. rep 컨텍스트/씬이 있어야 실제로 돈다.
  (조명 intensity 는 생성한 light 의 스키마에 따라 ERR-106 유형 주의 — DomeLight 는 int)

검증 범위:
  rep.get.prims / rep.modify.attribute / rep.distribution.uniform API 존재 확인.
  씬 없이 함수 자체는 실행 대상이 아니라 등록만 한다.
"""
import omni.replicator.core as rep


def randomize_rendering():
    # 조명 강도·색온도 (Ch15 EventTerm 과 연동 가능)
    with rep.get.prims(semantics=[("class", "light")]):
        rep.modify.attribute("intensity", rep.distribution.uniform(300, 5000))
        rep.modify.attribute("colorTemperature", rep.distribution.uniform(2700, 6500))
    # 카메라 노출·화이트밸런스는 후처리로 모사 (실제 카메라 자동노출 특성)


rep.randomizer.register(randomize_rendering)
