# Chapter 11 — 디지털 트윈 개념

블록 5개 (원고 b01~b05). 코드 블록은 b03·b05 두 개, 나머지는 개념 설명·GUI 절차다.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-086** 🔴 | b03 | `.py` 안에 파이썬이 아닌 산문 번호목록이 코드와 섞여 있다 → `SyntaxError: invalid syntax` |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | 디지털 트윈 정의·개념 | ✅ 텍스트, 코드 없음 |
| b02 | Omniverse 아키텍처 | ✅ 텍스트, 코드 없음 |
| **b03** | **포인트클라우드 → 메시 (open3d)** | ✅ **수정 후 성공** — ERR-086, 산문→주석 |
| b04 | Nucleus 데이터 관리 | ✅ 텍스트, 코드 없음 |
| **b05** | **omni.client 자산 확인** | ✅ API 동작 확인 (Nucleus 서버 의존) |

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`pcd_to_mesh.py`](pcd_to_mesh.py) | 11장 b03 | ✅ | 수정 1건 (ERR-086) |

## ERR-086 — `.py` 안에 산문이 코드와 섞여 있다

원고 b03 파일의 4행째:

```
1. 포인트 클라우드 수집 (.e57, .las, .pts 등)
   ^^^
SyntaxError: invalid syntax
```

번호를 매긴 산문 5줄이 파이썬 코드 사이에 주석 표시 없이 들어가 있다.
`#` 주석으로 바꾸면 정상 동작한다.

## 실행 방법

```bash
conda activate isaac_env
python pcd_to_mesh.py                    # 테스트 구 자동 생성
python pcd_to_mesh.py my_scan.pcd        # 실제 스캔 파일
```

## 실측 결과

| 단계 | 값 |
|---|---|
| 입력 점 수 | 5,000 |
| 다운샘플링 (2cm) | 4,009 |
| Poisson depth=9 | 17,633 vertices, 35,204 triangles |
| OBJ 저장 | `/tmp/factory_mesh.obj` |

## b05 — omni.client (Nucleus 자산 확인)

원고 코드는 `omniverse://localhost`에 접속해 파일을 확인한다.
Nucleus 서버가 없으면 `Result.ERROR_CONNECTION` (15회 재시도 후 128초에 종료).
API 자체는 정상 — `omni.client.stat`, `omni.client.Result.OK` 모두 존재한다.

| 항목 | 결과 |
|---|---|
| `omni.client.stat` | ✅ 존재 |
| `omni.client.Result.OK` | ✅ 존재 |
| 서버 없이 실행 | `ERROR_CONNECTION` (오류 아님, 서버 의존) |

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch11/ch11_pcd_to_mesh.png`](../_docs/images/book/ch11/ch11_pcd_to_mesh.png) | Poisson 복원 메시 (법선 컬러맵) — 5000점 구에서 복원 |
