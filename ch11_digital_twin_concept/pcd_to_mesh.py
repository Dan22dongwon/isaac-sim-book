"""책 11장 b03 — 포인트클라우드 → 메시 변환 (Open3D).

원문과의 차이:
  [ERR-086] 원고는 .py 파일 안에 파이썬이 아닌 산문 번호목록(1. ... 2. ...)을
            코드와 섞어 놓았다 → SyntaxError. 산문을 주석으로 변환했다.

사용법:
  python pcd_to_mesh.py                        # 테스트 구 자동 생성
  python pcd_to_mesh.py my_scan.pcd            # 실제 스캔 파일
  OUT=result.obj python pcd_to_mesh.py         # 출력 경로 지정
"""
import os
import sys

import numpy as np
import open3d as o3d

# ── 입력 ──
if len(sys.argv) > 1:
    pcd_path = sys.argv[1]
else:
    pcd_path = "/tmp/test_sphere.pcd"
    np.random.seed(42)
    n = 5000
    phi = np.random.uniform(0, 2 * np.pi, n)
    cos_theta = np.random.uniform(-1, 1, n)
    sin_theta = np.sqrt(1 - cos_theta**2)
    r = 0.5
    pts = np.stack(
        [r * sin_theta * np.cos(phi),
         r * sin_theta * np.sin(phi),
         r * cos_theta], axis=1,
    )
    pts += np.random.normal(0, 0.005, pts.shape)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"테스트 PCD 생성: {n} points → {pcd_path}")

# ── 원고 b03 코드 (산문 → 주석) ──
# 1. 포인트 클라우드 수집 (.e57, .las, .pts 등)
# 2. 노이즈 제거 및 다운샘플링
# 3. 메시 재구성 (Poisson Surface Reconstruction 등)
# 4. 텍스처 매핑 (사진 데이터 활용 시)
# 5. USD로 익스포트

pcd = o3d.io.read_point_cloud(pcd_path)
print(f"읽기: {len(pcd.points)} points")

pcd = pcd.voxel_down_sample(voxel_size=0.02)
print(f"다운샘플링: {len(pcd.points)} points")

pcd.estimate_normals()
print("법선 추정 완료")

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=9,
)
print(f"메시 생성: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

out_path = os.environ.get("OUT", "/tmp/factory_mesh.obj")
o3d.io.write_triangle_mesh(out_path, mesh)
print(f"OBJ 저장: {out_path}")
print("✅ b03 파이프라인 성공")
