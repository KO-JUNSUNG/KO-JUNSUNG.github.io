---
title: "Quadrotor min-snap trajectory generation: 구현 (basis · Q · KKT solve)"
date: 2026-07-02 10:00:00
permalink: /posts/2026/07/primitive-planner-2/
tags:
  - drone
  - trajectory-optimization
  - min-snap
  - implementation
  - python
---

> [이론 편](/posts/2026/07/primitive-planner-1/)에서 유도한 min-snap QP를 실제 코드로 옮기는 구현 편이다.
> 이론 편의 결론(스냅 제곱 적분 = 이차형식 $\mathbf{c}^\top Q\mathbf{c}$, 선형 제약 $A\mathbf{c}=\mathbf{b}$, KKT 선형 시스템)을 그대로 블록 단위로 구현한다.

---

## 구현 (작성 중)

> 아래는 직접 구현할 골격이다. 각 블록은 강의에서 유도한 수식을 그대로 코드로 옮긴 것.

- **블록 1 — 시간 basis:** `basis(t, r, N)` → $r$차 미분 basis 행벡터 $\beta_r(t)$.
- **블록 2 — 비용 행렬:** `cost_matrix_Q(T, N, r=4)` → 닫힌형 $Q$.
- **블록 3 — 제약 조립:** `build_A_b(waypoints, times, ...)` → 경계·연속 조건을 행으로 쌓아 $A,\mathbf{b}$.
- **블록 4 — KKT solve:** 블록 행렬을 만들어 `np.linalg.solve` 한 번.
- **블록 5 — 평가·플롯:** 계수로부터 위치·속도·가속도·저크·스냅을 복원해 시각화.

```python
# (여기에 내가 짠 코드가 들어갈 자리)
```
