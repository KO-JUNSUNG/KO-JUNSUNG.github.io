---
title: "Quadrotor min-snap trajectory generation: differential-flatness to Quadratic Programming"
date: 2026-07-02 09:00:00
permalink: /posts/2026/07/primitive-planner-1/
tags:
  - drone
  - trajectory-optimization
  - min-snap
  - differential-flatness
  - QP
---

> Primitive-Planner를 바닥부터 재구현하기 위한 토대 학습(Layer 0) 정리.
> 통계·머신러닝 배경에서 출발해, 쿼드로터 궤적 생성이 왜 결국 하나의 QP로 귀결되는지를 유도 중심으로 정리한다.

---

## 0. 이 글이 답하려는 질문

드론이 여러 지점(waypoint)을 부드럽게 통과하는 궤적을 어떻게 만들까? 

쿼드로터의 상태는 12차원이고 동역학은 비선형인데, 그 제약을 전부 최적화에 넣으면 문제가 감당이 안 된다. 

이 글은 그 12차원을 **정직하게 4차원으로 줄이고**, 남은 문제를 **닫힌 해가 있는 하나의 QP**로 만드는 전체 사슬을 따라간다.

결론을 한 줄로 미리 적으면 이렇다:

> Differential flatness로 인해 4개 곡선만 그리면 되고 → 다항식으로 표현하니 모든 게 계수에 선형 → 스냅 제곱 적분이 이차형식 → 제약이 선형 등식 → 라그랑주로 묶으면 KKT 선형 시스템 하나로 표현 가능.

---

## 1. Differential flatness(미분평탄성) — 12차원은 사실 4차원이었다

**답할 질문: 왜 위치(x,y,z)와 $\psi$ (yaw), 단 4개의 곡선만 정하면 드론의 나머지 상태와 입력이 전부 따라 나오는가?**

시스템이 미분 평탄하다(Differentialy Flat)하다는 것은, 다음 조건을 만족하는 평탄 출력(Flat output)이라는 가상의 출력이 존재한다는 뜻이다. 

```
"시스템의 모든 상태 변수(State)와 입력 변수(Input)를, 평탄 출력과 그 출력의 유한한 차수 미분값들만의 대수적 함수로 유일하게 나타낼 수 있다."
```

즉, 미분 방정식(적분 과정)을 풀 필요 없이, 평탄 출력의 값과 그 미분값들만 알면 시스템의 현재 상태와 입력이 자동으로 계산되는 성질을 말한다. 

  쿼드로터는 일반적으로 이 성질을 이용하여 설명할 수 있다. 이를 위해 쿼드로터의 상태에 대해 먼저 간단히 설명하겠다. 

### 12차원 상태의 정체

쿼드로터의 상태 $x$는 "3개짜리 물리량 4묶음"이다.

| 묶음 | 기호 | 뜻 |
|---|---|---|
| 위치 | $\mathbf{p}=(x,y,z)$ | 공간 어디에 |
| 속도 | $\mathbf{v}=\dot{\mathbf{p}}$ | 어디로 얼마나 빨리 |
| 자세 | $(\phi,\theta,\psi)$ = (roll, pitch, yaw) | 몸통이 어느 쪽으로 기울었나 |
| 각속도 | $\boldsymbol{\omega}=(p,q,r)$ | 각 축으로 얼마나 빨리 도나 |

제어 입력 $u$는 프로펠러가 4개이므로(쿼드로터니까) 정확히 4개다: 총추력 $T$ 하나와 세 축 토크 $(\tau_x,\tau_y,\tau_z)$. 수식으로 나타내면 아래와 같다. 

$$u=(T, \tau_x,\tau_y,\tau_z)$$

### 핵심: 가속도 $\ddot{\mathbf{p}}$가 기울기 $\mathbf{b}_3$를 강제한다

드론이 낼 수 있는 힘은 두 가지 뿐이다. 

1. 추력(항상 몸통 위쪽 $\mathbf{b}_3$ 방향)
2. 중력($g$)

![호버 vs 가속 시 추력·중력 벡터](/images/TIL/thrust_tilt_forces_hover_vs_accelerate.svg)


옆으로 미는 힘이 없으므로, 옆으로 가속하려면 반드시 그쪽으로 기울여야 한다. 

뉴턴 법칙으로 쓰면:

$$
m\ddot{\mathbf{p}} = T\mathbf{b}_3 - mg\mathbf{e}_3
\quad\Longrightarrow\quad
\mathbf{b}_3 = \frac{\ddot{\mathbf{p}}+g\mathbf{e}_3}{\lVert \ddot{\mathbf{p}}+g\mathbf{e}_3\rVert},\quad
T = m\lVert \ddot{\mathbf{p}}+g\mathbf{e}_3\rVert
$$

여기서 $\ddot{\mathbf{p}}$는 위치 $\mathbf{p}$를 두 번 미분한 **가속도**, $m$은 드론 질량, $g$는 중력가속도, $\mathbf{e}_3=(0,0,1)$은 world coordination의 수직 위 방향 단위벡터다. $\mathbf{b}_3$는 body frame 위쪽 방향 단위벡터(=기울기 방향), $T$는 총추력이다.

즉 **위치 곡선을 두 번 미분하면 기울기 방향 $\mathbf{b}_3$와 추력 $T$가 하나로 확정된다.** 추정이 아니라 방정식이 강제하는 유일한 답이다.

### 왜 하필 "4"인가

이 힘 방정식은 $\mathbf{b}_3$(자유도 2, 단위벡터의 방향)만 정해준다. 
<details>
<summary> 왜? </summary>
구면 위의 한 점 = 위도(Latitude), 경도(Longitude) 2개로 표현 가능 = 자유도가 2.
</details>
  
  
자세(Attitude)는 원래 자유도 3인 회전이므로, **그 $\mathbf{b}_3$ 축을 중심으로 도는 마지막 자유도 1**이 남는다. 
  힘 방정식은 이 회전에 대해 완전히 불변이라(추력은 어차피 $\mathbf{b}_3$ 방향이니 몸을 돌려도 안 변함), 이 마지막 자유도는 **밖에서 yaw로 공급**해야 한다.

![psi가 필요한 이유](/images/TIL/b3_fixes_two_dof_yaw_fills_last.svg)

정리하면 평탄출력의 개수는 입력의 개수와 같아야 한다:

| 평탄출력 | 결정하는 입력 |
|---|---|
| 위치 $x,y,z$ (3) | 추력 + roll·pitch 토크 (3) |
| yaw $\psi$ (1) | yaw 토크 (1) |

### 간단히 요약하면: 자유도(degrees of freedom)

데이터 100개가 직선 위에 있어야 한다는 제약을 받으면 진짜 자유도는 (기울기, 절편) 2개뿐이고, 나머지 98개는 그 2개의 결과다. 
  쿼드로터도 뉴턴 법칙이라는 제약 아래에서 진짜 자유도는 4개뿐이고, 나머지 8개는 그 결과다. 물리적으로 가능한 12차원 궤적은 12차원 공간을 채우지 않고 그 안의 **4차원 manifold** 위에만 존재한다. Flat output(평탄출력)은 그 manifold의 좌표다.

---

## 2. 다항식 표현 — 모든 것이 계수에 선형이다

**답할 질문: 4개의 평탄출력 곡선을 무슨 함수로 표현해야 궤적의 최적화가 쉬워지는가?**

한 축(예: x축)의 궤적을 $N$차 다항식으로 쓴다:

$$
x(t) = \sum_{i=0}^{N} c_i t^i
$$

다항식으로 표현하는 경우의 장점은 세 가지다. (1) 무한히 미분 가능해 매끄러움이 보장되고, (2) 몇 번을 미분해도 계속 다항식이라 스냅(4차 미분)까지 깔끔하며, (3) 결정적으로, **계수 $\mathbf{c}$에 대해 선형**이다.

세 번째가 왜 결정적인지 보자. 임의의 미분을 특정 시각 $t$에서 평가하면:

$$
x^{(r)}(t) = \sum_i c_i \frac{d^r}{dt^r}t^i
= \underbrace{\big[\dots,\ \tfrac{i!}{(i-r)!}t^{i-r},\ \dots\big]}_{\text{아는 상수 벡터 } \beta_r(t)^\top}\ \mathbf{c}
$$

이는 선형회귀 $y=\mathbf{w}^\top\boldsymbol{\phi}(x)$와 같은 구조다. 이해를 돕기 위해 아래 표를 통해 간단히 비교했다. 

| 선형회귀 | min-snap |
|---|---|
| 목표 $y$ | 궤적/미분값 $x^{(r)}(t)$ |
| 가중치 $\mathbf{w}$ (구할 것) | 계수 $\mathbf{c}$ (구할 것) |
| 특징 $\boldsymbol{\phi}(x)$ (아는 값) | 시간 basis $\beta_r(t)$ (아는 값) |

시간 $t$에 대해서는 비선형이지만 **미지수 $\mathbf{c}$에 대해서는 선형**이다. 이해가 잘 가지 않는다면 PRML 참조. 

---

## 3. 목적함수 — 왜 하필 min-snap인가

**답할 질문: 궤적의 부드러움(Smoothness)을 왜 "스냅(4차 미분)"으로 재고, 그 비용이 왜 이차형식이 되는가?**

### (Quadrotor의 경우) Snap은 Torque다

위치를 미분해 내려가는 사다리를 보면:

| 미분 차수 | 물리량 |
|---|---|
| 2차 (가속도) | 기울기 $\mathbf{b}_3$ + 추력 $T$ (입력 ①) |
| 3차 (jerk) | 각속도 $\boldsymbol{\omega}$ |
| 4차 (snap) | 토크 $\boldsymbol{\tau}$ (입력 ②) |

토크는 자세를 두 번 미분한 양(각가속도)에 붙는다. 
  
  그런데 자세 자체가 위치의 2차 미분이므로, 토크는 결국 **위치의 4차 미분 = 스냅**에 대응한다.
  
  즉 스냅을 최소화한다는 것은 **모터 토크를 최소한만 쓰는, 실현 가능하고 순한 궤적을 고른다**는 뜻이다.

이는 일종의 정규화(regularization)다. 

waypoint를 지나는 곡선은 무수히 많은데(과소결정), Lidge Regression이 $\lVert\mathbf{w}\rVert^2$로 과한 출렁임을 누르듯, $\int(x^{(4)})^2dt$는 궤적의 발작적 움직임에 거는 페널티다.

### 비용이 이차형식이 되는 이유

스냅이 $\mathbf{c}$에 선형($\ddddot x(t)=\beta_4(t)^\top\mathbf{c}$)이므로, 그것을 제곱해 적분하면 $\mathbf{c}$에 대한 이차형식이 된다:

$$
J = \int_0^T \big(\ddddot x(t)\big)^2 dt
= \mathbf{c}^\top \underbrace{\left[\int_0^T \beta_4(t)\beta_4(t)^\top dt\right]}_{Q} \mathbf{c}
= \mathbf{c}^\top Q\,\mathbf{c}
$$

### $Q$의 정체: 그램 행렬(Gram matrix)

$Q_{ij}=\langle \partial^4 t^i,\ \partial^4 t^j\rangle_{L^2[0,T]}$로, 선형회귀의 $X^\top X$와 같은 종류의 그램 행렬이다. 그램 행렬은 언제나 대칭·PSD인데, 그 이유가 곧 Convex의 뿌리다:

$$
\mathbf{c}^\top Q\mathbf{c} = \int_0^T (\ddddot x)^2 dt \ge 0 \quad\text{(제곱의 적분이므로)}
$$

닫힌 형태는 $Q_{ij}=\dfrac{i!}{(i-4)!}\dfrac{j!}{(j-4)!}\dfrac{T^{i+j-7}}{i+j-7}$ ($i,j\ge4$, 아니면 0)이다.

  여기서 중요한 관찰: **$c_0\sim c_3$(cubic 이하)은 4번 미분하면 사라지므로 스냅에 전혀 기여하지 않는다.** 
  
  따라서 $Q$는 왼쪽-위 $4\times4$가 통째로 0이고, 4차원 영공간(null space)을 가진다. 
  
  이 "공짜 자유도"를 정해주는 것이 다음 절의 제약이다.

---

## 4. 제약 — 경계·연속, 그리고 왜 7차인가

**답할 질문: $Q$만 최소화하면 답이 "$\mathbf{c}=0$(가만히 있기)"인데, 무엇으로 이를 막는가?**

두 종류의 선형 제약을 건다.

- **경계 조건:** 궤적 양 끝에서 위치·속도·가속도·저크를 지정 (예: 정지→정지).
- **연속 조건:** 구간 이음매에서 위치부터 6차 미분까지 양쪽 다항식을 일치.

두 조건 모두 "모든 미분은 $\mathbf{c}$에 선형"이라는 §2의 결론 덕분에 $(\text{아는 벡터})\cdot\mathbf{c}=\text{값}$ 한 줄로 써진다.
  쌓으면 $A\mathbf{c}=\mathbf{b}$가 된다. 예를 들어 $x(0)=P_0$은 $\beta_0(0)=[1,0,\dots,0]$이므로 그냥 $c_0=P_0$이다.

### 왜 7차인가 (한 구간 기준)

미지수 개수 = 제약 개수여야 자유도가 맞는다. 

  $N$차 다항식의 계수는 $N+1$개, 한 구간의 경계 제약은 (시작 위치·속도·가속도·저크 4개) + (끝 4개) = 8개다.

$$
N+1 = 8 \quad\Longrightarrow\quad N = 7
$$

더 일반적으로, **min-($k$차 미분) 문제의 최적 궤적은 $(2k-1)$차 다항식**이다.

  스냅은 $k=4$이므로 $2\cdot4-1=7$차. (min-jerk는 $k=3$ → 5차.)

  이는 변분법으로 $\int(x^{(k)})^2dt$를 최소화할 때 오일러–라그랑주 해가 요구하는 차수다.

### 최적화가 진짜 일하는 순간

- **한 구간:** 미지수 8 = 제약 8이라 $A$가 정방·가역, 해가 유일하게 $\mathbf{c}=A^{-1}\mathbf{b}$. 최적화할 여지가 없다.
- **여러 구간:** 미지수 $>$ 제약이라 자유도가 남고, $A\mathbf{c}=\mathbf{b}$만으론 해가 무한하다. 이때 남는 자유도를 "스냅 최소"로 채우는 것이 QP의 역할이다.

여기서 $Q$와 $A$가 서로의 빈틈을 정확히 메운다: $Q$는 고차(출렁임)를 누르고, $A$는 저차(어디서 출발·도착)를 못 박는다.

---

## 5. QP 정식화 — KKT 선형 시스템 한 방

**답할 질문: 제약 있는 이차 최소화를 어떻게 반복 없이 한 번에 푸는가?**

문제는 다음과 같다:

$$
\min_{\mathbf{c}}\ \tfrac12\mathbf{c}^\top Q\mathbf{c}\quad \text{s.t.}\quad A\mathbf{c}=\mathbf{b}
$$

$Q\succeq0$이고 제약이 선형 등식뿐이므로 이는 **Convex Optimization**다. Convex이라 국소해=전역해이고, heuristic 없이 결정론적으로 풀린다. (QP는 "이차 목적 + 선형 제약"인 별도 범주이지, 일반 비선형계획(NLP)이 아니다.)

라그랑지안으로 제약을 흡수하면:

$$
\mathcal{L}(\mathbf{c},\boldsymbol{\lambda})=\tfrac12\mathbf{c}^\top Q\mathbf{c}+\boldsymbol{\lambda}^\top(A\mathbf{c}-\mathbf{b})
$$

두 최적 조건을 세우면:

$$
\nabla_{\mathbf{c}}\mathcal{L}=Q\mathbf{c}+A^\top\boldsymbol{\lambda}=0,\qquad
\nabla_{\boldsymbol{\lambda}}\mathcal{L}=A\mathbf{c}-\mathbf{b}=0
$$

이 둘을 행렬로 쌓으면 **KKT 시스템**이 된다:

$$
\begin{bmatrix} Q & A^\top \\ A & 0 \end{bmatrix}
\begin{bmatrix} \mathbf{c} \\ \boldsymbol{\lambda} \end{bmatrix}
=
\begin{bmatrix} 0 \\ \mathbf{b} \end{bmatrix}
$$

선형 solve 한 번이면 $\mathbf{c}$가 나온다($\boldsymbol{\lambda}$는 부산물). 

이차 목적함수를 미분하면 정확히 1차(선형)가 되기 때문에 이 시스템이 선형으로 떨어진다 — 목적함수가 3차였다면 미분이 2차 방정식이 되어 이 성질이 무너진다.

통계로 보면 이는 **정규방정식의 "제약 있는" 사촌**이다. 

$X^\top X\boldsymbol{\beta}=X^\top\mathbf{y}$가 제약 없는 이차 최소화의 선형 조건이듯, KKT 시스템은 그 구조에 제약 블록 $(A, A^\top)$이 테두리로 붙은 형태다.

---

## 6. 마무리 — 이게 Primitive-Planner의 어디로 이어지나

min-snap은 "하나의 부드러운 궤적을 어떻게 만드는가"에 대한 답이다. 

Primitive-Planner는 이런 다항식 궤적을 미리 만들어 둔 **motion primitive**들의 라이브러리로 쓰고, 그중에서 충돌 없는 것을 빠르게 골라 잇는 방식으로 대규모 군집을 계획한다. 

즉 이 글의 다항식 파라미터화는 그 프리미티브가 "왜 이런 형태인지"의 토대다.

다음 Layer에서는 이 궤적에 **시간 최적 재파라미터화(TOPP-RA)**를 얹어, 같은 경로를 물리 한계 안에서 가장 빠르게 지나도록 만드는 문제로 넘어간다.

---

### 참고

- D. Mellinger, V. Kumar, *Minimum snap trajectory generation and control for quadrotors*, ICRA 2011.
- C. Richter, A. Bry, N. Roy, *Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments*, ISRR 2013.