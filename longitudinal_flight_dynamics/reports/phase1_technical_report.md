# Longitudinal Design — Cessna 182 Pitch-Rate Feedback (Phase 1)

## 1. Start from the open-loop longitudinal model

The Cessna 182 longitudinal model in standard state-space form is:

$$
\dot{x} = A x + B \delta_e
$$

with the state vector

$$
x =
\begin{bmatrix}
u \\
\alpha \\
q \\
\theta
\end{bmatrix}
$$

where:

- $u$ = forward-speed perturbation
- $\alpha$ = angle-of-attack perturbation
- $q$ = pitch rate
- $\theta$ = pitch angle

For the cruise-condition model, the numerical matrices are:

$$
A =
\begin{bmatrix}
-0.0456 & 19.4590 & 0 & -32.1740 \\
-0.001314 & -2.092534 & 0.970632 & 0 \\
0.003342 & -13.938686 & -6.805318 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
$$

$$
B =
\begin{bmatrix}
0 \\
-0.202562 \\
-34.735884 \\
0
\end{bmatrix}
$$

The open-loop poles are the eigenvalues of $A$:

$$
\lambda_{1,2} = -4.4497 \pm 2.8249i
$$

$$
\lambda_{3,4} = -0.0220 \pm 0.1698i
$$

These correspond to the short-period and phugoid longitudinal modes.

---

## 2. What pitch-rate feedback means

Pitch-rate feedback uses the measured pitch rate $q$ to modify the elevator deflection.

The feedback law used in this project is:

$$
\delta_e = \delta_{cmd} + K_q q
$$

where:

- $\delta_{cmd}$ = commanded elevator input
- $K_q$ = pitch-rate feedback gain
- $q$ = measured pitch rate

This control law is chosen to improve the damping of the longitudinal response, especially the short-period mode.

Because the elevator input matrix for this model has a negative pitch-rate channel coefficient, the stabilizing sign convention for this specific model is written with the plus sign above.

---

## 3. Closed-loop state equation

The pitch rate can be written as an output of the state vector:

$$
q = C_q x
$$

with

$$
C_q = \begin{bmatrix} 0 & 0 & 1 & 0 \end{bmatrix}
$$

Substituting the control law into the open-loop model gives:

$$
\dot{x} = A x + B(\delta_{cmd} + K_q q)
$$

$$
\dot{x} = A x + B(\delta_{cmd} + K_q C_q x)
$$

$$
\dot{x} = (A + B K_q C_q)x + B \delta_{cmd}
$$

Therefore, the closed-loop state matrix is:

$$
A_{cl} = A + B K_q C_q
$$

If we define

$$
K = \begin{bmatrix} 0 & 0 & K_q & 0 \end{bmatrix}
$$

then the same expression becomes:

$$
A_{cl} = A + B K
$$

and the closed-loop system is

$$
\dot{x} = A_{cl}x + B \delta_{cmd}
$$

---

## 4. Transfer-function view for root locus

For root-locus design, the most useful SISO channel is the elevator-to-pitch-rate transfer function:

$$
G_q(s) = \frac{q(s)}{\delta_e(s)} = C_q (sI - A)^{-1} B
$$

This transfer function shows how the pitch rate responds to elevator input.

It is the correct channel for designing a pitch-rate damper because the feedback law uses $q$ directly.

A second useful channel is the elevator-to-pitch-angle transfer function:

$$
G_{\theta}(s) = \frac{\theta(s)}{\delta_e(s)} = C_{\theta}(sI - A)^{-1} B
$$

with

$$
C_{\theta} = \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix}
$$

The root locus is constructed using the pitch-rate channel because it is the primary feedback variable in this design.

---

## 5. Characteristic equation for the closed loop

Starting from the pitch-rate transfer function,

$$
q(s) = G_q(s)\delta_e(s)
$$

and the control law,

$$
\delta_e(s) = \delta_{cmd}(s) + K_q q(s)
$$

we obtain

$$
q(s) = G_q(s)\left(\delta_{cmd}(s) + K_q q(s)\right)
$$

Rearranging,

$$
q(s)\left(1 - K_q G_q(s)\right) = G_q(s)\delta_{cmd}(s)
$$

Therefore, the closed-loop transfer function is

$$
\frac{q(s)}{\delta_{cmd}(s)} =
\frac{G_q(s)}{1 - K_q G_q(s)}
$$

and the closed-loop characteristic equation is

$$
1 - K_q G_q(s) = 0
$$

MATLAB root-locus tools are usually formulated for:

$$
1 + K G(s) = 0
$$

so the standard practical choice is to define

$$
G_{rl}(s) = -G_q(s)
$$

and then use the root locus of $G_{rl}(s)$.

---

## 6. What the root locus is telling you

The root locus shows how the closed-loop poles move as the feedback gain $K_q$ changes.

At:

$$
K_q = 0
$$

the poles are the open-loop poles.

As $K_q$ increases:

- the short-period poles move in the complex plane
- damping and response speed change
- the phugoid mode may also move slightly

The root locus helps determine whether the feedback gain makes the aircraft:

- faster
- more damped
- less oscillatory
- still physically reasonable

The key point is that the root locus is a pole-migration map for the controlled aircraft.

---

## 7. Quantitative design criteria

The pole locations are interpreted using standard second-order metrics.

For a complex pole pair

$$
s = \sigma \pm j\omega_d
$$

the natural frequency is

$$
\omega_n = \sqrt{\sigma^2 + \omega_d^2}
$$

the damping ratio is

$$
\zeta = \frac{-\sigma}{\omega_n}
$$

and the approximate settling time is

$$
T_s \approx \frac{4}{|\sigma|}
$$

These metrics are used to judge whether the closed-loop response is improving.

Typical design goals for the pitch-rate damper are:

- move the short-period poles farther left
- increase damping
- reduce oscillation in $q$ and $\alpha$
- keep the phugoid acceptable
- avoid excessive elevator demand

---

## 8. How to choose $K_q$ from the root locus

A practical root-locus design procedure is:

### Step 1
Plot the root locus of the pitch-rate channel.

### Step 2
Follow the branch associated with the short-period poles.

### Step 3
Choose a gain that moves the short-period poles to a better location, meaning:

- more negative real part
- adequate damping ratio
- smooth time response

### Step 4
Check that the phugoid poles do not move to an undesirable location.

### Step 5
Validate the chosen gain using:

- open-loop vs closed-loop pole comparison
- step responses
- elevator demand

The gain is not chosen from the root locus alone. It is chosen from the combination of:

- pole movement
- response quality
- control effort

---

## 9. What happens in your model as $K_q$ increases

For this Cessna 182 model, increasing $K_q$ has the following general effects:

- the short-period mode becomes more damped
- the rapid pitch response becomes less oscillatory
- $q$ settles faster
- the phugoid changes more slowly than the short-period

For example:

### Open loop, $K_q = 0$

$$
-4.4497 \pm 2.8249i, \qquad -0.0220 \pm 0.1698i
$$

### Moderate gain, $K_q = 0.05$

$$
-5.3175 \pm 1.7294i, \qquad -0.0226 \pm 0.1598i
$$

### Larger gain, $K_q = 1$

Approximately:

$$
-41.27,\qquad -2.36,\qquad -0.0268 \pm 0.0875i
$$

### High gain used in the final linear study, $K_q = 5$

Approximately:

$$
-180.49,\qquad -2.077,\qquad -0.0287 \pm 0.0367i
$$

So, as the gain becomes large, the short-period pair can become heavily damped and may split into real poles.

This explains why larger gains can produce a very smooth closed-loop response in the linear model.

---

## 10. A sensible gain range for your model

A reasonable early gain-search range for this aircraft is:

$$
0.03 \le K_q \le 0.06
$$

This is a useful starting range because:

- the short-period mode clearly improves
- the poles remain easy to interpret
- the response is refined without becoming extremely stiff

However, the final linear-study result showed that larger values can also work well.

In this project, the gain

$$
K_q = 5
$$

produced:

- a strongly damped closed-loop response
- a smooth transient
- modest elevator demand

So the final acceptable gain range is determined by simulation results, not by an arbitrary fixed interval such as $0 < K_q < 1$.

---

## 11. Open-loop vs closed-loop equations

### Open-loop system

$$
\dot{x} = A x + B \delta_e
$$

Poles are found from:

$$
\det(sI - A) = 0
$$

### Closed-loop system with pitch-rate feedback

$$
\delta_e = \delta_{cmd} + K_q q
$$

$$
q = C_q x
$$

Therefore,

$$
\dot{x} = (A + B K_q C_q)x + B \delta_{cmd}
$$

or equivalently,

$$
\dot{x} = A_{cl}x + B \delta_{cmd}
$$

with

$$
A_{cl} = A + B K
$$

where

$$
K = \begin{bmatrix} 0 & 0 & K_q & 0 \end{bmatrix}
$$

The closed-loop poles are the eigenvalues of $A_{cl}$.

---

## 12. Engineering rule behind the choice

The engineering rule is simple:

> Choose $K_q$ so that the closed-loop poles provide better damping and faster settling, while keeping the control effort physically reasonable.

In practice, that means:

- improve the short-period mode first
- do not damage the phugoid mode
- verify the time response
- verify the elevator demand

A gain that gives excellent poles but unrealistic elevator motion is not a good design.
A gain that improves both the poles and the control effort is a good design.

That is why the final gain selection must be based on both:

- pole placement
- time-domain behavior

---

## 13. Design conclusion for Phase 1

For the Cessna 182 linear longitudinal model, pitch-rate feedback is an effective way to improve dynamic damping.

The root locus shows how the feedback gain moves the poles, while the time-domain response shows the practical effect on:

- $u$
- $\alpha$
- $q$
- $\theta$

The chosen gain for the final linear study was:

$$
K_q = 5
$$

because it produced:

- stable closed-loop poles
- strongly reduced oscillation
- smooth state responses
- acceptable elevator demand

This makes it a defensible final gain for the Phase 1 linear control-design study.

