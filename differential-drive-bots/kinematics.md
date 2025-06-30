# Differential Drive Bot Kinematics

A **Differential Wheeled Robot** is a mobile robot whose movement is based on $2$ separately driven wheeels placed on either side of the robot body. It can thus change its direction by varying the relative rate of rotation of its wheels and hence does not require an additional steering motion.

### Notation

$$
    \begin{align}
        (X,Y) :& \text{ global coordinate system } \\
        (X_b,Y_b) :& \text{ local body coordinate system } \\
        \phi :& \text{ orientation of the bot w.r.t. global coordinates } \\
        r :& \text{ radius of the wheel } \\
        b :& \text{ width of the vehicle } \\
        ICR :& \text{ instantaneous center of rotation } \\
        R :& \text{ distance to the ICR from the bot's center } \\
        v_l :& \text{ ground contact speed on the left } \\
        v_r :& \text{ ground contact speed on the right } \\
        \omega :& \text{ angular velocity }
    \end{align}
$$

### Derivations

According to the definition of *angular velocity*,

$$
    \omega(R+b/2) = v_r \\
    \omega(R-b/2) = v_l \\
$$

solving for $\omega$ and $R$ we get,

$$
    \omega = (v_r - v_l)/b \\[1em]
    R = {b \over 2}{(v_r + v_l) \over {v_r  v_l}} \\[1em]
$$

Instantaneous velocity, 

$$
    V = \omega R = (v_r + v_l)/2
$$

The wheel tangential velocities are,

$$
    v_r = r\omega_r \\
    v_l = r\omega_l
$$

### Translation

Kinematics in local body coordinates,

$$
    \begin{bmatrix}
        \dot x_b \\ \dot y_b \\ \dot\phi
    \end{bmatrix}
    =
    \begin{bmatrix}
        v_{xb} \\ v_{yb} \\ \omega
    \end{bmatrix}
    =
    \begin{bmatrix}
        {r\over2} & {r\over2} \\
        0 & 0 \\
        -{r\over b} & {r\over b}
    \end{bmatrix}
    \begin{bmatrix}
        \omega_l \\ \omega_r
    \end{bmatrix}
$$

Kinematics in global coordinate system,

$$
    \begin{bmatrix}
        \dot x \\ \dot y \\ \dot\phi
    \end{bmatrix}
    =
    \begin{bmatrix}
        \cos\phi & 0 \\
        sin\phi & 0 \\
        0 & 1
    \end{bmatrix}
    \begin{bmatrix}
        V \\ \omega
    \end{bmatrix}
$$