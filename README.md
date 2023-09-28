# embedded_PID

Embedded systems programming tutorial. The example in question is that of a 
DC Motor:

![alt text](./docs/motor.png "DC Motor")

## Phisical Parameters

* (J)     moment of inertia of the rotor     0.01 kg.m^2
* (b)     motor viscous friction constant    0.1 N.m.s
* (Ke)    electromotive force constant       0.01 V/rad/sec
* (Kt)    motor torque constant              0.01 N.m/Amp
* (R)     electric resistance                1 Ohm
* (L)     electric inductance                0.5 H

**Constitutive Equations**

$$ J\ddot{\theta} + b\dot{\theta} = Ki $$

$$ L \frac{d i}{dt} + Ri = V - K\theta $$

**The Transfer Function Description**
$$P(s) = \frac{\dot{\Theta}(s)}{V(s)} = \frac{K}{(Js+b)(Ls+R) + K^2} $$
where $K = K_e = K_t$
