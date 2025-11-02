
# **SOME_BALL_STUFF**

70 =< θ =< 90
tan^-1(θ)
vball = d * sqrt(-9.8 / (2cos^2(θ)(d * tan((θ)) - (.6952))))

# **INERTIA CALCULATION**

m flywheel = .6699 kg
r flywheel = .048 m
inertia of flywheel = .5 * m flywheel * radius of flywheel^2
actual value: 0.0007717248

m ball = .075
inertia of ball = (m ball * radius of flywheel^2)
actual value: .000172

inertia reduction factor = (inertia of system)/(inertia of system + inertia of ball)
actual value: 0.81774347776

# **MOTOR SPEED CALCULATION**

inertia reduction factor = .81774347776

rad/s = vball / (inertia reduction factor * radius of flywheel)
actual value: vball / 0.03888

tps = (28 * (rad/s)) / (2 * Math.Pi)

# _**Speed to run the motor: (28 * vball) / 0.244290244743**_