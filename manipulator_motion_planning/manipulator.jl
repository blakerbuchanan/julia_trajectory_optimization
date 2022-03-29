"""
    Manipulator{T}

3DOF manipulator dynamics.

# Constructors
    Manipulator(; kwargs...)

with keyword arguments
* `Mi` - mass of the ith link, in kg (default = 1)
* `Ii` - inertia of the ith link, in kg-m^2 (default = 1)
* `mi` - mass of ith joint, in kg (default = 0.2)
* `Ji` - inertia of the ith joint, in kg-m^2 (default = 0.2)
* `li` - length of the ith link, in m (default = 0.5)
* `g` - gravity, in m/s² (default = 9.81)
"""
@autodiff struct Manipulator{T} <: ContinuousDynamics 
    M1::T
    M2::T
    I1::T
    I2::T
    m1::T
    m2::T
    J1::T
    J2::T
    J3::T
    l1::T
    l2::T
    g::T
    function Manipulator(M1, M2, I1, I2, m1, m2, J1, J2, J3, l1, l2, g)
        T = eltype(promote(M1, M2, I1, I2, m1, m2, J1, J2, J3, l1, l2, g))
        new{T}(M1, M2, I1, I2, m1, m2, J1, J2, J3, l1, l2, g)
    end
end

Cartpole(; mc=1.0, mp=0.2, l=0.5, g=9.81) = Cartpole(mc, mp, l, g)

function dynamics(model::Cartpole, x, u)
    mc = model.mc  # mass of the cart in kg (10)
    mp = model.mp   # mass of the pole (point mass at the end) in kg
    l = model.l   # length of the pole in m
    g = model.g  # gravity m/s^2

    q = x[ @SVector [1,2] ]
    qd = x[ @SVector [3,4] ]

    s = sin(q[2])
    c = cos(q[2])

    H = @SMatrix [mc+mp mp*l*c; mp*l*c mp*l^2]
    C = @SMatrix [0 -mp*qd[2]*l*s; 0 0]
    G = @SVector [0, mp*g*l*s]
    B = @SVector [1, 0]

    qdd = -H\(C*qd + G - B*u[1])
    return [qd; qdd]
end

function dynamics!(model::Cartpole, xdot, x, u)
    xdot .= dynamics(model, x, u)
end

RobotDynamics.state_dim(::Cartpole) = 4
RobotDynamics.control_dim(::Cartpole) = 1

# Base.position(::Cartpole, x::StaticVector) = SA[0,x[1],0]
# RobotDynamics.orientation(::Cartpole, x::StaticVector) = UnitQuaternion(expm(SA[1,0,0]*x[2]))