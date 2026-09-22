# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""SI-unit reduced models. See ../REALISM.md for sources and assumptions.

The engine resolves contacts. This module never adds impact impulses or moves
bodies. Damage loads are estimates: the public contact API lacks force/normal.
"""
from dataclasses import dataclass, replace
import math


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def local(rotation, vector):
    return [sum(rotation[3*j+i]*vector[j] for j in range(3)) for i in range(3)]


def world(rotation, vector):
    return [dot(rotation[3*i:3*i+3], vector) for i in range(3)]


def unit(vector):
    length = math.sqrt(dot(vector, vector))
    return [v/length for v in vector] if length > 1e-10 else [1, 0, 0]


def box_inertia(mass, size):
    x, y, z = size
    return (mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12)


def cylinder_inertia(mass, radius, length):
    transverse = mass*(3*radius*radius+length*length)/12
    return (transverse, transverse, mass*radius*radius/2)


@dataclass(frozen=True)
class Part:
    mass: float
    inertia: tuple
    size: tuple
    stiffness: float
    yield_force: float
    plastic_travel: float
    cylinder: bool = False

    @property
    def fracture_work(self):
        return self.yield_force*self.plastic_travel


# Masses are a design specification, not weighed real robots. Inertias use
# homogeneous primitive approximations; decorations do not add hidden mass.
# Shafts: circular section modulus pi*d^3/32, assumed 350 MPa yield steel.
WHEEL = Part(2.4, cylinder_inertia(2.4, .17, .122), (.17, .122),
             1.5e6, 350e6*math.pi*.020**3/(32*.065), .05, True)
CHASSIS = Part(48, box_inertia(48, (1.12, .78, .30)), (1.12, .78, .30),
               6e6, 350e6*(.12*.012**2/6)/.04, .04)
BAR = Part(8, box_inertia(8, (.99, .15, .075)), (.99, .22, .09),
           8e6, 650e6*math.pi*.035**3/(32*.075), .04)
DRUM = Part(8, cylinder_inertia(8, .176, .49), (.203, .49),
            8e6, BAR.yield_force, .04, True)


def part_spec(robot, part, loadouts=None):
    if loadouts is None:
        return WHEEL if part.endswith('wheel') else CHASSIS if part == 'chassis' else DRUM if robot == 'RAZOR' else BAR
    from foundry_config import FRAMES
    settings = loadouts[robot]
    frame_mass, shaft_scale = FRAMES[settings['frame']]
    base = WHEEL if part.endswith('wheel') else CHASSIS if part == 'chassis' else DRUM if settings['weapon'] == 'drum' else BAR
    mass = base.mass if part.endswith('wheel') else frame_mass if part == 'chassis' else settings['rotor_kg']
    # Package assumptions: shaft diameter scales bending yield by d^3 and
    # bending stiffness by d^4. Additional bracing mass is in the frame package.
    return replace(base, mass=mass, inertia=tuple(v*mass/base.mass for v in base.inertia),
                   stiffness=base.stiffness*shaft_scale**4, yield_force=base.yield_force*shaft_scale**3)


@dataclass(frozen=True)
class Motor:
    reduction: float
    current_limit: float
    efficiency: float = .9
    resistance: float = .064
    kv_rpm_v: float = 256.9
    no_load_current: float = 4.4

    @property
    def k(self):
        # Kt = Ke in SI; manufacturer's rounded Kt agrees within 0.1%.
        return 60/(math.tau*self.kv_rpm_v)

    def evaluate(self, duty, omega, voltage=24., coast=False):
        rotor_speed = self.reduction*omega
        current = 0. if coast else max(-self.current_limit, min(self.current_limit,
                                    (duty*voltage-self.k*rotor_speed)/self.resistance))
        no_load_speed = (24-self.no_load_current*self.resistance)/self.k
        drag = self.k*self.no_load_current*rotor_speed/no_load_speed
        shaft_torque = self.k*current-drag
        # Forward and backdriven transmission losses both dissipate energy.
        efficiency = self.efficiency if shaft_torque*rotor_speed >= 0 else 1/self.efficiency
        torque = shaft_torque*self.reduction*efficiency
        # Current limiting is PWM chopping: terminal voltage falls when the
        # limit engages. duty*I would incorrectly charge full bus power at stall.
        terminal_voltage = current*self.resistance+self.k*rotor_speed
        electrical_power = terminal_voltage*current
        mechanical_power = torque*omega
        return {'torque_nm':torque, 'current_a':current, 'omega_rad_s':omega,
                'electrical_w':electrical_power, 'mechanical_w':mechanical_power,
                'loss_w':max(0., electrical_power-mechanical_power)}


DRIVE = Motor(19.2, 40.)
WEAPON = {'ANVIL':Motor(8.26, 100.), 'RAZOR':Motor(4.38, 100.)}


def drive_motor(settings):
    from foundry_config import GEARING
    return replace(DRIVE, reduction=GEARING[settings['gearing']])


def weapon_motor(settings):
    return WEAPON['RAZOR' if settings['weapon'] == 'drum' else 'ANVIL']


def powertrain(requests, dt, battery_j):
    """Shared 24 V bus, 12 mOhm sag; regeneration goes to a brake resistor.

    requests maps names to (Motor, duty, measured relative speed, coast).
    Battery capacity/resistance/current limits are authored design assumptions.
    """
    if dt <= 0 or not math.isfinite(dt):
        raise ValueError('positive finite timestep required')
    charged = battery_j > 0
    def sample(voltage):
        values = {key:m.evaluate(duty if charged else 0, omega, voltage, coast or not charged)
                  for key,(m,duty,omega,coast) in requests.items()}
        watts = sum(max(0, v['electrical_w']) for v in values.values())
        return values, watts
    # Stable solve of V = Voc - R*P(V)/V; bounded, small, CPU-only.
    lo, hi = 0., 24. if charged else 0.
    for _ in range(24):
        voltage = (lo+hi)/2
        _, watts = sample(voltage)
        if voltage + .012*watts/max(voltage, 1e-9) > 24:
            hi = voltage
        else:
            lo = voltage
    voltage = (lo+hi)/2
    values, watts = sample(voltage)
    current = watts/max(voltage, 1e-9)
    drawn_j = 24*current*dt
    # No final partial tick can create energy from an exhausted pack.
    if drawn_j > battery_j and charged:
        values = {key:m.evaluate(0, omega, 0, True)
                  for key,(m,_,omega,_) in requests.items()}
        voltage = current = drawn_j = 0.
        battery_j = 0.
    return values, {'voltage_v':voltage, 'current_a':current,
                    'remaining_j':max(0., battery_j-drawn_j)}


def kinetic(spec, velocity, rotation):
    omega = local(rotation, velocity[3:])
    return .5*spec.mass*dot(velocity[:3], velocity[:3]) + .5*sum(i*w*w for i,w in zip(spec.inertia,omega))


def body_record(robot, part, position, velocity, rotation, loadouts=None):
    spec = part_spec(robot, part, loadouts)
    p = [spec.mass*v for v in velocity[:3]]
    spin = world(rotation, [i*w for i,w in zip(spec.inertia,local(rotation,velocity[3:]))])
    angular = [a+b for a,b in zip(cross(position,p),spin)]
    return {'mass_kg':spec.mass, 'energy_j':kinetic(spec,velocity,rotation),
            'momentum':p, 'angular_momentum':angular}


def surface_normal(spec, position, rotation, point):
    """Outward normal of a proxy surface, explicitly NOT solver readback."""
    p = local(rotation,[a-b for a,b in zip(point,position)])
    if spec.cylinder:
        radius, length = spec.size
        radial = math.hypot(*p[:2])
        if abs(abs(p[2])-length/2) < abs(radial-radius):
            normal = [0,0,math.copysign(1,p[2])]
        else:
            normal = unit([p[0],p[1],0])
    else:
        distances = [abs(abs(p[i])-spec.size[i]/2) for i in range(3)]
        axis = distances.index(min(distances))
        normal = [0.,0.,0.]
        normal[axis] = math.copysign(1,p[axis])
    return world(rotation,normal)


def contact_load(a, b, point, normal, assembly_masses, loadouts=None):
    """Reduced articulated effective mass along an estimated normal.

    a/b = (robot, part, position, velocity, rotation). Non-chassis spin is
    constrained to its hinge axis; chassis rotational compliance is omitted.
    This approximation must not be presented as measured contact force.
    """
    from foundry_logic import point_velocity
    va = point_velocity(a[3],a[2],point)
    vb = point_velocity(b[3],b[2],point)
    closing = max(0., -dot([x-y for x,y in zip(va,vb)],normal))
    inverse_mass = 0.
    for name,part,position,velocity,rotation in (a,b):
        spec = part_spec(name,part,loadouts)
        inverse_mass += 1/assembly_masses[name]
        if part != 'chassis':
            axis = world(rotation,[0,0,1])  # wheel/drum cylinder and blade z
            lever = [x-y for x,y in zip(point,position)]
            inverse_mass += dot(cross(lever,normal),axis)**2/spec.inertia[2]
    effective_mass = 1/inverse_mass
    energy = .5*effective_mass*closing**2
    sa,sb = part_spec(*a[:2],loadouts),part_spec(*b[:2],loadouts)
    stiffness = 1/(1/sa.stiffness+1/sb.stiffness)
    return {'closing_m_s':closing, 'effective_mass_kg':effective_mass,
            'incident_j':energy, 'estimated_peak_n':math.sqrt(2*stiffness*energy),
            'estimated_duration_s':math.pi*math.sqrt(effective_mass/stiffness)}
