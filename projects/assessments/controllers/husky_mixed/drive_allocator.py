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

"""Wheel-only body feedback, preserving longitudinal authority under yaw saturation."""
import math


def clamp(x,lo,hi):return max(lo,min(hi,x))


def allocate(speed,yaw_rate,forward,measured_yaw,linear_integral,yaw_integral,config,dt=.016):
    if not all(math.isfinite(v) for v in (speed,yaw_rate,forward,measured_yaw,linear_integral,yaw_integral,dt)):
        raise ValueError('Nonfinite body-feedback input')
    linear_error=speed-forward
    yaw_error=yaw_rate-measured_yaw
    linear_integral=clamp(linear_integral+4*linear_error*dt,-2.,2.)
    yaw_integral=clamp(yaw_integral+6*yaw_error*dt/config['yaw_rate_gain'],-3.3,3.3)
    common=(speed+2*linear_error+linear_integral)/config['wheel_radius_m']
    differential=(yaw_rate/config['yaw_rate_gain']+yaw_integral)*config['half_track_m']/config['wheel_radius_m']
    limit=config['max_wheel_speed_radps']
    # Independent limits allow the high-traction side to slow or reverse.
    # The legacy mixer instead forced common toward zero at large differential.
    left=clamp(common-differential,-limit,limit)
    right=clamp(common+differential,-limit,limit)
    return left,right,linear_integral,yaw_integral


def lookahead_point(position,course,distance=.35):
    """Geometric pursuit of an explicit map-conditioned route, no body writes."""
    best=None
    for i,(a,b) in enumerate(zip(course,course[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        fraction=clamp(((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length),0,1)
        near=(a[0]+fraction*dx,a[1]+fraction*dy)
        error=math.hypot(position[0]-near[0],position[1]-near[1])
        if best is None or error<best[0]:best=(error,i,fraction,length)
    _,i,fraction,length=best
    remaining=(1-fraction)*length
    if distance<=remaining:
        a,b=course[i],course[i+1];u=fraction+distance/length
        return a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])
    distance-=remaining
    for a,b in zip(course[i+1:],course[i+2:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if distance<=length:
            u=distance/length;return a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])
        distance-=length
    return course[-1]
