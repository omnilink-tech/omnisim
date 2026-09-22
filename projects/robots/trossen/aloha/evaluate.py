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
# limitations under the License."""Contact-based carry and full-cylinder slot containment checks."""
import math

FINGERS = {'right_left_finger_link', 'right_right_finger_link'}


def cylinder_in_slot(position, battery_rotation, remote_rotation):
    """Test cylinder extents in the remote frame, not only its centre."""
    axis = [sum(remote_rotation[j * 3 + i] * battery_rotation[j * 3 + 2]
                for j in range(3)) for i in range(3)]
    extent = [.0245 * abs(a) + .0068 * math.sqrt(max(0, 1 - a * a)) for a in axis]
    lo, hi = [-.0095, -.0175, .006], [.0445, -.001, .021]
    return (abs(axis[0]) > .98 and
            all(position[i] - extent[i] >= lo[i] - .0005 and
                position[i] + extent[i] <= hi[i] + .0005 for i in range(3)))


def evaluate(rows, final, fps, initial_z):
    longest = current = 0
    origin = None
    slip = 0.0
    start_position = None
    transport = 0.0
    for row in rows:
        contact = set(row['contacts'])
        if (FINGERS <= contact and not contact.intersection({'table', 'remote'})
                and row['battery_position'][2] > initial_z + .025):
            current += 1
            origin = origin or row['battery_in_gripper']
            start_position = start_position or row['battery_position']
            transport = max(transport, math.dist(start_position[:2], row['battery_position'][:2]))
            slip = max(slip, math.dist(origin, row['battery_in_gripper']))
            longest = max(longest, current)
        else:
            current, origin, start_position = 0, None, None
    contained = cylinder_in_slot(final['final_battery_in_remote'],
                                 final['final_battery_rotation'],
                                 final['final_remote_rotation'])
    velocity = final['final_battery_velocity']
    settled = math.dist(velocity[:3], [0, 0, 0]) < .005 and math.dist(velocity[3:], [0, 0, 0]) < .05
    released = bool(rows) and all(not FINGERS.intersection(r['contacts']) for r in rows[-round(fps / 2):])
    # Carry means sustained bilateral contact while transporting the battery.
    # Grip stability is a separate, stricter quality check; report slip even
    # when the battery remains held and the task completes.
    carried = longest / fps >= .5 and transport >= .08
    left_brace = sum(bool({'left_left_finger_link','left_right_finger_link'} & set(r.get('remote_contacts',[]))) for r in rows) / fps
    return {'bilateral_airborne_s': longest / fps, 'max_carry_slip_m': slip,
            'bilateral_transport_m': transport, 'strict_grip_stability_passed': slip <= .01 and carried,
            'left_brace_contact_s': left_brace,
            'carried': carried, 'inside_slot': contained, 'settled': settled,
            'released': released, 'task_success': carried and contained and settled and released}


def evaluate_insertion(rows, result, fps):
    """Require mechanical compression and both terminal contacts after release."""
    def tilt(row):
        axis_z=sum(row['remote_rotation'][j*3+2]*row['battery_rotation'][j*3+2] for j in range(3))
        return math.asin(max(-1,min(1,axis_z)))
    def contained(row):
        return cylinder_in_slot(row['battery_in_remote'],row['battery_rotation'],row['remote_rotation'])
    compressed=[r for r in rows if r.get('spring_compression_m',0)>=.005
                and 'negative_terminal' in r['contacts'] and FINGERS<=set(r['contacts'])
                and tilt(r)>=math.radians(10)]
    final_rows=rows[-round(fps*.5):]
    both={'negative_terminal','positive_terminal'}
    seated=bool(final_rows) and all(both<=set(r['contacts']) and r['spring_compression_m']>=.003
                                   and contained(r) and abs(tilt(r))<math.radians(4)
                                   and r.get('floor_contact_points') and abs(r['battery_in_remote'][2]-.0128)<.0007 for r in final_rows)
    compressed_before_release=len(compressed)>=max(1,round(fps*.1))
    seated_while_held=compressed_before_release and any(r['frame']>compressed[0]['frame'] and contained(r)
                                      and FINGERS<=set(r['contacts']) and abs(tilt(r))<math.radians(4) for r in rows)
    pressing=[r for r in rows if r.get('stage') in ('press','press_withdraw') and FINGERS.intersection(r['contacts'])]
    lowering=(pressing[0]['battery_in_remote'][2]-min(r['battery_in_remote'][2] for r in pressing)) if pressing else 0
    # The spring can finish seating just after the fingertip clears. Require
    # that transition within 0.5 s of contact, not an unrelated later drop.
    press_end=pressing[-1]['frame'] if pressing else -1
    press_seating=[r for r in rows if press_end<=r['frame']<=press_end+round(fps*.5)
                   and contained(r) and both<=set(r['contacts'])]
    pressed_to_seat=compressed_before_release and lowering>=.001 and bool(press_seating)
    depth_measured=bool(rows) and all(all(field in r and math.isfinite(r[field]) for field in
                                    ('negative_contact_max_depth_m','housing_contact_max_depth_m')) for r in rows)
    peak_depth=max((r.get('negative_contact_max_depth_m',0) for r in rows),default=0)
    housing_depth=max((r.get('housing_contact_max_depth_m',0) for r in rows),default=0)
    shallow_contact=depth_measured and max(peak_depth,housing_depth)<=.0015
    return {'placement_success':result['task_success'],'spring_compression_peak_m':max((r['spring_compression_m'] for r in rows),default=0),
            'spring_compressed_while_held':compressed_before_release,'seated_before_release':bool(seated_while_held),
            'pressed_to_seat':bool(pressed_to_seat),'tool_press_lowering_m':lowering,
            'both_terminals_after_release':seated,
            'negative_contact_peak_depth_m':peak_depth,'contact_depth_measured':depth_measured,
            'housing_contact_peak_depth_m':housing_depth,
            'contact_depth_passed':shallow_contact,
            'task_success':result['carried'] and compressed_before_release and (seated_while_held or pressed_to_seat) and seated
                           and result['inside_slot'] and result['released'] and result['settled'] and shallow_contact,
            'success_scope':'Mechanically compressed and seated; electrical conductivity is not simulated'}
