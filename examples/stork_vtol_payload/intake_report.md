# CAD + parts list intake

Plan written: `examples/stork_vtol_payload/stork_vtol_payload_test.buildplan.yaml`

Frame: origin (wing-root leading edge) at CAD [0.0, 5.0, 335.0] mm; aft = CAD +z, right = CAD +x, up = CAD +y

Total mass 2294 g, CG at x/y/z = [14.9, 0.0, 10.5] mm (point-mass model; the mass stage refines it).

## Questions to settle before trusting the run

None.

## Parts matched

| id | part | plan component | CAD bodies | matched by | mass g | x mm | mass source |
|---|---|---|---|---|---|---|---|
| P001 | Fuselage print | fuselage_print | 1 | id P001 | 180.0 | 152.5 | slicer weight 180 g (geometry model would give 271 g) |
| P002 | Wing print | wing_print_R (mirrored pair) | 2 | id P002 | 308.0 | 85.0 | slicer weight 154 g (geometry model would give 183 g) |
| P003 | V-tail print | vtail_print_R (mirrored pair) | 2 | id P003 | 66.0 | 530.0 | slicer weight 33 g (geometry model would give 31 g) |
| P004 | Hatch | hatch_misc | 1 | id P004 | 15.0 | -150.0 | slicer weight 15 g (geometry model would give 10 g) |
| P010 | Main spar 10x1000 carbon tube | spar_10x8x1000 | 1 | id P010 | 44.0 | 57.5 | od/id/length x 1550 kg/m3 (1 mm wall assumed) |
| P011 | Secondary spar 8x1000 carbon tube | spar_8x6x1000 | 1 | id P011 | 34.0 | 120.0 | od/id/length x 1550 kg/m3 (1 mm wall assumed) |
| P012 | Wing spar 6x500 carbon tube | spar_6x4x500_R (mirrored pair) | 2 | id P012 | 24.0 | 80.0 | od/id/length x 1550 kg/m3 (1 mm wall assumed) |
| P013 | V-tail spars (8x155 + 6x265 x2 each) | tail_tubes | 1 | id P013 | 34.0 | 520.0 | estimate from tube sizes |
| P014 | Hinge rods 3 mm | hinge_rods | 0 | no CAD body | 20.0 | 240.0 | estimate |
| P015 | VTOL boom 10x10x500 square carbon | boom_R (mirrored pair) | 2 | id P015 | 56.0 | 50.0 | estimate from tube size |
| P020 | Lift motor pod (T-Motor F90 1300KV + 7in prop + mount) | lift_pod_FR (mirrored pair) | 2 | id P020 | 141.2 | -200.0 | 46.6 g motor (T-Motor) + 9 g prop + 15 g mount (estimate) |
| P020 | Lift motor pod (T-Motor F90 1300KV + 7in prop + mount) | lift_pod_RR (mirrored pair) | 2 | id P020 | 141.2 | 300.0 | 46.6 g motor (T-Motor) + 9 g prop + 15 g mount (estimate) |
| P021 | Pusher motor BrotherHobby Avenger 2812 V5 910KV | motor_cruise | 1 | id P021 | 76.5 | 650.0 | getfpv listing 76.5 g |
| P022 | Pusher prop APC 10x6E | prop_cruise | 0 | no CAD body | 20.0 | 668.0 | APC 10x6E ~20 g |
| P023 | Pusher ESC Lumenier 51A BLHeli_32 | esc_cruise | 1 | id P023 | 20.0 | 360.0 | estimate |
| P024 | Lift ESC 4-in-1 Velox V50A | lift_escs | 1 | id P024 | 25.0 | -100.0 | estimate |
| P030 | Battery 4S2P Molicel P50B | battery | 1 | id P030 | 600.0 | -210.0 | 8 x 69-71 g + nickel/leads |
| P040 | Flight controller SpeedyBee F405 Wing | flight_controller | 1 | id P040 | 35.0 | 20.0 | estimate |
| P041 | GPS Matek M10Q | gps | 1 | id P041 | 8.0 | 230.0 | estimate |
| P042 | Receiver Matek R24-D ELRS | telemetry_rx | 1 | id P042 | 3.0 | 110.0 | estimate |
| P043 | Video transmitter Walksnail Avatar VTX | vtx | 1 | id P043 | 30.0 | 140.0 | estimate |
| P044 | FPV camera Walksnail | camera | 1 | id P044 | 10.0 | -310.0 | estimate |
| P050 | Aileron servo EMAX ES08MA II | servo_aileron (mirrored pair) | 2 | id P050 | 24.0 | 58.0 | EMAX datasheet 12 g |
| P051 | Ruddervator servo EMAX ES08MA II | servo_ruddervator (mirrored pair) | 2 | id P051 | 24.0 | 420.0 | EMAX datasheet 12 g |
| P060 | Wiring harness | wiring | 0 | no CAD body | 50.0 | 80.0 | estimate |
| P061 | Lift motor wiring | lift_wiring | 0 | no CAD body | 45.0 | 50.0 | estimate |
| P062 | Hardware (inserts, screws, horns, pushrods, glue) | hardware | 0 | no CAD body | 40.0 | 60.0 | estimate |
| P070 | Inspection camera payload (HYPOTHETICAL test payload: not on the Flightory list) | inspection_camera | 0 | no CAD body | 220.0 | -300.0 | assumed test value |

## CAD geometry vs the base plan

| quantity | CAD | plan | |
|---|---|---|---|
| wing_span_m | 1.6201 | 1.62 | ok |
| fuselage_length_m | 0.975 | 0.975 | ok |
| fuselage_x_nose_m | -0.335 | -0.335 | ok |
| fuselage_width_m | 0.095 | 0.095 | ok |
| fuselage_height_m | 0.1 | 0.1 | ok |
