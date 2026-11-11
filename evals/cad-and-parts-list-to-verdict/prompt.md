---
tags: [uav-virtual-build]
max_turns: 80
timeout_seconds: 3600
allowed_tools: [Read, Glob, Grep, Skill, Write, Edit, Bash]
---

we exported the stork vtol assembly from fusion and filled in the parts list with what flightory recommends. files are in examples/stork_vtol_flightory/ in the vbuild repo (stork_vtol_assembly.step, parts_list.csv, datasheets.yaml). datum: wing root leading edge is at CAD 0,5,335 mm, aft is +z, right is +x, up is +y. any part is allowed, no US-only restriction. base it on the stork phase0 plan, vtol variant. is the CG right and will it fly? quick fidelity is fine for now.
