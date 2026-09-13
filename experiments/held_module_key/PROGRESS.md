# Independent held-module key geometry candidate

User scope: fixed-socket v1/v2 frozen, GPU 0, no HIGH, training, P2, large evaluation, or H-advantage assertion. Seeds here are development only.

The new files reuse frozen RoboTwin loading, robot motion queues, allow-listed camera adapter, and recording helpers without modifying them. `PRESERVED_CONTROLS.json` records all 9,331 pre-existing source/result files, including failed runs and videos.

Physical design: original 76 × 64 × 16 mm body and identical handle; a 32 × 20 × 12 mm bottom key with an asymmetric side tab; key translation and yaw vary within the body footprint. Fixed socket has a matching real L aperture with 2.5 mm clearance. The four colored patches are non-colliding paint on the actual key face. Removing paint would leave the same mechanical constraint. An elevated two-edge support leaves the bottom face physically open for putdown/self-observation. No grasp welds, hidden pose injection, selective camera masks, or disabled key collisions.

Initial development attempts:

- `dev400_translate_a/b/c`: raising the module center to 1.04 m made the initial IK branch collide between fr_link3 and fr_link5. The instrumented first step measured a 470 N·s contact and the module was ejected. All failures retained. The initialization now checks robot collision before settling.
- Returning to the previously workable module center height 0.94 m gives a contact-held stable relative pose. The settled grasp differs by roughly 16 degrees from the nominal design; this is **not** injected as a calibration. Visual perception directly estimates the full key-to-EE pose.
- `dev400_translate_d`: physical 7 cm translation and return, no key estimate from any camera. Relative drift during this route is 0.0665 mm / 0.00533 degrees. Nonzero solver velocities are logged; this is approximate pose stability, not a claim of zero vibration.
- `dev400_helper_a`: high candidate could not see the bottom. `helper_b`: shortest lower candidate was still too oblique. Both complete negative videos retained.
- `dev400_helper_c`: next pre-existing low candidate gets valid left-wrist RGB-D information at 4.3 s and returns the helper at 9.196 s. GT-only first-pose error 1.59 mm / 0.42 degrees. This establishes feasibility only.
- `dev400_head_a`: the first bounded robot-only list had no collision-free straight joint path; 72 candidates collided fr_link3/fr_link5 and 8 IK failures. This does not prove head presentation impossible. Additional public positions are being tried.
- `dev400_passive_left_a`: initial fixed-left optical-axis presentation list had no IK-feasible route. The passive left camera looks down into its own left workspace; it is never moved just to make S convenient, nor is its input masked. Alternative off-axis placements and IK branches remain open.

Timing: t=0 after the same three-second physical contact initialization. Physics runs at 4 ms, RGB-D/video at 10 Hz. Observation is accepted after two consistent frames in the moving EE frame. Every route may use head/left/right. Controllers stop a dedicated observation when information is sufficient. Full installation uses the same visually estimated key-to-EE transform and fixed public socket frame. Helper recovery and right installation run through independent queues with one physics step per tick.
