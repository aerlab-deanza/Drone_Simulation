# Theory

Each page explains the idea behind one part of the simulator, in layers:

1. **In plain words**: a short summary with no equations.
2. **The key idea**: the one or two equations the code actually uses, with
   every symbol explained.
3. **Show the derivations**: the full proofs, folded away. Click to expand.
4. **Why it matters for the drone**: what the result changes in the
   simulator.

You can stop after any layer. Unsure what a symbol means? See the
{doc}`notation` page.

The pages build on each other in this order:

1. {doc}`frames`: which way is "up", and what an accelerometer really measures.
2. {doc}`euler-angles`: roll, pitch, yaw, and why they fail at 90° pitch.
3. {doc}`quaternions`: the four-number attitude that never fails.
4. {doc}`dynamics`: the physics that moves the drone, and how it is stepped
   forward in time.
5. {doc}`imu-noise`: why sensor errors make angles and position drift.
6. {doc}`madgwick`: correcting drift by nudging toward gravity.
7. {doc}`ekf`: correcting drift while tracking uncertainty, and what an IMU can
   never see.
8. {doc}`pid`: the three stacked flight-control loops.
9. {doc}`lqr`: the "optimal" controller, and where it stops being optimal.
