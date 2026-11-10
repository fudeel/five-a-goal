# CalcioFive — Development TODO

Phase 1 is **one human against AI (or with AI teammates)** on a five-a-side pitch,
written in **Rust**, with realistic ball physics and a physically simulated goal net.
Only what is listed in this file is in scope for this phase.

**Status:** Phase 1 (single-player gameplay) is complete. Phase 2 (realistic animation,
characters, full action set, key bindings) is in progress — see the Phase 2 section below.

---

## Game rules & design decisions for this phase

| Topic | Decision |
|---|---|
| Format | Team size selectable **1v1, 2v2, 3v3, 4v4, 5v5** (5v5 is the main format). |
| Control rule | **The human always controls exactly one footballer** for the whole match (no switching). Every other player on both teams is AI. |
| Goalkeepers | 3v3, 4v4 and 5v5 have one AI goalkeeper per team. 1v1 and 2v2 have no dedicated goalkeeper. |
| Pitch | 40 m × 20 m indoor/cage pitch (futsal size) enclosed by **rebound boards**: the ball is always in play. |
| Goals | 3 m wide × 2 m high, ~1 m deep, with a cloth-simulated net. |
| Ball | Size 5: radius 0.11 m, mass 0.43 kg. |
| Units | SI (metres, seconds, kilograms). World axes: **x** = pitch length (goals at x = ±20), **y** = up, **z** = pitch width (touchlines at z = ±10). |
| Physics rate | Fixed timestep **240 Hz** for ball, net and players. Rendering at monitor rate. |
| Engine | `calcio_core`: pure Rust, **zero dependencies**, holds all physics/gameplay/AI and is unit-tested headless. `calcio_game`: thin renderer + input using **macroquad 0.4**. |

---

## Git workflow (for every agent working on this repo)

1. Every step below has its own branch: `step/NN-short-name`.
2. Start a step: `git checkout main && git checkout -b step/NN-short-name`.
3. Small, focused commits. Run `cargo test -p calcio_core` and `cargo build` before finishing.
4. When the step is finished: tick its boxes in this file, commit, then
   `git checkout main && git merge --no-ff step/NN-short-name`.
5. `main` always contains only finished steps. An unfinished step lives only in its branch:
   continue it from there.
6. Handoff: when work is produced outside this machine, all branches are delivered as
   `handoff/five-a-goal.bundle`. Import them with:
   `git fetch --update-head-ok handoff/five-a-goal.bundle "refs/heads/*:refs/heads/*"` then `git reset`.

---

## Progress

### Step 00 — Plan & repo hygiene · `step/00-plan`
- [x] Detailed TODO (this file)
- [x] README with project overview
- [x] `.gitignore` (target/, IDE folders), `.gitattributes` (LF line endings), `rustfmt.toml`

### Step 01 — Cargo workspace & game loop skeleton · `step/01-workspace`
- [x] Workspace `Cargo.toml` with members `crates/calcio_core` and `crates/calcio_game`
- [x] `calcio_core` library crate with **no dependencies** (module skeleton + `World` placeholder)
- [x] `calcio_game` binary crate (`calcio-five`) depending on `macroquad = "0.4"` and `calcio_core`
- [x] Window: title, 1600×900, MSAA ×4, high-DPI
- [x] Fixed-timestep accumulator loop (240 Hz, frame time clamped to 0.1 s, max steps per frame)
- [x] Dev profile tuned so debug builds run smoothly (`opt-level = 1`, dependencies at `opt-level = 3`)
- **Done when:** `cargo run` opens a window that clears to a sky colour and shows FPS; `cargo test` passes.

### Step 02 — Math & deterministic RNG · `step/02-math-rng`
- [x] `Vec3` (add/sub/mul/div/neg, dot, cross, length, normalize, lerp, horizontal helpers)
- [x] `Quat` (from axis-angle, multiply, rotate vector, normalize, integrate angular velocity)
- [x] Scalar helpers: `clamp`, `lerp`, `smoothstep`, `move_towards`, angle wrap
- [x] Geometry helpers: closest point on segment, segment–segment closest points
- [x] `Rng` (xorshift64*, seedable): `f32` in range, gaussian, `chance`
- [x] Unit tests for all of the above
- **Done when:** `cargo test -p calcio_core` passes with math tests.

### Step 03 — Pitch & goal geometry model · `step/03-pitch-model`
- [x] `PitchSpec`: length 40, width 20, line width 0.08, centre circle r = 3, penalty area (6 m arcs + 3.16 m line), penalty spots (6 m and 10 m)
- [x] `ArenaSpec`: rebound boards 1.0 m high, 1.0 m behind touchlines, 1.5 m behind goal lines; cage netting above boards
- [x] `GoalSpec`: 3 × 2 m mouth, post/bar radius 0.04 m, net depth 0.8 m at top / 1.0 m at ground, back support bar
- [x] Goal frame as a list of capsule colliders (posts, crossbar, back bars)
- [x] Queries: `is_inside_penalty_area`, `goal_mouth_crossing(prev, curr)`, attack direction per team/half
- [x] Unit tests (penalty area, crossing detection)
- **Done when:** geometry tests pass.

### Step 04 — Rendering foundation · `step/04-render-foundation`
- [x] Mesh builder that splits geometry into macroquad-safe chunks (≤ 4000 vertices / indices per draw)
- [x] CPU lighting: directional sun + sky/ground hemisphere ambient + Blinn specular, baked into vertex colours
- [x] Primitives: box, icosphere, cylinder/capsule between two points, disc/ring, quad strip, ribbon
- [x] Broadcast camera (elevated sideline view) + screen→world direction helper for input
- [x] Sky dome with gradient (dusk/evening floodlit look)
- **Done when:** the game shows a lit test scene (spheres/boxes) under the broadcast camera.

### Step 05 — Arena rendering · `step/05-render-arena`
- [x] Artificial turf with mowing stripes and subtle per-vertex colour noise
- [x] White line markings (touchlines, goal lines, halfway line, centre circle & spot, penalty areas, penalty spots)
- [x] Rebound boards with panel shading; cage netting & posts above the boards
- [x] Goal frames (posts, crossbar, back support bars) — white, lit
- [x] Four floodlight towers, low stands around the arena, surrounding ground
- [x] All static meshes built once at start-up
- **Done when:** the empty arena renders nicely from the broadcast camera.

### Step 06 — Ball flight physics · `step/06-ball-flight`
- [x] `Ball` state: position, velocity, angular velocity, orientation (quaternion)
- [x] Gravity; quadratic air drag with drag crisis (Cd ≈ 0.45 slow → ≈ 0.22 fast)
- [x] Magnus lift from spin (lift coefficient from spin parameter, saturating)
- [x] Spin decay in the air; orientation integrated from angular velocity
- [x] Semi-implicit integration at 240 Hz
- [x] Tests: no spin → no lateral drift; topspin dips, backspin floats, sidespin curves the right way; drag shortens range vs vacuum
- **Done when:** flight tests pass.

### Step 07 — Ball–ground contact · `step/07-ball-ground`
- [x] Impulse-based bounce: speed-dependent restitution, Coulomb friction coupled to spin (hollow-sphere inertia 2/3·m·r²)
- [x] Sliding → rolling transition on the ground (friction until contact point stops slipping)
- [x] Rolling resistance on artificial turf (constant + speed-proportional) and vertical-axis spin decay
- [x] Resting state (ball settles, no jitter)
- [x] Tests: successive bounces get lower; rolling ball stops in a realistic distance; backspin bounce checks up; rolling ω matches v/r
- **Done when:** ground tests pass.

### Step 08 — Boards & goal frame collisions + ball rendering · `step/08-ball-frame-boards`
- [x] Ball vs rebound boards (planes with restitution/friction, spin effect) and cage
- [x] Ball vs posts, crossbar and back bars (capsule contact, high restitution "ping")
- [x] Game events: `PostHit`, `BoardHit` (for later use by camera/HUD)
- [x] Ball mesh with classic pentagon/hexagon panel pattern, rotating with the ball's orientation
- [x] Soft blob shadow under the ball (size/opacity by height)
- [x] Debug key `T`: fire a random test shot at a goal
- [x] Tests: ball cannot leave the arena; post hit deflects
- **Done when:** test shots bounce realistically off posts and boards on screen.

### Step 09 — Goal net cloth simulation · `step/09-net-cloth`
- [x] Net built from panels (roof, back, two sides) on a ~0.15 m mesh with shared seam nodes
- [x] Verlet integration + position-based distance constraints, slight slack so the net sags
- [x] Pinned nodes on crossbar, posts, back support bar and ground pegs
- [x] Damping, gravity, ground collision for net nodes
- [x] Net rendering as thin lit strands (one per constraint)
- [x] Tests: net settles to rest without exploding; pinned nodes never move
- **Done when:** both nets hang naturally and are stable.

### Step 10 — Ball ↔ net interaction & goals · `step/10-ball-net`
- [x] Two-way coupling: ball sphere vs net strands inside the constraint solver (mass ratio, momentum exchange)
- [x] Anti-tunnelling: adaptive substeps near the goal + side-of-net memory
- [x] Ball–net friction so the ball dies in the net and drops
- [x] Goal detection: ball wholly crosses the goal line between the posts and under the bar (`GoalScored` event)
- [x] Tests: shots up to 35 m/s from many angles never pass through the net; ball ends inside the goal; side-netting hit from outside stays outside
- **Done when:** shots bulge the net and goals are detected.

### Step 11 — Footballer body physics · `step/11-player-body`
- [x] `Player` state: position, velocity, facing, team, role
- [x] Movement model: acceleration, braking, top speed (jog/sprint), turn rate falling with speed, slower with the ball
- [x] Player–player collision (push apart), player–board collision
- [x] Ball–body collision (vertical capsule, moving body adds velocity) so the ball can be blocked
- [x] Tests: reaches top speed in a realistic time; players never overlap; ball rebounds off a body
- **Done when:** body tests pass.

### Step 12 — Player rendering & animation · `step/12-player-render`
- [x] Procedural humanoid from a joint skeleton (capsules for limbs, torso, head)
- [x] Team kits (shirt/shorts/socks), varied skin tones, goalkeeper kit
- [x] Run cycle driven by distance travelled; idle; kick swing; tackle lunge
- [x] Blob shadows for players
- [x] Marker ring + arrow under the human-controlled player
- **Done when:** players render and animate convincingly.

### Step 13 — Ball control & possession · `step/13-ball-control`
- [x] Dribbling as real touches (small kicks) that keep the ball ahead of the player; touch size grows with speed
- [x] First touch / receiving: kill or redirect an incoming ball, quality depends on ball speed and height
- [x] Possession tracking (who controls the ball, last toucher)
- [x] Tests: a dribbling player keeps the ball within range while running a straight line and a turn
- **Done when:** a scripted player can dribble across the pitch.

### Step 14 — Kicks · `step/14-kicks`
- [x] Kick request API used by both the human and the AI: kind, direction/target, power (0–1), curl (−1…1)
- [x] Ground pass with auto-target (teammate nearest the aim direction) and receiver lead
- [x] Shot: power → speed, elevation, aim inside the goal mouth, accuracy error from power/speed/angle
- [x] Lob/chip: high launch with backspin
- [x] Curl: side-spin with direction compensation
- [x] Launch solver that uses the real physics simulation to find the speed that reaches a target
- [x] Tests: passes arrive within tolerance; curl bends the right way; chip clears a 2 m obstacle at 5 m
- **Done when:** kick tests pass.

### Step 15 — Human controls · `step/15-human-input`
- [x] Camera-relative movement: WASD / arrow keys; **Shift** sprint
- [x] **J** shoot, **K** pass, **L** lob — hold to charge, release to kick
- [x] **Q / E** held while kicking add curl left/right
- [x] **Space** standing tackle (success depends on distance, angle and ball exposure; failure → short stumble)
- [x] Kick power bar near the controlled player
- **Done when:** you can run, dribble, pass, shoot, lob and tackle with the keyboard.

### Step 16 — Match flow & rules · `step/16-match-flow`
- [x] Kick-off setup (own halves, opponents outside the centre circle until the ball is touched)
- [x] Goal → celebration pause (ball keeps moving in the net) → kick-off for the team that conceded
- [x] Two halves (length configurable), sides swap at half-time, full-time state
- [x] HUD: score, clock, half indicator, “GOAL!” banner, pause (`Esc`)
- [x] Tests: goal updates score and restarts with the right team; clock and halves progress
- **Done when:** a full match can be played start to finish (AI still idle).

### Step 17 — AI team positioning · `step/17-ai-positioning`
- [x] Formations for 1–5 players per team (5v5: GK + diamond 1-2-1)
- [x] Team phase (attacking / defending / transition) from possession
- [x] Shape shifts and compresses with the ball position
- [x] Ball-chaser selection using intercept prediction from the physics simulation
- [x] Support runs into open space (pass-lane scoring), marking when defending
- **Done when:** AI players hold sensible shapes and chase the ball.

### Step 18 — AI decisions on and off the ball · `step/18-ai-on-ball`
- [x] On-ball choices: shoot / pass / dribble / clear, scored by goal chance, pass-lane safety and pressure
- [x] Executes the same kick and dribble mechanics as the human (no cheating)
- [x] Pressing goal-side and timing tackles on exposed balls
- [x] Reaction time and error by difficulty (Easy / Normal / Hard)
- [x] Tests: headless AI-vs-AI match produces passes, shots and goals
- **Done when:** AI teams play real football against each other and against you.

### Step 19 — Goalkeeper AI · `step/19-goalkeeper`
- [x] Positioning on the ball–goal line, narrowing the angle, staying near the area
- [x] Shot detection with trajectory prediction (where/when the ball crosses the goal plane)
- [x] Dive with reach limits; catch slow/central shots, parry fast/wide ones
- [x] Holding the ball (cannot be tackled), then roll/throw/kick to a free teammate
- [x] Goalkeeper hands only inside own penalty area
- [x] Tests: saves easy shots, beaten by well-placed hard shots
- **Done when:** keepers make believable saves.

### Step 20 — Match setup screen · `step/20-match-setup`
- [x] Minimal keyboard setup screen: team size (1v1…5v5), your position, match length, difficulty, start
- [x] Human controls exactly one player; teammates and opponents AI
- [x] Full time → result → back to setup
- **Done when:** every team size can be started and played.

### Step 21 — Camera feel & debug overlay · `step/21-camera-feel`
- [x] Broadcast camera smoothing, look-ahead in the ball's direction, context zoom, goal focus
- [x] `F1` debug overlay: FPS, ball speed/spin/height, physics colliders, AI targets
- **Done when:** the camera feels smooth and the overlay helps tuning.

### Step 22 — Tuning & QA · `step/22-tuning-qa`
- [x] Headless soak test: several full AI-vs-AI matches per team size (no NaNs, ball stays inside arena, no stuck states)
- [x] Physics tuning pass (pass speeds, shot power, bounce, net feel) — AI balance: ≈2 goals per 6-min 5v5 match, keepers parry wide, dropped ball for dead balls
- [x] Performance check in release build (target 144 FPS on a mid-range PC) — CPU cost measured headless: ~2–4 ms/frame for 5v5 geometry + lighting, ~25 µs per physics step; FPS still to be confirmed on the target PC
- [x] README: controls, how to build/run, architecture overview
- **Done when:** all tests pass and a full match feels good.

---

# Phase 2 — Realistic animation, characters, full action set, key bindings

Goal: footballers that look and move like real players, every football action available
to the human (and the AI), and keyboard controls the player can remap. Everything that
touches the ball stays **pure physics** (impulses, contacts, timing), exactly as in Phase 1.

| Topic | Decision |
|---|---|
| Animation approach | Procedural skeletal animation: joint rig with forward kinematics, two-bone IK for legs and arms, foot trajectories driven by the real ground speed (no foot sliding), keyframed action clips blended on top. |
| Realistic cadence | Step frequency and stride length follow running biomechanics (walk ≈ 0.9 Hz cycles, jog ≈ 1.4 Hz, sprint ≈ 2.1 Hz); stride = speed ÷ cadence. |
| Kick timing | A kick has a wind-up, a contact moment (ball must be at the foot, otherwise it is missed) and a follow-through. Animation and physics share that timing. |
| New actions | Jump, header, slide tackle, cross, chip, volley, keeper throw/roll; falls and get-ups. |
| Facing | Players can keep facing the ball while moving ("jockey"), which gives backward and side running. |
| Key bindings | Every action is remappable in a Controls screen and saved to a config file. |

## Phase 2 progress

### P2-00 — Plan · `p2/00-plan`
- [x] Phase 2 plan (this section)
- [x] Commit `Cargo.lock` (pins macroquad 0.4.16)

### P2-01 — Skeleton rig · `p2/01-rig`
- [x] Joint hierarchy (pelvis, spine, chest, neck, head, clavicles, arms, hands, legs, feet, toes) with real proportions
- [x] Pose representation that blends linearly (pelvis transform, spine/head angles, foot targets, arm channels / hand targets)
- [x] Forward kinematics to world joints and frames; two-bone IK with pole vectors for legs and arms
- [x] Per-player animator state kept between frames
- [x] Renderer draws from the rig
- [x] Tests: bone lengths preserved, IK reaches reachable targets, no NaNs

### P2-02 — Character model · `p2/02-character-model`
- [x] Lofted torso (hips, waist, chest, shoulders) following pelvis and chest frames
- [x] Tapered, muscled limbs (thigh, calf, deltoid, forearm) with elliptical sections and joint caps
- [x] Head shape (skull, jaw, nose, ears) and several hair styles
- [x] Boots, socks with shin guards, shorts, short-sleeved shirt with collar and cuffs, keeper gloves
- [x] Shirt numbers on the back
- [x] Body variety: height and build per player

### P2-03 — Movement modes · `p2/03-movement-modes`
- [x] Walk speed in addition to jog and sprint
- [x] Face-the-ball movement ("jockey") enabling backward and side running
- [x] Turning in place and realistic turning at speed
- [x] AI defenders jockey when close to the ball carrier
- [x] Tests: backward movement keeps facing, speeds per mode

### P2-04 — Locomotion animation · `p2/04-locomotion-anim`
- [x] Idle: breathing, weight shift, ready stance near the ball
- [x] Walk, jog, run, sprint with realistic cadence and stride length
- [x] Backward walk/run and side steps
- [x] Foot trajectories from ground speed + foot locking during stance (no sliding)
- [x] Shuffle steps when turning on the spot
- [x] Lean with acceleration, banking in turns, pelvis/torso counter-rotation, arm swing
- [x] Head looks at the ball
- [x] Smooth transitions between all gaits

### P2-05 — Kick states · `p2/05-kick-states`
- [x] Kick state machine: wind-up → contact → follow-through, duration by kick type and power
- [x] Contact only if the ball is at the kicking foot (otherwise a miss)
- [x] Preferred foot per player
- [x] Cross as its own kick type (curling, driven into the box)
- [x] Keeper throw / roll distribution
- [x] AI and human use the same kick states; tests updated

### P2-06 — Kick animations · `p2/06-kick-anims`
- [x] Pass (inside of the foot), shot (laces), cross (wrapped instep), chip, volley
- [x] Plant foot beside the ball, kicking-foot IK to the ball at contact
- [x] Arms and torso counter-balance, follow-through by power
- [x] Keeper underarm roll and overarm throw

### P2-07 — Jump & header · `p2/07-jump-header`
- [x] Jump physics (take-off crouch, flight, landing) for standing and running jumps
- [x] Headers: aimed impulse from the head when the ball meets it; unaimed balls bounce off
- [x] Body collision follows the jump
- [x] AI: attacking and defensive headers on high balls
- [x] Tests: jump height, header direction

### P2-08 — Slide tackle · `p2/08-slide-tackle`
- [x] Slide physics: body slides with ground friction, extended leg collides with the ball
- [x] Players caught by the slide are tripped (fall and get up)
- [x] Recovery time after sliding
- [x] AI uses slide tackles when a standing tackle cannot reach
- [x] Tests: slide distance, ball deflection, trips

### P2-09 — Action animations · `p2/09-action-anims`
- [x] Jump and header
- [x] Slide tackle and standing tackle
- [x] Fall and get-up
- [x] First touch / trap (foot, thigh, chest)
- [x] Keeper ready stance, catch, dive polish
- [x] Goal celebrations

### P2-10 — Key bindings · `p2/10-key-bindings`
- [ ] Every action bound to a key, defaults included
- [ ] Controls screen (from setup and pause): select action, press a key, conflicts swap, reset to defaults
- [ ] Bindings saved to and loaded from a config file
- [ ] Tests: parsing and saving the config

### P2-11 — Human actions · `p2/11-human-actions`
- [ ] Keys for jump/header, slide tackle, cross, chip, walk, jockey
- [ ] Context header when the ball is at head height
- [ ] HUD help reflects the current bindings

### P2-12 — Animation QA · `p2/12-anim-qa`
- [ ] Preview sequences of every animation
- [ ] Tests: no foot sliding at constant speed, IK limits, no NaNs across a soak
- [ ] AI-vs-AI soak with the new actions, balance check
- [ ] README and TODO updated
