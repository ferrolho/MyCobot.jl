# Checks controller firmware 5.0 over UDP (the WebSocket side: tools/firmware-tests/ws_smoke_test.py).
# MOVES THE ROBOT: the arm holds where it is (plans with zero motion); J7 (the gripper, if found) opens
# 5° from its goal and comes back. Do not run it while the gripper holds an object.
#
#     julia --project=. scripts/check_firmware5.jl [192.168.1.107]
using MyCobot, Printf

link = MyCobot.AtomLink(isempty(ARGS) ? "192.168.1.107" : ARGS[1])
fails = Ref(0)
check(ok, what) = (println(ok ? "ok   " : "FAIL ", what); ok || (fails[] += 1); ok)

p = MyCobot.atom_ping(link)
check(p.version >= v"5.0.0", "version $(p.version)")
s = MyCobot.atom_state(link)
n = length(s.q)
check(n in (6, 7) && length(s.goal) == n, "STATE: $n joints with goals")
check(maximum(abs, s.goal[1:6] .- s.q[1:6]) < 3, "J1-J6 goals within 3° of measured while holding (5.0: the goals, not the sagged positions)")
n == 7 && @printf("     J7 %.1f° (goal %.1f°)\n", s.q[7], s.goal[7])

# A 6-joint plan that stays where the arm is: 77-byte TELEM, J7 holds its goal.
t = collect(0:0.02:1.0)
q6 = repeat(permutedims(s.q[1:6]), length(t))
MyCobot.atom_upload_plan(link, t, q6, q6)
samples, done = MyCobot.atom_play(link)
check(done.result == "done" && all(length.(samples) .== 77), "6-joint plan: $(done.result), $(length(samples)) samples of $(unique(length.(samples))) bytes")
s2 = MyCobot.atom_state(link)
n == 7 && check(abs(s2.goal[7] - s.goal[7]) < 0.2, "J7 held its goal during the 6-joint plan")

if n == 7
    g0 = s.goal[7]
    g1 = g0 + 5 <= MyCobot.JOINT_LIMITS_DEG[7][2] - 2 ? g0 + 5 : g0 - 5   # 5° inside the goal range
    # A 7-joint plan: J7 to g1 and back, the arm still.
    q7 = hcat(q6, [g0 + (g1 - g0) * sin(pi * ti) for ti in t])
    MyCobot.atom_upload_plan(link, t, q7, q7)
    samples, done = MyCobot.atom_play(link)
    rec = MyCobot.decode_telemetry(samples)
    h = MyCobot.recording_header(7)
    j7max = maximum(abs, rec[:, findfirst(==("q_cmd_7"), h)] .- g0)
    check(done.result == "done" && all(length.(samples) .== 87), "7-joint plan: $(done.result), samples of $(unique(length.(samples))) bytes")
    check(abs(j7max - 5) < 0.3, @sprintf("J7 command went %.2f° from its goal (5 expected)", j7max))
    # Onboard MOVE_TO (atom_move!): J7 to g1, then back; J1-J6 keep their goals.
    s3 = MyCobot.atom_state(link)
    _, d1 = MyCobot.atom_move!(link, vcat(s3.goal[1:6], g1))
    sleep(0.3)
    s4 = MyCobot.atom_state(link)
    check(d1.result == "done" && abs(s4.goal[7] - g1) < 0.2, @sprintf("atom_move! J7 to %.1f°: measured %.1f°", g1, s4.q[7]))
    check(maximum(abs, s4.goal[1:6] .- s3.goal[1:6]) < 0.1, "J1-J6 goals unchanged by the J7 move")
    _, d2 = MyCobot.atom_move!(link, vcat(s4.goal[1:6], g0))
    check(d2.result == "done", "J7 back to $(round(g0; digits=1))°")
else
    q7 = hcat(q6, fill(-20.0, length(t)))
    MyCobot.atom_upload_plan(link, t, q7, q7)
    check(try MyCobot.atom_play(link); false catch e; occursin("-7", sprint(showerror, e)) end, "no gripper: a 7-joint plan is refused (-7)")
end
println(fails[] == 0 ? "all checks passed" : "$(fails[]) FAILURES")
