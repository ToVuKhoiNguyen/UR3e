#!/usr/bin/env python3

import sys, os, math, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))

import torch
import viser
from curobo_ursim_control import (
    CuRoboURSim, send_urscript, wait_for_move, GoalToolPose
)
from curobo.types import ContentPath, JointState
from curobo.viewer import ViserVisualizer
from curobo.motion_planner import MotionPlannerCfg, MotionPlanner

from singularity_guard.guard_core import SingularityGuard
from singularity_guard.runtime_monitor_v2 import RuntimeMonitorV2
from singularity_guard.data_logger import DataLogger
from teleoperation import (
    GizmoPoseSource, PyRokiIKSolver, CuRoboIKSolver,
    SafetyGate, ServoJExecutor, TeleoperationController
)
import yaml

def load_config():
    path = os.path.join(os.path.dirname(__file__), "config", "singularity.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)

VISER_PORT   = 8080   
MOVE_SPEED   = 0.7  
MOVE_ACCEL   = 1.2    
POLL_SLEEP   = 0.05   # thoi gian ngu giua moi vong lap (20 hz)
POS_THRESH   = 0.003  # nguong di chuyen chuot (3mm) de bat dau tinh toan ik
RELEASE_WAIT = 0.4    # thoi gian im lang (khong keo chuot) de xac nhan dich den

HOME_Q = [0.0, -2.2, 1.9, -1.38, -1.57, 0.0] # vi tri home

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
]

SCENE_FILE = os.path.join(os.path.dirname(__file__), "obstacle_scene.yml") 


def build_urscript_traj(waypoints, speed=0.8, accel=1.5): # doi tu duong di waypoints thanh lenh movej
    script = "def traj():\n"
    # duyet qua tung diem tren quy dao
    for i, q in enumerate(waypoints):
        # neu chua phai la diem cuoi cung thi tinh toan do bo goc (r)
        if i < len(waypoints) - 1:
            q_next = waypoints[i+1]
            dist = math.sqrt(sum((a - b)**2 for a, b in zip(q, q_next)))
            r = min(0.02, dist * 0.2) # ep r toi da la 2cm de tranh cat goc dam vao vat can
        else:
            r = 0.0 # diem cuoi cung khong duoc bo goc
        # ghep toa do khop thanh chuoi lenh movej cua urscript
        j = ", ".join(f"{x:.6f}" for x in q)
        script += f"  movej([{j}], a={accel}, v={speed}, r={r})\n"
    script += "end\n"
    return script


def main():
    # ── 1. khoi tao ket noi voi robot that/ao va bo giai ik ───────────────
    import os
    scene_file = os.path.join(os.path.dirname(__file__), "obstacle_scene.yml")
    ctrl = CuRoboURSim(scene_file=scene_file)
    ik   = ctrl.ik_solver

    print("[PyRoki] Khởi tạo bộ giải IK JAX...")
    import pyroki as pk, jax, jax.numpy as jnp, jaxlie, jaxls, jax_dataclasses as jdc, yourdfpy, logging, numpy as np
    import pyroki_mp
    logging.getLogger("yourdfpy").setLevel(logging.CRITICAL)
    urdf = yourdfpy.URDF.load("/home/nguyen/ur_ws/curobo_ursim/ur3e_custom.urdf")
    pk_robot = pk.Robot.from_urdf(urdf)
    
    # khoi tao the gioi va robot cho pyRoki MP
    world_coll = [pk.collision.Box.from_extent(jnp.array([1.5, 1.5, 0.1]), position=jnp.array([0.0, 0.0, -0.05]))]
    robot_coll = pk.collision.RobotCollision.from_urdf(urdf)
    target_link_index = pk_robot.links.names.index("tool0")
    
    @jdc.jit
    def solve_ik_pyroki(target_pos, target_quat, current_q_jnp):
        joint_var = pk_robot.joint_var_cls(0)
        costs = [
            pk.costs.pose_cost_analytic_jac(pk_robot, joint_var, jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_quat), target_pos), jnp.array(target_link_index), pos_weight=50.0, ori_weight=10.0),
            pk.costs.limit_constraint(pk_robot, joint_var),
            pk.costs.rest_cost(joint_var, current_q_jnp, 2.0)
        ]
        return jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var]).analyze().solve(
            verbose=False, 
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
            initial_vals=jaxls.VarValues.make((joint_var.with_value(current_q_jnp),))
        )[joint_var]

    @jax.jit
    def check_error(q_sol, target_pos):
        return jnp.linalg.norm(pk_robot.forward_kinematics(q_sol)[target_link_index, 4:] - target_pos)

    # ── 1.5. khoi tao he thong bao ve singularity (v2) ────────────────────
    config = load_config()
    guard  = SingularityGuard(config)
    logger = DataLogger(config)
    monitor = RuntimeMonitorV2(config, ctrl) # luong giam sat chay ngam

    # ── 1.6. khoi tao teleop modules ─────────────────────────────────────
    # (wiring se hoan thanh sau khi co target_tf o buoc 5)
    _teleop_pyroki_solver = PyRokiIKSolver(
        pk_robot=pk_robot,
        solve_fn=solve_ik_pyroki,
        check_error_fn=check_error,
        target_link_index=target_link_index
    )
    _teleop_curobo_solver = CuRoboIKSolver(ik_solver=ik)
    _teleop_gate = SafetyGate(guard=guard)

    # ── 2. khoi tao bo nao ai (motion planner) va nap du lieu cai ban ─────
    if not os.path.exists(SCENE_FILE):
        print(f"[ERROR] Không tìm thấy: {SCENE_FILE}")
        ctrl.close(); return

    print(f"[MP] Đang khởi tạo Motion Planner với scene: {SCENE_FILE} ...")
    # nap file cau hinh robot ur3e va file cau hinh cai ban
    mp_cfg = MotionPlannerCfg.create(robot="/home/nguyen/ur_ws/curobo_ursim/ur3e_custom.yml", scene_model=SCENE_FILE)
    mp     = MotionPlanner(mp_cfg)
    mp.warmup(enable_graph=True, num_warmup_iterations=2) # lam nong ai (chay thu truoc de toi uu)
    print("[MP] Motion Planner sẵn sàng ✓")

    # ── 3. khoi tao giao dien web 3d (viser) ───────────────────────────────
    subprocess.run("fuser -k 8080/tcp 2>/dev/null", shell=True) # tat cac tien trinh dang dung cong 8080 (neu co)
    time.sleep(0.5)

    # khoi tao server viser
    viz = ViserVisualizer(
        content_path=ContentPath(robot_config_file="/home/nguyen/ur_ws/curobo_ursim/ur3e_custom.yml"),
        connect_ip="0.0.0.0",
        connect_port=VISER_PORT,
        add_control_frames=False, # Ẩn tất cả các trục tọa độ ở các khớp khác
    )
    server: viser.ViserServer = viz._server

    # ── 4. ve cai ban vao moi truong 3d de nhin thay duoc ──────────────────
    scene_cfg = mp_cfg.scene_collision_cfg.scene_model
    viz.add_scene(scene_cfg)
    print("[Viser] Đã render vật cản ✓")

    # ── 5. tao cai truc toa do de user keo tha (target widget) ────────────
    q_init_tensor = torch.tensor([ctrl.joints()], device="cuda", dtype=torch.float32)
    fk_init = ik.solver.kinematics.get_link_poses(q_init_tensor, ["tool0"])
    init_pos = fk_init.position[0, 0].cpu().tolist()
    init_quat = fk_init.quaternion[0, 0].cpu().tolist()

    target_tf = server.scene.add_transform_controls(
        "/curobo_target",
        position=init_pos,
        wxyz=init_quat, # dat huong ban dau chinh xac bang huong hien tai cua robot!
        scale=0.18, # kich thuoc truc
        disable_sliders=False, # bat slider de hien thi hinh vuong cho de keo
    )
    
    ui_moved = False
    @target_tf.on_update
    def _(_):
        nonlocal ui_moved
        ui_moved = True

    # ── 6. them cac nut bam va bang trang thai tren web ───────────────────
    with server.gui.add_folder("Robot Control (CuRobo & PyRoki)"):
        status_text = server.gui.add_text("Status", initial_value="Ready")
        cb_preview  = server.gui.add_checkbox("Bật IK Preview", initial_value=True)
        ik_mode_dropdown = server.gui.add_dropdown("IK Solver", options=["CuRobo", "PyRoki"], initial_value="CuRobo")
        mp_mode_dropdown = server.gui.add_dropdown("MP Solver", options=["CuRobo", "PyRoki"], initial_value="CuRobo")
        latency_text = server.gui.add_text("Latency", initial_value="0.0 ms", disabled=True)
        btn_home    = server.gui.add_button("Go Home")

    with server.gui.add_folder("🕹 Teleoperation"):
        cb_teleop    = server.gui.add_checkbox("Bật Teleop (servoj)", initial_value=False)
        teleop_status = server.gui.add_text("Teleop Status", initial_value="⏸ Tắt", disabled=True)
        teleop_lat   = server.gui.add_text("Teleop Latency", initial_value="-", disabled=True)

    # bien toan cuc de theo doi tien trinh di chuyen cua robot
    movement_target_q = None
    movement_start_time = 0
    movement_timeout = 0

    # su kien khi bam nut go home
    @btn_home.on_click
    def _home(_):
        nonlocal movement_target_q, movement_start_time, movement_timeout
        
        # Tắt Teleop nếu đang bật để tránh xung đột lệnh servoj và movej
        if teleop.enabled:
            teleop.enabled = False
            cb_teleop.value = False
            teleop_status.value = "Tắt"
            
        status_text.value = "Going home..."
        # khoa luong giam sat lai, khong cho no xen vao luc dang ve nha
        monitor.is_moving = True
        
        # tinh toan thoi gian timeout phu thuoc vao khoang cach xa hay gan
        current_q = ctrl.joints()
        travel = max(abs(HOME_Q[i] - current_q[i]) for i in range(6))
        movement_timeout = max(15.0, travel / 0.4 * 3.0)
        movement_start_time = time.time()
        movement_target_q = HOME_Q

        # gui lenh movej yeu cau robot chay thang ve home_q
        script = f"movej([{', '.join(f'{x:.6f}' for x in HOME_Q)}], a=1.0, v=0.4, r=0)\n"
        send_urscript(script)

    print(f"Open http://localhost:{VISER_PORT}")

    # cac bien dung de kiem tra thay doi cua chuot
    last_ts      = target_tf.update_timestamp
    dragging     = False
    pending_pos  = None
    pending_quat = None
    pending_q_ik = None
    is_target_reachable = None

    monitor.start() # bat dau chay luong bao ve ngam 20hz
    
    frame_counter = 0

    # ── Wire teleop sau khi co target_tf ─────────────────────────────────
    _pose_source = GizmoPoseSource(target_tf)
    _executor    = ServoJExecutor(dt=POLL_SLEEP)
    teleop = TeleoperationController(
        pose_source    = _pose_source,
        pyroki_solver  = _teleop_pyroki_solver,
        curobo_solver  = _teleop_curobo_solver,
        safety_gate    = _teleop_gate,
        executor       = _executor,
        dt             = POLL_SLEEP,
    )

    @cb_teleop.on_update
    def _toggle_teleop(_):
        teleop.enabled = cb_teleop.value
        if not cb_teleop.value:
            teleop_status.value = "Tắt"

    try:
        # vong lap chinh, chay lien tuc voi toc do 20 vong tren 1 giay
        while True:
            frame_counter += 1

            # ── TELEOP MODE: uu tien xuong lenh servoj ngay ───────────────
            q_real = ctrl.joints()
            if teleop.enabled:
                teleop.step(q_real)
                teleop_status.value = teleop.get_status()
                lat = teleop.get_latency()
                teleop_lat.value = f"PyRoki:{lat['pyroki_ms']:.1f}ms | CuRobo:{lat['curobo_ms']:.1f}ms"
                
                # Cập nhật hiển thị tay máy trên Viser (nếu không có đoạn này, robot ảo sẽ bị đóng băng)
                js_real = JointState.from_position(
                    torch.tensor([q_real], dtype=torch.float32, device="cuda"),
                    joint_names=JOINT_NAMES
                )
                viz.set_joint_state(js_real)

                time.sleep(POLL_SLEEP)
                continue  # bo qua toan bo logic drag-and-release khi dang teleop

            # ── a. goi ve si kiem tra an toan (chi kiem tra khi dung yen) ──
            if monitor.check_current_state():
                monitor.trigger_safe_return(monitor.danger_q) # loi co robot ve home ngay neu nguy hiem
                monitor.danger_detected = False
                continue

            # doc 6 goc khop thuc te tu ursim
            q_real = ctrl.joints()

            # cap nhat hinh anh robot tren web cho giong voi ngoai doi thuc
            if not dragging:
                js_real = JointState.from_position(
                    torch.tensor([q_real], dtype=torch.float32, device="cuda"),
                    joint_names=JOINT_NAMES
                )
                viz.set_joint_state(js_real)

            # ── b. doc toa do cua cai truc muc tieu ───────────────────────
            pos  = target_tf.position.tolist()
            quat = target_tf.wxyz.tolist()

            # ── c. dang keo chuot: tinh toan ik de hien thi bong ma ────────
            if ui_moved:
                ui_moved = False
                dragging     = True
                pending_pos  = pos
                pending_quat = quat
                last_ts = time.time()

                if cb_preview.value:
                    t_ik_start = time.perf_counter()
                    
                    if ik_mode_dropdown.value == "CuRobo":
                        # ai tinh toan ik tu vi tri hien tai den vi tri chuot bang CuRobo
                        q_ik = ik.solve(pos, quat, current_q=q_real)
                    else:
                        # giai ik bang PyRoki
                        q_sol_jnp = solve_ik_pyroki(jnp.array(pos), jnp.array(quat), jnp.array(q_real))
                        if not np.isnan(q_sol_jnp).any():
                            err = check_error(q_sol_jnp, jnp.array(pos))
                            if err <= 0.005:
                                q_ik = np.array(q_sol_jnp).tolist()
                            else:
                                q_ik = None
                        else:
                            q_ik = None
                            
                    t_ik_end = time.perf_counter()
                    latency_ms = (t_ik_end - t_ik_start) * 1000
                    latency_text.value = f"{latency_ms:.2f} ms (Live)"
                    
                    is_target_reachable = (q_ik is not None)
                    pending_q_ik = q_ik
                    
                    if q_ik is not None: # neu co nghiem ik (nam trong tam voi)
                        # hien thi bong ma robot o vi tri do len web
                        js_preview = JointState.from_position(
                            torch.tensor([q_ik], dtype=torch.float32, device="cuda"),
                            joint_names=JOINT_NAMES
                        )
                        viz.set_joint_state(js_preview)
                        
                        # tien the tinh luon do an toan (sigma_min) xem co nguy hiem khong
                        metrics = guard.metrics_calc.get_metrics(
                            torch.tensor([q_ik], device=guard.metrics_calc.device))
                        sigma = metrics["sigma_min"].item()
                        
                        # bao mau do hoac cam len man hinh neu qua sat diem ky di
                        if sigma < guard.danger_threshold:
                            status_text.value = f" IK (DANGER): sigma={sigma:.4f}"
                        elif sigma < guard.warn_threshold:
                            status_text.value = f" IK (WARN): sigma={sigma:.4f}"
                        else:
                            status_text.value = f" IK preview: sigma={sigma:.4f}"

                        # ghi log lanh lang de phan tich sau
                        logger.log_step(q_ik, None, None, pos, sigma,
                                        metrics["manipulability"].item(),
                                        metrics["condition_number"].item(),
                                        "danger" if sigma < guard.danger_threshold else "none",
                                        q_real, "OK", "ik_preview")
                    else: # neu ik that bai (nguoi dung keo qua xa)
                        status_text.value = " IK: Ngoài tầm với"
                else:
                    status_text.value = f"Đang nhắm mục tiêu: {[round(v,2) for v in pos]}"

            # ── d. nha tay: kich hoat motion planner ve duong ───────────────
            elif dragging and pending_pos is not None:
                idle_time = time.time() - last_ts
                if idle_time <= RELEASE_WAIT:
                    if 'latency_ms' in locals() and cb_preview.value:
                        latency_text.value = f"{latency_ms:.2f} ms (Live)"
                else:
                    if 'latency_ms' in locals() and cb_preview.value:
                        latency_text.value = f"{latency_ms:.2f} ms (Idle - Chốt)"
                    
                    dragging = False
                    p  = pending_pos
                    qu = pending_quat
                    q_goal = pending_q_ik
                    pending_pos = None
                    pending_q_ik = None

                    print(f"Nhả tay → lập kế hoạch né vật cản tới {[round(v,3) for v in p]}...")
                    status_text.value = "Đang lập kế hoạch..."

                    # khoi tao diem bat dau (start) va diem den (goal)
                    q_start = JointState.from_position(
                        torch.tensor([q_real], dtype=torch.float32, device="cuda"),
                        joint_names=mp.joint_names
                    )
                    goal_pos  = torch.tensor([[[[[p[0],  p[1],  p[2]]]]]], dtype=torch.float32, device="cuda")
                    goal_quat = torch.tensor([[[[[qu[0], qu[1], qu[2], qu[3]]]]]], dtype=torch.float32, device="cuda")
                    goal = GoalToolPose(
                        tool_frames=["tool0"],
                        position=goal_pos,
                        quaternion=goal_quat
                    )

                    # chay ai do duong ne vat can
                    if mp_mode_dropdown.value == "CuRobo":
                        t_start_mp = time.perf_counter()
                        result = mp.plan_pose(goal, q_start)
                        t_end_mp = time.perf_counter()
                        print(f"CuRobo tìm đường mất {(t_end_mp - t_start_mp)*1000:.2f} ms")
                        
                        if result is not None and result.success.any():
                            # neu co ket qua, lay danh sach cac diem (waypoints) cua duong di
                            if hasattr(result, 'interpolated_plan') and result.interpolated_plan is not None:
                                traj = result.interpolated_plan.squeeze().cpu().tolist()
                            else:
                                traj = result.solution.squeeze().cpu().tolist()
                        else:
                            traj = None
                    else: # PyRoki MP
                        print("[MP] Đang giải bằng PyRoki TrajOpt (Online Planning)...")
                        try:
                            timesteps = 20
                            start_cfg_np = onp.array(q_real)
                            
                            if q_goal is not None:
                                q_goal_np = onp.array(q_goal)
                                prev_sols = onp.linspace(start_cfg_np, q_goal_np, timesteps)
                            else:
                                prev_sols = onp.tile(start_cfg_np, (timesteps, 1))
                            
                            t_start_mp = time.perf_counter()
                            sol_traj, _, _ = pyroki_mp.solve_online_planning(
                                robot=pk_robot,
                                robot_coll=robot_coll,
                                world_coll=world_coll,
                                target_link_name="tool0",
                                target_position=onp.array(p),
                                target_wxyz=onp.array(qu),
                                timesteps=timesteps,
                                dt=0.05,
                                start_cfg=start_cfg_np,
                                prev_sols=prev_sols
                            )
                            t_end_mp = time.perf_counter()
                            print(f"[MP] PyRoki tìm đường mất {(t_end_mp - t_start_mp)*1000:.2f} ms")
                            traj = sol_traj.tolist()
                        except Exception as e:
                            print(f"[MP] PyRoki Lỗi: {e}")
                            traj = None
                    
                    if traj is not None:
                        
                        # --- GUARD 5: CHỐNG TỰ KẸP TAY (Protective Stop C403A0) ---
                        # kiem tra toan bo duong di xem ai co xui bay robot gap co tay qua sau khong
                        is_clamped = False
                        for q in traj:
                            if abs(q[3]) > 2.9 or abs(q[4]) > 2.9: # 2.9 rad la khoang 166 do
                                is_clamped = True
                                break
                        
                        if is_clamped: # neu co gap sau thi tu choi vinh vien duong di nay
                            print("TỪ CHỐI: Quỹ đạo gập cổ tay quá sâu (nguy cơ kẹp nách)!")
                            status_text.value = "Từ chối: Quá gập, dễ kẹp tay!"
                            pending_pos = None
                            pending_quat = None
                            continue

                        # rut gon bot cac diem trung lap tren duong di (cho nhe code urscript)
                        filtered_traj = [traj[0]]
                        for pt in traj[1:]:
                            if math.sqrt(sum((a - b)**2 for a, b in zip(pt, filtered_traj[-1]))) > 0.01:
                                filtered_traj.append(pt)
                        if len(filtered_traj) == 1:
                            filtered_traj.append(traj[-1])

                        # quet toan bo duong di xem co xuyen qua lo hong singularity nao khong
                        q_tensor = torch.tensor(filtered_traj, device=guard.metrics_calc.device, dtype=torch.float32)
                        m_all = guard.metrics_calc.get_metrics(q_tensor)
                        min_sigma_traj = torch.min(m_all["sigma_min"]).item()

                        clamped_traj = filtered_traj
                        
                        # bao dong neu duong di xuyen qua hoac liem sat vao diem ky di
                        if min_sigma_traj < guard.danger_threshold:
                            print(f"CẢNH BÁO: Quỹ đạo có cắt ngang qua ĐIỂM KỲ DỊ (min_sigma = {min_sigma_traj:.4f})!")
                            print(f"Đã tự động xử lý an toàn nhờ di chuyển trong Joint Space.")
                            status_text.value = f"Xuyên qua kỳ dị ({len(clamped_traj)} wps)..."
                        elif min_sigma_traj < guard.warn_threshold:
                            print(f"Cảnh báo nhẹ: Quỹ đạo đi sát vùng kỳ dị (min_sigma = {min_sigma_traj:.4f})")
                            status_text.value = f"Đi sát kỳ dị ({len(clamped_traj)} wps)..."
                        else:
                            print(f"Tìm được đường đi an toàn ({len(clamped_traj)} waypoints) → gửi URSim...")
                            status_text.value = f"Đang di chuyển ({len(clamped_traj)} wps)..."

                        # ghi log
                        for i, q in enumerate(clamped_traj):
                            m = guard.metrics_calc.get_metrics(
                                torch.tensor([q], device=guard.metrics_calc.device))
                            logger.log_step(q, None, None, None,
                                            m["sigma_min"].item(), m["manipulability"].item(),
                                            m["condition_number"].item(), "none", None, "OK", "mp_execute")
                        logger.flush()

                        # ── GỬI LỆNH CHO ROBOT CHẠY THẬT ────────────────────
                        # bien danh sach cac diem thanh ngon ngu urscript de robot hieu
                        script = build_urscript_traj(clamped_traj, speed=MOVE_SPEED, accel=MOVE_ACCEL)
                        
                        # buoc chan monitor lai, dung choc pha khi robot dang tap trung chay ne chuong ngai vat
                        monitor.is_moving = True
                        send_urscript(script) # day lenh xuong cong 30002 cua ursim

                        # tinh thoi gian timeout dua tren do dai duong di
                        total_dist = sum(
                            max(abs(clamped_traj[i][j] - clamped_traj[i-1][j]) for j in range(6))
                            for i in range(1, len(clamped_traj))
                        )
                        movement_timeout = max(15.0, total_dist / MOVE_SPEED * 3.0)
                        movement_start_time = time.time()
                        movement_target_q = clamped_traj[-1] # luu lai dich den de kiem tra

                    else:
                        print("Lỗi: Không thể tìm đường!")
                        if is_target_reachable is False:
                            status_text.value = "Lỗi: Mục tiêu ngoài tầm với!"
                        elif is_target_reachable is True:
                            status_text.value = "Lỗi: Vướng vật cản chặn đường!"
                        else:
                            status_text.value = "Lỗi: Chạm vật cản hoặc ngoài tầm với!"

            # ── e. the hien trang thai luc robot dang di chuyen thuc te ─────
            if movement_target_q is not None:
                # lay van toc that ma robot dang quay (tu rtde cong 30004)
                qd_real = ctrl.rr.getActualQd()
                max_qd = max(abs(v) for v in qd_real)
                
                # tinh chi so an toan sigma_min hien tai
                m_curr = guard.metrics_calc.get_metrics(torch.tensor([q_real], device=guard.metrics_calc.device))
                sigma_curr = m_curr["sigma_min"].item()
                
                # dich ra cac muc canh bao cho hinh anh truc quan hon
                if sigma_curr < guard.danger_threshold:
                    state_str = f" ĐANG TRONG LÕI KỲ DỊ (sigma={sigma_curr:.4f})"
                elif sigma_curr < guard.warn_threshold:
                    state_str = f" Sát vùng kỳ dị (sigma={sigma_curr:.4f})"
                else:
                    state_str = f" An toàn (sigma={sigma_curr:.4f})"
                    
                status_text.value = f"Chạy | vmax: {max_qd:.2f} rad/s | {state_str}"
                
                if frame_counter % 10 == 0:
                    print(f"Vận tốc max: {max_qd:.3f} rad/s | {state_str}")

                # kiem tra xem robot da den dich chua (sai so giua thuc te va dich < 0.05 rad)
                if max(abs(q_real[i] - movement_target_q[i]) for i in range(6)) < 0.05:
                    status_text.value = "Đã đến đích!"
                    movement_target_q = None
                    monitor.is_moving = False # thao xich cho ve si monitor tiep tuc hoat dong
                # kiem tra xem co bi loi mat ket noi hoac bi tac nghen dan toi het thoi gian khong
                elif time.time() - movement_start_time > movement_timeout:
                    status_text.value = f" Timeout ({movement_timeout:.0f}s)"
                    movement_target_q = None
                    monitor.is_moving = False # thao xich cho ve si

            # ngu mot chut truoc khi qua vong lap khac de tiet kiem cpu
            time.sleep(POLL_SLEEP)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        monitor.stop()
        ctrl.close()

if __name__ == "__main__":
    main()
