        document.getElementById('host-ip').innerText = window.location.hostname || '127.0.0.1 (Local Preview)';
        const statusDiv = document.getElementById('status');
        const btnEnterVR = document.getElementById('btn-enter-vr');
        const btnExitVR = document.getElementById('btn-exit-vr');
        const vrOverlay = document.getElementById('vr-overlay');
        
        // Element bindings for URSim UI
        const elValSource = document.getElementById('val-source');
        const elValTrigger = document.getElementById('val-trigger');
        const elValGrip = document.getElementById('val-grip');
        const elValGripperBtn = document.getElementById('val-gripper-btn');
        const elValX = document.getElementById('val-x');
        const elValY = document.getElementById('val-y');
        const elValZ = document.getElementById('val-z');
        const elValQx = document.getElementById('val-qx');
        const elValQy = document.getElementById('val-qy');
        const elValQz = document.getElementById('val-qz');
        const elValQw = document.getElementById('val-qw');
        const elValMode = document.getElementById('val-mode');

        let ws = null;
        let xrSession = null;
        let xrRefSpace = null;
        let glContext = null;
        let activeHandedness = 'right';
        let lastDriveTogglePressed = false;

        function updateCamDisplay(src) {
            const camImg = document.getElementById('wrist-cam-img');
            const placeholder = document.getElementById('cam-placeholder');
            if (camImg) {
                camImg.src = src;
                camImg.style.display = 'block';
            }
            if (placeholder) {
                placeholder.style.display = 'none';
            }
        }

        function connectWebSocket() {
            let host = window.location.hostname || '127.0.0.1';
            let isHttps = window.location.protocol === 'https:';
            // Meta Quest WebXR chạy https: -> dùng wss:// cổng 8444
            // Xem PC (file:/// hoặc http:) -> dùng ws:// cổng 8445 không cần chứng chỉ SSL tự ký
            const wsUrl = isHttps ? `wss://${host}:8444` : `ws://${host}:8445`;
            
            try {
                ws = new WebSocket(wsUrl);
            } catch (err) {
                console.error("Lỗi khởi tạo WebSocket:", err);
                return;
            }

            ws.onopen = () => {
                statusDiv.innerText = isHttps ? "Connected (Secure WSS)" : "Connected (Local WS)";
                statusDiv.classList.remove("warning");
                statusDiv.classList.add("success");
                checkWebXR();
            };

            ws.onerror = (err) => {
                statusDiv.innerText = isHttps ? "Connection Error on port 8444" : "Connection Error on port 8445";
                statusDiv.classList.remove("success");
                statusDiv.classList.add("warning");
                console.error("WS Error", err);
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'haptic' && xrSession) {
                        for (let source of xrSession.inputSources) {
                            if (source.handedness === activeHandedness && source.gamepad && source.gamepad.hapticActuators && source.gamepad.hapticActuators.length > 0) {
                                source.gamepad.hapticActuators[0].pulse(data.intensity, data.duration);
                            }
                        }
                    } else if (data.type === 'teleop_state') {
                        setDriveMode(Boolean(data.enabled), false);
                    } else if (data.type === 'camera' && data.image) {
                        lastCamUpdate = Date.now();
                        updateCamDisplay('data:image/jpeg;base64,' + data.image);
                    }
                } catch (e) { console.error("WS error", e); }
            };

            ws.onclose = () => {
                statusDiv.innerText = "Disconnected. Reconnecting...";
                statusDiv.classList.remove("success");
                statusDiv.classList.add("warning");
                setTimeout(connectWebSocket, 2000);
            };
        }

        function checkWebXR() {
            btnEnterVR.style.display = 'flex';
            if (navigator.xr) {
                navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
                    if (supported) {
                        btnEnterVR.innerText = 'START IMMERSIVE VR';
                        btnEnterVR.isVR = true;
                    } else {
                        btnEnterVR.innerText = 'START DASHBOARD';
                        btnEnterVR.isVR = false;
                    }
                });
            } else {
                btnEnterVR.innerText = 'START DASHBOARD';
                btnEnterVR.isVR = false;
            }
        }


        
        btnExitVR.addEventListener('click', () => {
            if (xrSession) {
                xrSession.end();
            } else {
                // Exit Preview Mode
                vrOverlay.style.display = 'none';
                document.getElementById('launcher-screen').style.display = 'flex';
                document.body.style.backgroundColor = '#f0f0f0';
            }
        });

        // Quản lý chuyển đổi giữa Edit Mode (chỉnh UI) và Drive Mode (lái Robot)
        let uiLocked = false;
        const btnToggleUI = document.getElementById('btn-toggle-ui');
        const cockpitPanel = document.getElementById('cockpit-panel');

        function setDriveMode(locked, notifyServer = true) {
            uiLocked = Boolean(locked);
            const overlay = document.getElementById('vr-overlay');

            if (uiLocked) {
                if (cockpitPanel) cockpitPanel.classList.add('drive-mode');
                if (overlay) overlay.style.pointerEvents = 'none';
                btnToggleUI.style.pointerEvents = 'auto';
                btnExitVR.style.pointerEvents = 'auto';
                btnToggleUI.innerHTML = '<span></span> Unlock UI (Edit Mode) [B/Y]';
                btnToggleUI.style.background = '#10b981';
                if (elValMode) {
                    elValMode.innerText = "Drive Mode";
                    elValMode.className = "badge green";
                }
            } else {
                if (cockpitPanel) cockpitPanel.classList.remove('drive-mode');
                if (overlay) overlay.style.pointerEvents = 'auto';
                btnToggleUI.style.pointerEvents = 'auto';
                btnExitVR.style.pointerEvents = 'auto';
                btnToggleUI.innerHTML = '<span></span> Lock UI (Drive Mode) [B/Y]';
                btnToggleUI.style.background = '#f59e0b';
                if (elValMode) {
                    elValMode.innerText = "Edit Mode";
                    elValMode.className = "badge orange";
                }
            }

            // Đồng bộ trạng thái lên Python server
            if (notifyServer && ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'ui_state',
                    drive_mode: uiLocked
                }));
            }
        }

        btnToggleUI.addEventListener('click', (e) => {
            e.stopPropagation();
            setDriveMode(!uiLocked, true);
        });

        btnEnterVR.addEventListener('click', () => {
            if (btnEnterVR.isVR) {
                if (!xrSession) {
                    // DOM Overlay BẮT BUỘC phải hiển thị (không được display: none) TRƯỚC KHI requestSession
                    vrOverlay.style.display = 'flex';
                    
                    // Yêu cầu chế độ AR với dom-overlay
                    navigator.xr.requestSession('immersive-ar', {
                        optionalFeatures: ['local-floor', 'bounded-floor', 'dom-overlay'],
                        domOverlay: { root: vrOverlay }
                    }).then(onSessionStarted).catch((err) => {
                        console.log("AR failed, fallback to VR", err);
                        navigator.xr.requestSession('immersive-vr', {
                            optionalFeatures: ['local-floor', 'bounded-floor', 'dom-overlay'],
                            domOverlay: { root: vrOverlay }
                        }).then(onSessionStarted);
                    });
                }
            } else {
                // PC Preview mode
                vrOverlay.style.display = 'flex';
                document.getElementById('launcher-screen').style.display = 'none';
                document.body.style.backgroundColor = '#ccc'; // Giả lập nền AR
            }
        });

        function onSessionStarted(session) {
            xrSession = session;
            btnEnterVR.style.display = 'none';
            document.getElementById('launcher-screen').style.display = 'none';
            vrOverlay.style.display = 'flex'; // Hiện Overlay bằng flex để căn giữa
            setDriveMode(uiLocked, false);
            statusDiv.innerText = "In VR Session";

            session.addEventListener('end', onSessionEnded);

            // Thiết lập WebGL dummy để WebXR chịu chạy loop
            const canvas = document.getElementById('webgl-canvas');
            glContext = canvas.getContext('webgl', { xrCompatible: true, alpha: true });
            session.updateRenderState({ baseLayer: new XRWebGLLayer(session, glContext) });

            session.requestReferenceSpace('local').then((refSpace) => {
                xrRefSpace = refSpace;
                session.requestAnimationFrame(onXRFrame);
            });
        }

        function onSessionEnded(event) {
            xrSession = null;
            btnEnterVR.style.display = 'inline-block';
            vrOverlay.style.display = 'none'; // Ẩn Cockpit UI
            document.getElementById('launcher-screen').style.display = 'flex';
            statusDiv.innerText = "VR Session Ended.";
        }

        function onXRFrame(time, frame) {
            let session = frame.session;
            session.requestAnimationFrame(onXRFrame);
            
            // Xóa buffer WebGL để không cản trở lớp AR (Xuyên thấu)
            if (glContext) {
                glContext.bindFramebuffer(glContext.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
                glContext.clearColor(0, 0, 0, 0); // Trong suốt hoàn toàn
                glContext.clear(glContext.COLOR_BUFFER_BIT | glContext.DEPTH_BUFFER_BIT);
            }

            if (!ws || ws.readyState !== WebSocket.OPEN) return;

            let sentData = false;

            let activeSource = null;
            // Ưu tiên tay phải
            for (let source of session.inputSources) {
                if (source.targetRayMode === 'gaze') continue;
                if (!source.gripSpace && !source.targetRaySpace) continue;
                if (source.handedness === 'right') {
                    activeSource = source;
                    break;
                }
            }
            // Nếu không có tay phải thì lấy tay trái
            if (!activeSource) {
                for (let source of session.inputSources) {
                    if (source.targetRayMode === 'gaze') continue;
                    if (!source.gripSpace && !source.targetRaySpace) continue;
                    if (source.handedness === 'left') {
                        activeSource = source;
                        break;
                    }
                }
            }

            if (activeSource) {
                activeHandedness = activeSource.handedness;
                let space = activeSource.gripSpace || activeSource.targetRaySpace;
                let pose = frame.getPose(space, xrRefSpace);
                
                if (pose) {
                    let triggerPressed = false;
                    let gripPressed = false;
                    let gripperBtnPressed = false;
                    let driveTogglePressed = false;
                    
                    if (activeSource.gamepad && activeSource.gamepad.buttons) {
                        // Button 0: Index Trigger (bóp trỏ - tịnh tiến)
                        if (activeSource.gamepad.buttons.length > 0) triggerPressed = activeSource.gamepad.buttons[0].pressed;
                        // Button 1: Hand Grip (bóp hông - xoay)
                        if (activeSource.gamepad.buttons.length > 1) gripPressed = activeSource.gamepad.buttons[1].pressed;
                        // Button 4: Nút A (tay phải) hoặc Nút X (tay trái) - Kẹp/Nhả Gripper
                        if (activeSource.gamepad.buttons.length > 4) gripperBtnPressed = activeSource.gamepad.buttons[4].pressed;
                        // Button 5: Nút B (tay phải) hoặc Nút Y (tay trái) - Phím cứng chuyển đổi Drive Mode
                        if (activeSource.gamepad.buttons.length > 5) driveTogglePressed = activeSource.gamepad.buttons[5].pressed;
                    }

                    // Xử lý sườn lên nút B/Y: bấm 1 lần để chuyển Drive Mode và rung nhẹ xác nhận
                    if (driveTogglePressed && !lastDriveTogglePressed) {
                        setDriveMode(!uiLocked, true);
                        if (activeSource.gamepad && activeSource.gamepad.hapticActuators && activeSource.gamepad.hapticActuators.length > 0) {
                            activeSource.gamepad.hapticActuators[0].pulse(0.7, 120);
                        }
                    }
                    lastDriveTogglePressed = driveTogglePressed;

                    let pos = pose.transform.position;
                    let quat = pose.transform.orientation;

                    ws.send(JSON.stringify({
                        t: Date.now() / 1000.0,
                        hand: activeSource.handedness || "unknown",
                        pos: [pos.x, pos.y, pos.z],
                        quat: [quat.w, quat.x, quat.y, quat.z],
                        trigger: triggerPressed,
                        grip: gripPressed,
                        gripper_btn: gripperBtnPressed,
                        drive_mode: uiLocked
                    }));
                    
                    let handName = activeSource.handedness === 'right' ? "Right Hand" : (activeSource.handedness === 'left' ? "Left Hand" : "Unknown");
                    
                    // Cập nhật dữ liệu lên URSim Layout
                    elValSource.innerText = handName;
                    
                    elValTrigger.innerText = triggerPressed ? "Pressed" : "Released";
                    elValTrigger.className = triggerPressed ? "badge red" : "badge gray";
                    
                    elValGrip.innerText = gripPressed ? "Pressed" : "Released";
                    elValGrip.className = gripPressed ? "badge red" : "badge gray";

                    if (elValGripperBtn) {
                        elValGripperBtn.innerText = gripperBtnPressed ? "Closed" : "Open";
                        elValGripperBtn.className = gripperBtnPressed ? "badge red" : "badge gray";
                    }
                    
                    elValX.innerText = pos.x.toFixed(4);
                    elValY.innerText = pos.y.toFixed(4);
                    elValZ.innerText = pos.z.toFixed(4);
                    
                    elValQx.innerText = quat.x.toFixed(4);
                    elValQy.innerText = quat.y.toFixed(4);
                    elValQz.innerText = quat.z.toFixed(4);
                    elValQw.innerText = quat.w.toFixed(4);
                    
                    sentData = true;
                }
            }

            // Ghi log để debug nếu không có tay cầm nào được nhận diện
            if (!sentData) {
                elValSource.innerText = "Đang chờ...";
                
                let debugStr = "";
                for(let s of session.inputSources) {
                    debugStr += `[${s.handedness || 'none'}, ${s.targetRayMode}, pose:${frame.getPose(s.gripSpace||s.targetRaySpace, xrRefSpace) ? 'yes':'null'}] `;
                }
                // Optional: Console log the debug string since the UI doesn't have a giant box for it anymore
                // console.log("Missing pose: ", debugStr);
            }
        }

        // Quản lý luồng stream camera ảo cổ tay
        let lastCamUpdate = 0;
        let isPolling = false;

        function pollCameraFrame() {
            if (Date.now() - lastCamUpdate < 250 || isPolling) return;
            isPolling = true;

            const pollImg = new Image();
            pollImg.onload = () => {
                isPolling = false;
                updateCamDisplay(pollImg.src);
            };
            pollImg.onerror = () => {
                isPolling = false;
                // Nếu đang mở bằng file:/// và chưa nạp được, thử fallback qua cổng HTTPS 8443
                if (window.location.protocol === 'file:' && !pollImg.hasFallenBack) {
                    pollImg.hasFallenBack = true;
                    let host = window.location.hostname || '127.0.0.1';
                    pollImg.src = `https://${host}:8443/wrist_camera.jpg?t=${Date.now()}`;
                }
            };
            // Đường dẫn tương đối không có dấu gạch chéo đầu giúp tương thích hoàn hảo cả file:/// lẫn http/https
            pollImg.src = `wrist_camera.jpg?t=${Date.now()}`;
        }

        setInterval(pollCameraFrame, 100);

        // Start
        connectWebSocket();

        // Auto-assign the correct IP for Viser iframe based on current host
        const viserIframe = document.getElementById('viser-iframe');
        if (viserIframe) {
            let host = window.location.hostname || '127.0.0.1';
            viserIframe.src = `http://${host}:8080`;
        }
