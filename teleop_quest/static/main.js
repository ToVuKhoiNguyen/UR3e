        document.getElementById('host-ip').innerText = window.location.hostname;
        const statusDiv = document.getElementById('status');
        const btnEnterVR = document.getElementById('btn-enter-vr');
        const btnExitVR = document.getElementById('btn-exit-vr');
        const vrOverlay = document.getElementById('vr-overlay');
        
        // Element bindings for URSim UI
        const elValSource = document.getElementById('val-source');
        const elValTrigger = document.getElementById('val-trigger');
        const elValGrip = document.getElementById('val-grip');
        const elValX = document.getElementById('val-x');
        const elValY = document.getElementById('val-y');
        const elValZ = document.getElementById('val-z');
        const elValQx = document.getElementById('val-qx');
        const elValQy = document.getElementById('val-qy');
        const elValQz = document.getElementById('val-qz');
        const elValQw = document.getElementById('val-qw');

        let ws = null;
        let xrSession = null;
        let xrRefSpace = null;
        let glContext = null;

        function connectWebSocket() {
            const wsUrl = `wss://${window.location.hostname}:8444`;
            ws = new WebSocket(wsUrl);

            ws.onopen = () => {
                statusDiv.innerText = "Connected (Secure)";
                statusDiv.classList.remove("warning");
                statusDiv.classList.add("success");
                checkWebXR();
            };

            ws.onerror = (err) => {
                statusDiv.innerText = "Connection Error on port 8444";
                statusDiv.classList.remove("success");
                statusDiv.classList.add("warning");
                console.error("WS Error", err);
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'haptic' && xrSession) {
                        for (let source of xrSession.inputSources) {
                            if (source.handedness === 'right' && source.gamepad && source.gamepad.hapticActuators && source.gamepad.hapticActuators.length > 0) {
                                source.gamepad.hapticActuators[0].pulse(data.intensity, data.duration);
                            }
                        }
                    }
                } catch (e) { console.error("Haptic error", e); }
            };

            ws.onclose = () => {
                statusDiv.innerText = "Disconnected. Reconnecting...";
                statusDiv.classList.remove("success");
                statusDiv.classList.add("warning");
                setTimeout(connectWebSocket, 2000);
            };
        }

        function checkWebXR() {
            if (navigator.xr) {
                navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
                    if (supported) {
                        btnEnterVR.style.display = 'inline-block';
                        statusDiv.innerText = "WebXR Ready. Press Enter VR.";
                    } else {
                        statusDiv.innerText = "immersive-vr not supported on this browser.";
                    }
                });
            } else {
                statusDiv.innerText = "WebXR not available (navigator.xr is undefined). Are you on HTTPS?";
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

        // Debug mode: Cho phép xem trước Cockpit trên máy tính PC
        document.getElementById('btn-preview-ui').addEventListener('click', () => {
            vrOverlay.style.display = 'flex';
            document.getElementById('launcher-screen').style.display = 'none';
            document.body.style.backgroundColor = '#ccc'; // Giả lập nền AR
        });

        btnEnterVR.addEventListener('click', () => {
            if (!xrSession) {
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
        });

        function onSessionStarted(session) {
            xrSession = session;
            btnEnterVR.style.display = 'none';
            document.getElementById('launcher-screen').style.display = 'none';
            vrOverlay.style.display = 'flex'; // Hiện Overlay bằng flex để căn giữa
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

            for (let source of session.inputSources) {
                // Bỏ qua điểm nhìn bằng mắt/đầu (gaze)
                if (source.targetRayMode === 'gaze') continue;

                let space = source.gripSpace || source.targetRaySpace;
                if (!space) continue;

                let pose = frame.getPose(space, xrRefSpace);
                if (pose) {
                    let triggerPressed = false;
                    let gripPressed = false;
                    
                    if (source.gamepad && source.gamepad.buttons) {
                        if (source.gamepad.buttons.length > 0) triggerPressed = source.gamepad.buttons[0].pressed;
                        if (source.gamepad.buttons.length > 1) gripPressed = source.gamepad.buttons[1].pressed;
                    }

                    let pos = pose.transform.position;
                    let quat = pose.transform.orientation;

                    ws.send(JSON.stringify({
                        t: Date.now() / 1000.0,
                        hand: source.handedness || "unknown",
                        pos: [pos.x, pos.y, pos.z],
                        quat: [quat.w, quat.x, quat.y, quat.z],
                        trigger: triggerPressed,
                        grip: gripPressed
                    }));
                    
                    let handName = source.handedness === 'right' ? "Tay Phải" : (source.handedness === 'left' ? "Tay Trái" : "Unknown");
                    
                    // Cập nhật dữ liệu lên URSim Layout
                    elValSource.innerText = handName;
                    
                    elValTrigger.innerText = triggerPressed ? "BÓP" : "Nhả";
                    elValTrigger.className = triggerPressed ? "badge red" : "badge gray";
                    
                    elValGrip.innerText = gripPressed ? "BÓP" : "Nhả";
                    elValGrip.className = gripPressed ? "badge red" : "badge gray";
                    
                    elValX.innerText = pos.x.toFixed(4);
                    elValY.innerText = pos.y.toFixed(4);
                    elValZ.innerText = pos.z.toFixed(4);
                    
                    elValQx.innerText = quat.x.toFixed(4);
                    elValQy.innerText = quat.y.toFixed(4);
                    elValQz.innerText = quat.z.toFixed(4);
                    elValQw.innerText = quat.w.toFixed(4);
                    
                    sentData = true;
                    break;
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

        // Start
        connectWebSocket();

        // Auto-assign the correct IP for Viser iframe based on current host
        const viserIframe = document.getElementById('viser-iframe');
        if (viserIframe) {
            viserIframe.src = `http://${window.location.hostname}:8080`;
        }
