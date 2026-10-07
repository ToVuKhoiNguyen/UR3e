class VRPoseSource:
    """
    Nguồn dữ liệu tọa độ lấy từ Kính Meta Quest qua Máy chủ VR (WebSockets).
    """
    def __init__(self, vr_server):
        self.server = vr_server
        self._last_pose = None
        
    def get_pose(self):
        pose = self.server.get_pose()
        
        # Chỉ truyền tọa độ cho robot nếu sếp đang bóp cò (Trigger)
        if pose is None or not pose.get("trigger", False):
            return None
            
        # Tránh trả về dữ liệu trùng lặp nếu người dùng giữ yên tay
        if pose == self._last_pose:
            return None
            
        self._last_pose = pose
        return pose

    def get_current_pose(self):
        return self.server.get_pose()
