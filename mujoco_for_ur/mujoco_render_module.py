import mujoco
import glfw
import numpy as np

class MujocoRenderer:
    def __init__(self, model, data, camera_name=None, resolution=(320, 240)):
        """
        初始化Mujoco渲染器，负责Mujoco自身的渲染功能
        
        参数:
        - model: Mujoco模型对象
        - data: Mujoco数据对象
        - camera_name: 要使用的相机名称
        - resolution: 渲染分辨率 (宽度, 高度)
        """
        self.model = model
        self.data = data
        self.resolution = resolution
        self.camera_id = -1
        
        # 初始化GLFW
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")
        
        # 获取相机ID
        if camera_name:
            try:
                self.camera_id = self.model.camera(camera_name).id
                print(f"Mujoco渲染器: 相机 '{camera_name}' ID: {self.camera_id}")
            except:
                print(f"Mujoco渲染器警告: 未找到相机 '{camera_name}'")
        
        # 离屏渲染相关变量
        self.offscreen_window = None
        self.offscreen_scene = None
        self.offscreen_context = None
        self.offscreen_cam = None
        self.viewport = None
        
    def setup_offscreen_rendering(self):
        """设置离屏渲染环境"""
        # 创建离屏渲染窗口
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        self.offscreen_window = glfw.create_window(
            self.resolution[0], self.resolution[1], "Offscreen Camera", None, None
        )
        
        if not self.offscreen_window:
            glfw.terminate()
            raise RuntimeError("Failed to create offscreen window")
        
        glfw.make_context_current(self.offscreen_window)
        
        # 创建场景和上下文
        self.offscreen_scene = mujoco.MjvScene(self.model, maxgeom=1000)
        self.offscreen_context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value
        )
        
        # 创建相机
        self.offscreen_cam = mujoco.MjvCamera()
        self.offscreen_cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        
        if self.camera_id != -1:
            self.offscreen_cam.fixedcamid = self.camera_id
        
        # 创建帧缓冲对象
        self.viewport = mujoco.MjrRect(0, 0, self.resolution[0], self.resolution[1])
        
        # 设置离屏缓冲区
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.offscreen_context)
        
        print("Mujoco离屏渲染环境已设置完成")
    
    def render_image(self):
        """
        使用Mujoco渲染图像
        
        返回:
        - rgb_image: RGB格式的图像数据 (高度, 宽度, 3)
        """
        if not self.offscreen_window:
            raise RuntimeError("离屏渲染环境未初始化，请先调用setup_offscreen_rendering()")
        
        # 确保使用离屏渲染上下文
        glfw.make_context_current(self.offscreen_window)
        
        # 更新场景
        mujoco.mjv_updateScene(
            self.model, self.data, mujoco.MjvOption(), 
            mujoco.MjvPerturb(), self.offscreen_cam, 
            mujoco.mjtCatBit.mjCAT_ALL, self.offscreen_scene
        )
        
        # 渲染
        mujoco.mjr_render(self.viewport, self.offscreen_scene, self.offscreen_context)
        
        # 读取像素数据 - 使用numpy数组替代不存在的MjrImage
        rgb = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, self.viewport, self.offscreen_context)
        
        #修复mujoco 渲染的坐标系和OpenCV的坐标系不一致问题 mujoco 左下角为(0,0)，OpenCV左上角为(0,0)
        rgb = np.flipud(rgb).copy()
        return rgb
    
    def is_window_open(self):
        """检查离屏窗口是否打开"""
        if self.offscreen_window:
            return glfw.window_should_close(self.offscreen_window) == 0
        return True
    
    def cleanup(self):
        """清理Mujoco渲染资源"""
        try:
            if self.offscreen_context:
                mujoco.mjr_freeContext(self.offscreen_context)
            if self.offscreen_window:
                glfw.destroy_window(self.offscreen_window)
            glfw.terminate()
            print("Mujoco渲染资源已清理")
        except Exception as e:
            print(f"Mujoco渲染资源清理时出错: {e}")