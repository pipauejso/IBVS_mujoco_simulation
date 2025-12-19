import cv2
import numpy as np

class OpenCVRenderer:
    def __init__(self, window_name="Camera View", resolution=(320, 240)):
        """
        初始化OpenCV渲染器，负责图像的显示和处理
        
        参数:
        - window_name: 显示窗口的名称
        - resolution: 显示分辨率 (宽度, 高度)
        """
        self.window_name = window_name
        self.resolution = resolution
        
        # 创建OpenCV窗口
        self._create_window()
        
        print(f"OpenCV渲染器: 窗口 '{window_name}' 已创建")
    
    def _create_window(self):
        """创建OpenCV显示窗口"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.resolution[0], self.resolution[1])
    
    def convert_image(self, rgb_image):
        """
        将Mujoco渲染的RGB图像转换为OpenCV可用的BGR格式
        
        参数:
        - rgb_image: Mujoco渲染的RGB图像 (高度, 宽度, 3)
        
        返回:
        - bgr_image: OpenCV可用的BGR图像 (高度, 宽度, 3)
        """
        # 翻转图像并转换颜色空间
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        return bgr_image
    
    def show_image(self, image):
        """
        显示图像
        
        参数:
        - image: 要显示的图像 (可以是RGB或BGR格式)
        
        返回:
        - key_pressed: 按下的键盘按键代码，-1表示没有按键按下
        """
        # 如果是RGB格式，转换为BGR
        if image.shape[2] == 3:
            image = self.convert_image(image)
        
        # 显示图像
        cv2.imshow(self.window_name, image)
        
        # 检查键盘输入
        key_pressed = cv2.waitKey(1)
        return key_pressed
    
    def process_image(self, image, operations=None):
        """
        对图像进行处理
        
        参数:
        - image: 输入图像
        - operations: 图像处理操作列表，例如["gray", "blur", "edge"]
        
        返回:
        - processed_image: 处理后的图像
        """
        processed_image = image.copy()
        
        if operations:
            for op in operations:
                if op == "gray":
                    processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
                elif op == "blur":
                    processed_image = cv2.GaussianBlur(processed_image, (5, 5), 0)
                elif op == "edge":
                    processed_image = cv2.Canny(processed_image, 100, 200)
        
        return processed_image
    
    def capture_screenshot(self, image, filename="screenshot.png"):
        """
        捕获并保存当前图像
        
        参数:
        - image: 要保存的图像
        - filename: 保存的文件名
        """
        # 如果是RGB格式，转换为BGR
        if image.shape[2] == 3:
            image = self.convert_image(image)
        
        cv2.imwrite(filename, image)
        print(f"截图已保存为: {filename}")
    
    def cleanup(self):
        """清理OpenCV资源"""
        try:
            cv2.destroyWindow(self.window_name)
            print(f"OpenCV渲染器: 窗口 '{self.window_name}' 已关闭")
        except Exception as e:
            print(f"OpenCV渲染器清理时出错: {e}")
