#!/usr/bin/env python3
"""
Open3D URDF可视化工具
使用Open3D库实现高质量的3D模型显示和流畅的交互体验
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import xml.etree.ElementTree as ET
import numpy as np
import math
import os
import copy
from typing import Dict, List, Tuple
import threading
import time

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Open3D not available. Please install it with: pip install open3d")
    
try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def rotation_matrix_from_rpy(roll, pitch, yaw):
    """从RPY角计算旋转矩阵"""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    return R


def create_transformation_matrix(xyz, rpy):
    """创建4x4变换矩阵"""
    transform = np.eye(4)
    transform[:3, 3] = xyz
    transform[:3, :3] = rotation_matrix_from_rpy(*rpy)
    return transform


class Link:
    """链接类"""
    def __init__(self, name):
        self.name = name
        self.visual = None


class Joint:
    """关节类"""
    def __init__(self, name, joint_type):
        self.name = name
        self.type = joint_type
        self.parent = None
        self.child = None
        self.origin_xyz = [0, 0, 0]
        self.origin_rpy = [0, 0, 0]
        self.axis = [0, 0, 1]
        self.limit_lower = -math.pi
        self.limit_upper = math.pi


class Open3DURDFVisualizer:
    def __init__(self, master):
        self.master = master
        master.title("Open3D URDF Visualizer")
        master.geometry("1400x900")
        
        # 数据存储
        self.links = {}  # name -> Link
        self.joints = {}  # name -> Joint
        self.joint_variables = {}  # name -> tk.DoubleVar
        self.root_links = []
        self.meshes = {}  # link_name -> open3d mesh
        self.original_meshes = {}  # link_name -> original open3d mesh
        self.link_transforms = {}  # link_name -> transformation matrix
        
        # Open3D可视化相关
        self.vis = None
        self.vis_thread = None
        self.vis_running = False
        self.vis_geometries = []  # 存储可视化几何体
        self.coord_frame = None  # 坐标系
        
        # URDF路径信息
        self.urdf_dir = ""
        self.package_paths = []
        
        # 创建界面
        self.create_widgets()
        
    def create_widgets(self):
        # 菜单栏
        menubar = tk.Menu(self.master)
        self.master.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Open URDF", command=self.load_urdf)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.master.quit)
        
        # 主框架
        main_paned = ttk.PanedWindow(self.master, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左侧面板
        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=1)
        
        # 右侧面板
        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=2)
        
        # 左侧: 结构树和关节控制
        structure_frame = ttk.LabelFrame(left_frame, text="Robot Structure")
        structure_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        # 结构建树
        self.tree = ttk.Treeview(structure_frame)
        tree_scroll = ttk.Scrollbar(structure_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 关节控制
        joint_frame = ttk.LabelFrame(left_frame, text="Joint Control")
        joint_frame.pack(fill=tk.BOTH, expand=True)
        
        # 关节控制滚动区域
        joint_canvas = tk.Canvas(joint_frame)
        joint_scrollbar = ttk.Scrollbar(joint_frame, orient="vertical", command=joint_canvas.yview)
        self.joint_control_frame = ttk.Frame(joint_canvas)
        
        joint_canvas.configure(yscrollcommand=joint_scrollbar.set)
        joint_scrollbar.pack(side="right", fill="y")
        joint_canvas.pack(side="left", fill="both", expand=True)
        joint_canvas.create_window((0, 0), window=self.joint_control_frame, anchor="nw")
        
        self.joint_control_frame.bind(
            "<Configure>",
            lambda e: joint_canvas.configure(scrollregion=joint_canvas.bbox("all"))
        )
        
        # 右侧: 控制面板
        control_frame = ttk.LabelFrame(right_frame, text="Controls")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(button_frame, text="Reset Joints", command=self.reset_joints).pack(side=tk.LEFT)
        ttk.Button(button_frame, text="Show 3D View", command=self.show_3d_view).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Update View", command=self.update_3d_view).pack(side=tk.LEFT)
        
        # 信息显示区域
        info_frame = ttk.LabelFrame(right_frame, text="Information")
        info_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.info_text = tk.Text(info_frame, height=10)
        info_scroll = ttk.Scrollbar(info_frame, orient="vertical", command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=info_scroll.set)
        
        self.info_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        info_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 状态栏
        self.status_var = tk.StringVar(value="Ready to load URDF file")
        status_bar = ttk.Label(self.master, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def load_urdf(self):
        """加载URDF文件"""
        if not OPEN3D_AVAILABLE:
            messagebox.showerror("Error", "Open3D is not available. Please install it with: pip install open3d")
            return
            
        file_path = filedialog.askopenfilename(
            title="Select URDF File",
            filetypes=[("URDF files", "*.urdf"), ("XACRO files", "*.xacro"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
            
        try:
            self.urdf_dir = os.path.dirname(file_path)
            # 设置可能的package路径
            self.package_paths = [
                os.path.join(self.urdf_dir, "..", "jn3_description"),
                os.path.join(self.urdf_dir, "..", "..", "jn3_description"),
                os.path.join(self.urdf_dir, "..", "jn2_description"),
                os.path.join(self.urdf_dir, "..", "..", "jn2_description")
            ]
            
            self.parse_urdf(file_path)
            self.build_structure_tree()
            self.create_joint_controls()
            self.compute_forward_kinematics()
            self.status_var.set(f"Loaded: {os.path.basename(file_path)}")
            self.update_info_display()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load URDF: {str(e)}")
            import traceback
            print(traceback.format_exc())
            
    def parse_urdf(self, file_path: str):
        """解析URDF文件"""
        # 清空现有数据
        self.links.clear()
        self.joints.clear()
        self.joint_variables.clear()
        self.root_links.clear()
        self.meshes.clear()
        self.original_meshes.clear()
        self.link_transforms.clear()
        
        # 解析XML
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        # 解析链接
        for link_elem in root.findall('link'):
            name = link_elem.get('name')
            if name:
                link = Link(name)
                link.visual = self.parse_visual(link_elem.find('visual'))
                self.links[name] = link
                
        # 解析关节
        for joint_elem in root.findall('joint'):
            name = joint_elem.get('name')
            joint_type = joint_elem.get('type')
            if name and joint_type:
                joint = Joint(name, joint_type)
                
                # 解析父子链接
                parent_elem = joint_elem.find('parent')
                child_elem = joint_elem.find('child')
                if parent_elem is not None:
                    joint.parent = parent_elem.get('link')
                if child_elem is not None:
                    joint.child = child_elem.get('link')
                
                # 解析原点
                origin_elem = joint_elem.find('origin')
                if origin_elem is not None:
                    xyz = origin_elem.get('xyz')
                    rpy = origin_elem.get('rpy')
                    if xyz:
                        joint.origin_xyz = [float(x) for x in xyz.split()]
                    if rpy:
                        joint.origin_rpy = [float(x) for x in rpy.split()]
                
                # 解析轴
                axis_elem = joint_elem.find('axis')
                if axis_elem is not None:
                    xyz = axis_elem.get('xyz')
                    if xyz:
                        joint.axis = [float(x) for x in xyz.split()]
                
                # 解析限制
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    lower = limit_elem.get('lower')
                    upper = limit_elem.get('upper')
                    if lower:
                        joint.limit_lower = float(lower)
                    if upper:
                        joint.limit_upper = float(upper)
                
                self.joints[name] = joint
                
                # 为可动关节创建变量
                if joint_type in ['revolute', 'continuous', 'prismatic']:
                    self.joint_variables[name] = tk.DoubleVar(value=0.0)
        
        # 确定根链接
        child_links = {joint.child for joint in self.joints.values() if joint.child}
        self.root_links = [name for name in self.links.keys() if name not in child_links]
        
        # 加载网格模型
        self.load_meshes()
        
    def parse_visual(self, visual_elem):
        """解析视觉元素"""
        if visual_elem is None:
            return None
            
        visual = {}
        
        # 解析原点
        origin_elem = visual_elem.find('origin')
        if origin_elem is not None:
            xyz_str = origin_elem.get('xyz')
            rpy_str = origin_elem.get('rpy')
            visual['origin_xyz'] = [float(x) for x in xyz_str.split()] if xyz_str else [0, 0, 0]
            visual['origin_rpy'] = [float(x) for x in rpy_str.split()] if rpy_str else [0, 0, 0]
            
        # 解析几何
        geometry_elem = visual_elem.find('geometry')
        if geometry_elem is not None:
            mesh_elem = geometry_elem.find('mesh')
            if mesh_elem is not None:
                filename = mesh_elem.get('filename')
                if filename:
                    visual['mesh_filename'] = filename
                    
        return visual
        
    def resolve_mesh_path(self, filename):
        """解析网格文件路径"""
        # 处理package://路径
        if filename.startswith('package://'):
            # 移除package://前缀
            relative_path = filename[len('package://'):]
            
            # 分割package名和相对路径
            parts = relative_path.split('/', 1)
            if len(parts) == 2:
                package_name, mesh_relative_path = parts
                
                # 在所有可能的package路径中查找
                for package_path in self.package_paths:
                    if os.path.basename(package_path) == package_name:
                        full_path = os.path.join(package_path, mesh_relative_path)
                        if os.path.exists(full_path):
                            return full_path
                        
                # 特殊处理jn3_description中的meshes/visual路径
                for package_path in self.package_paths:
                    if os.path.basename(package_path) == package_name:
                        # 检查是否已经包含meshes/visual路径
                        if "meshes/visual" not in mesh_relative_path:
                            # 尝试添加meshes/visual路径
                            full_path = os.path.join(package_path, "meshes", "visual", mesh_relative_path)
                            if os.path.exists(full_path):
                                return full_path
                        else:
                            # 如果已经包含meshes/visual，则直接使用
                            full_path = os.path.join(package_path, mesh_relative_path)
                            if os.path.exists(full_path):
                                return full_path
                            
        # 处理相对路径
        elif not os.path.isabs(filename):
            # 在URDF目录中查找
            full_path = os.path.join(self.urdf_dir, filename)
            if os.path.exists(full_path):
                return full_path
                
        # 绝对路径
        else:
            if os.path.exists(filename):
                return filename
                
        return None
        
    def load_meshes(self):
        """加载网格模型"""
        if not OPEN3D_AVAILABLE:
            return
            
        for link_name, link in self.links.items():
            if link.visual and 'mesh_filename' in link.visual:
                mesh_filename = link.visual['mesh_filename']
                
                # 解析实际文件路径
                resolved_path = self.resolve_mesh_path(mesh_filename)
                
                if resolved_path and os.path.exists(resolved_path):
                    try:
                        # 加载网格
                        mesh = o3d.io.read_triangle_mesh(resolved_path)
                        if not mesh.has_vertex_normals():
                            mesh.compute_vertex_normals()
                        self.original_meshes[link_name] = mesh
                        # 创建副本用于变换
                        self.meshes[link_name] = copy.deepcopy(mesh)
                        print(f"Loaded mesh for {link_name}: {resolved_path}")
                    except Exception as e:
                        print(f"Failed to load mesh {resolved_path}: {e}")
                else:
                    print(f"Mesh file not found: {mesh_filename}")
                    
    def build_structure_tree(self):
        """构建结构树"""
        # 清空树
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        # 添加根链接
        for link_name in self.root_links:
            link_item = self.tree.insert('', 'end', text=link_name, values=('link',))
            
            # 递归添加子结构
            self.add_structure_items(link_item, link_name)
            
    def add_structure_items(self, parent_item, parent_link):
        """递归添加结构项"""
        # 查找从该链接出发的关节
        for joint in self.joints.values():
            if joint.parent == parent_link:
                # 添加关节
                joint_item = self.tree.insert(parent_item, 'end', text=joint.name, values=('joint',))
                
                # 添加子链接
                if joint.child:
                    link_item = self.tree.insert(joint_item, 'end', text=joint.child, values=('link',))
                    self.add_structure_items(link_item, joint.child)
                    
    def create_joint_controls(self):
        """创建关节控制面板"""
        # 清空现有控件
        for widget in self.joint_control_frame.winfo_children():
            widget.destroy()
            
        # 为每个可动关节创建控件
        for joint_name, joint_var in self.joint_variables.items():
            joint = self.joints[joint_name]
            
            frame = ttk.Frame(self.joint_control_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            
            # 关节名称
            ttk.Label(frame, text=joint_name, width=25, anchor="w").pack(side=tk.LEFT)
            
            # 滑块
            slider = ttk.Scale(
                frame, 
                from_=joint.limit_lower, 
                to=joint.limit_upper, 
                variable=joint_var,
                orient=tk.HORIZONTAL,
                command=lambda x, jn=joint_name: self.on_joint_change(jn)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            # 数值显示
            value_label = ttk.Label(frame, textvariable=joint_var, width=10)
            value_label.pack(side=tk.LEFT)
            
    def on_joint_change(self, joint_name):
        """关节值变化回调"""
        # 更新正向运动学
        self.compute_forward_kinematics()
        
        # 更新3D视图
        self.update_3d_view()
        
    def reset_joints(self):
        """重置所有关节"""
        for joint_var in self.joint_variables.values():
            joint_var.set(0.0)
            
        # 更新正向运动学
        self.compute_forward_kinematics()
        
        # 更新3D视图
        self.update_3d_view()
        
    def compute_forward_kinematics(self):
        """计算正向运动学"""
        # 初始化变换矩阵字典
        self.link_transforms = {}
        
        # 设置根链接变换为单位矩阵
        for root_link in self.root_links:
            self.link_transforms[root_link] = np.eye(4)
            
        # 递归计算所有链接的变换
        for root_link in self.root_links:
            self.compute_link_transforms(root_link)
            
    def compute_link_transforms(self, parent_link):
        """递归计算链接变换"""
        # 查找从该链接出发的关节
        for joint in self.joints.values():
            if joint.parent == parent_link and joint.child:
                # 构建关节变换矩阵
                joint_transform = self.build_joint_transform(joint)
                
                # 计算子链接变换
                parent_transform = self.link_transforms[parent_link]
                child_transform = np.dot(parent_transform, joint_transform)
                self.link_transforms[joint.child] = child_transform
                
                # 递归计算子链接的变换
                self.compute_link_transforms(joint.child)
                
    def build_joint_transform(self, joint):
        """构建关节变换矩阵"""
        # 创建关节变换矩阵（包含关节的origin）
        transform = create_transformation_matrix(joint.origin_xyz, joint.origin_rpy)
        
        # 如果是可动关节，应用关节变换
        if joint.name in self.joint_variables and joint.type in ['revolute', 'continuous']:
            joint_value = self.joint_variables[joint.name].get()
            
            # 创建关节旋转矩阵
            axis = np.array(joint.axis)
            axis = axis / np.linalg.norm(axis)  # 归一化
            
            # Rodrigues旋转公式
            cos_theta = math.cos(joint_value)
            sin_theta = math.sin(joint_value)
            k = axis
            
            # 计算旋转矩阵
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            
            R = np.eye(3) + sin_theta * K + (1 - cos_theta) * np.dot(K, K)
            
            # 创建关节旋转的变换矩阵
            joint_rotation = np.eye(4)
            joint_rotation[:3, :3] = R
            
            # 应用关节旋转到变换矩阵
            transform = np.dot(transform, joint_rotation)
            
        return transform
        
    def show_3d_view(self):
        """显示3D视图"""
        if not OPEN3D_AVAILABLE:
            messagebox.showerror("Error", "Open3D is not available. Please install it with: pip install open3d")
            return
            
        if not self.links:
            messagebox.showwarning("Warning", "Please load a URDF file first")
            return
            
        # 如果可视化线程已经在运行，就不创建新的
        if self.vis_running:
            return
            
        # 在新线程中运行可视化
        self.vis_thread = threading.Thread(target=self._run_visualization)
        self.vis_thread.daemon = True
        self.vis_thread.start()
        
    def _run_visualization(self):
        """在独立线程中运行可视化"""
        self.vis_running = True
        
        try:
            # 创建可视化窗口
            self.vis = o3d.visualization.VisualizerWithKeyCallback()
            self.vis.create_window(window_name="URDF 3D Visualization", width=1000, height=800)
            
            # 添加坐标系
            self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            self.vis.add_geometry(self.coord_frame)
            
            # 添加网格模型
            self.add_meshes_to_visualization()
            
            # 设置视图控制
            ctr = self.vis.get_view_control()
            ctr.set_front([0, 0, -1])
            ctr.set_lookat([0, 0, 0])
            ctr.set_up([0, 1, 0])
            ctr.set_zoom(0.8)
            
            # 运行可视化循环
            while self.vis_running:
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.01)  # 避免占用过多CPU
                
        except Exception as e:
            print(f"Error in visualization: {e}")
        finally:
            self.vis_running = False
            
    def add_meshes_to_visualization(self):
        """添加网格到可视化"""
        if not self.vis:
            return
            
        for link_name, mesh in self.meshes.items():
            # 应用视觉元素变换
            link = self.links[link_name]
            if link.visual and 'origin_xyz' in link.visual and 'origin_rpy' in link.visual:
                visual_transform = create_transformation_matrix(
                    link.visual['origin_xyz'], 
                    link.visual['origin_rpy']
                )
                mesh.transform(visual_transform)
                
            # 应用链接变换
            if link_name in self.link_transforms:
                link_transform = self.link_transforms[link_name]
                mesh.transform(link_transform)
                
            self.vis.add_geometry(mesh)
            
    def update_3d_view(self):
        """更新3D视图"""
        if not self.vis or not self.vis_running:
            return
            
        # 更新网格变换
        for link_name, mesh in self.meshes.items():
            # 恢复原始网格
            if link_name in self.original_meshes:
                self.meshes[link_name].vertices = o3d.utility.Vector3dVector(np.asarray(self.original_meshes[link_name].vertices))
                self.meshes[link_name].triangles = self.original_meshes[link_name].triangles
                if self.original_meshes[link_name].has_vertex_normals():
                    self.meshes[link_name].vertex_normals = self.original_meshes[link_name].vertex_normals
            
            # 应用视觉元素变换
            link = self.links[link_name]
            if link.visual and 'origin_xyz' in link.visual and 'origin_rpy' in link.visual:
                visual_transform = create_transformation_matrix(
                    link.visual['origin_xyz'], 
                    link.visual['origin_rpy']
                )
                self.meshes[link_name].transform(visual_transform)
                
            # 应用链接变换
            if link_name in self.link_transforms:
                link_transform = self.link_transforms[link_name]
                self.meshes[link_name].transform(link_transform)
                
            # 更新几何体
            self.vis.update_geometry(self.meshes[link_name])
        
    def update_info_display(self):
        """更新信息显示"""
        self.info_text.delete(1.0, tk.END)
        
        info = f"Loaded URDF with:\n"
        info += f"  Links: {len(self.links)}\n"
        info += f"  Joints: {len(self.joints)}\n"
        info += f"  Meshes: {len(self.meshes)}\n"
        info += f"\nJoints:\n"
        
        for joint_name, joint_var in self.joint_variables.items():
            info += f"  {joint_name}: {joint_var.get():.3f}\n"
            
        self.info_text.insert(tk.END, info)

    def close_visualization(self):
        """关闭可视化"""
        self.vis_running = False
        if self.vis_thread:
            self.vis_thread.join(timeout=1.0)


def main():
    root = tk.Tk()
    app = Open3DURDFVisualizer(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close_visualization(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()