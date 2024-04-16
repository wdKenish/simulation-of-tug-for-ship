
# 在全局范围内定义一个变量，用于存储椭圆的ID
oval_id = None
# 导入 tkinter 库，用于创建 GUI
import tkinter as tk
from PIL import Image, ImageDraw, ImageTk
# 定义一个函数，用于绘制船体
def draw_ship():
    global oval_id
    # 从输入框获取长度和宽度，并转换为浮点数
    ship_length = float(length_entry.get())
    ship_width = float(width_entry.get())
    ship_center = center_entry.get().split(',')
    center_x = float(ship_center[0])
    center_y = float(ship_center[1])
    #ship_angle=float(angle_entry.get())
    # 清除画布上的所有内容
    canvas.delete("all")
    # 创建一个新的图像
    img = Image.new('RGBA', (500, 500), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    # 在图像上绘制一个椭圆
    draw.ellipse([(center_x - ship_length / 2, center_y - ship_width / 2), 
                  (center_x + ship_length / 2, center_y + ship_width / 2)], 
                 fill='blue')
    # 旋转图像
    img = img.rotate(float(angle_entry.get()))
    # 将图像添加到画布上
    tk_img = ImageTk.PhotoImage(img)
    oval_id = canvas.create_image(0, 0, image=tk_img, anchor='nw')
    canvas.image = tk_img  # keep a reference to the image
    canvas.create_line(0, 0, 500, 0, fill='red',width=10) #x轴
    canvas.create_line(0, 0, 0, 500, fill='red',width=10) #y轴

def move_ship():
    # 获取椭圆当前的位置
    current_pos = canvas.coords(oval_id)
    ship_length = float(length_entry.get())
    ship_width = float(width_entry.get())
    # 计算椭圆的中心
    center_x = current_pos[0] + ship_length / 2
    center_y = current_pos[1] + ship_width / 2
    # 检查椭圆的中心是否已经到达目标位置
    # 获取目标位置
    target_center = target_center_entry.get().split(',')
    target_x = float(target_center[0])
    target_y = float(target_center[1])
   
    # 计算移动速度
    speed_x = (target_x - center_x) / 100
    speed_y = (target_y - center_y) / 100
    if center_x < 400 and center_y < 300:
        # 移动椭圆
        canvas.move(oval_id, speed_x, speed_y)
        # 每100毫秒后再次调用move_ship函数，形成一个循环
        root.after(100, move_ship)
# 创建一个 tkinter 窗口
root = tk.Tk()

# 创建一个标签，显示 "ship_Length:"
length_label = tk.Label(root, text="ship_Length:")
# 将标签添加到窗口
length_label.pack()

# 创建一个输入框，用于输入船长
length_entry = tk.Entry(root)
# 将输入框添加到窗口
length_entry.pack()

# 创建一个标签，显示 "ship_Width:"
width_label = tk.Label(root, text="ship_Width:")
# 将标签添加到窗口
width_label.pack()

# 创建一个输入框，用于输入船宽
width_entry = tk.Entry(root)
# 将输入框添加到窗口
width_entry.pack()

# 创建一个标签，显示 "ship_center:"
center_label = tk.Label(root, text="ship_center:")
# 将标签添加到窗口
center_label.pack()

# 创建一个输入框，用于输入船中坐标
center_entry = tk.Entry(root)
# 将输入框添加到窗口
center_entry.pack()

# 创建一个标签，显示 "ship_angle:"
angle_label = tk.Label(root, text="ship_angle:")
# 将标签添加到窗口
angle_label.pack()

# 创建一个输入框，用于输入船的旋转角度
angle_entry = tk.Entry(root)
# 将输入框添加到窗口
angle_entry.pack()

# 创建一个标签，显示 "target_center:"
target_center_label = tk.Label(root, text="target_center:")
# 将标签添加到窗口
target_center_label.pack()

# 创建一个输入框，用于输入目标x坐标
target_center_entry = tk.Entry(root)
# 将输入框添加到窗口
target_center_entry.pack()

# 创建一个按钮，点击时调用 draw_ship 函数
draw_button = tk.Button(root, text="Run", command=draw_ship)
# 将按钮添加到窗口
draw_button.pack()


# 创建一个按钮，点击时调用 move_ship 函数
move_button = tk.Button(root, text="Move", command=move_ship)
# 将按钮添加到窗口
move_button.pack()

# 创建一个画布，用于绘制矩形
canvas = tk.Canvas(root, width=500, height=500)
# 将画布添加到窗口
canvas.pack()

# 进入 tkinter 的主循环，等待用户操作
root.mainloop()
