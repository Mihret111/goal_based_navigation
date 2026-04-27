import queue
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty, String


class NavTkNode(Node):
    def __init__(self, gui_queue: queue.Queue):
        super().__init__('tkinter_ui_node')

        self.gui_queue = gui_queue

        self.target_pub = self.create_publisher(Pose2D, '/ui/target_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, '/ui/cancel_goal', 10)

        self.status_sub = self.create_subscription(
            String,
            '/ui/nav_status',
            self.status_callback,
            10
        )

        self.result_sub = self.create_subscription(
            String,
            '/ui/nav_result',
            self.result_callback,
            10
        )

        self.get_logger().info('Tkinter UI node started.')

    def status_callback(self, msg: String):
        self.gui_queue.put(("status", msg.data))

    def result_callback(self, msg: String):
        self.gui_queue.put(("result", msg.data))

    def publish_goal(self, x: float, y: float, theta: float):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.target_pub.publish(msg)
        self.gui_queue.put(("local", f"Published goal: x={x:.3f}, y={y:.3f}, theta={theta:.3f}"))

    def publish_cancel(self):
        msg = Empty()
        self.cancel_pub.publish(msg)
        self.gui_queue.put(("local", "Published cancel request."))


class NavTkApp:
    def __init__(self, root: tk.Tk, ros_node: NavTkNode, gui_queue: queue.Queue):
        self.root = root
        self.ros_node = ros_node
        self.gui_queue = gui_queue

        self.root.title("Goal Based Navigation")
        self.root.geometry("760x520")

        self.build_layout()
        self.root.after(100, self.process_gui_queue)

    def build_layout(self):
        # bg_color = '#eaf2f8'  # A very light, subtle blue
        bg_color = '#e4e9f0'  # A light, cool gray-blue
        
        fg_color = '#11224d'  # A deep, distinct blue-black
        
        self.root.configure(bg=bg_color)
        
        style = ttk.Style()
        # Set the default background for all ttk widgets
        style.configure('.', background=bg_color)
        # Configure the label frame titles
        style.configure('TLabelframe.Label', font=('Arial', 11, 'bold'), foreground=fg_color, background=bg_color)
        # Regular labels remain black
        style.configure('TLabel', foreground='black', background=bg_color)

        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(
            main_frame,
            text="Control Panel",
            font=("Arial", 12, "bold"),
            foreground=fg_color
        )
        title.pack(anchor="center", pady=(0, 10))

        input_frame = ttk.LabelFrame(main_frame, text="Target Pose", padding=10)
        input_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(input_frame, text="x (m)").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.x_entry = ttk.Entry(input_frame, width=12)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="y (m)").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.y_entry = ttk.Entry(input_frame, width=12)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(input_frame, text="theta (rad)").grid(row=0, column=4, padx=5, pady=5, sticky="w")
        self.theta_entry = ttk.Entry(input_frame, width=12)
        self.theta_entry.grid(row=0, column=5, padx=5, pady=5)

        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(0, 10))

        clear_btn = ttk.Button(button_frame, text="Clear Log", command=self.clear_log)
        clear_btn.pack(side=tk.RIGHT, padx=(10, 0))

        cancel_btn = ttk.Button(button_frame, text="Cancel Goal", command=self.on_cancel_goal)
        cancel_btn.pack(side=tk.RIGHT, padx=(10, 0))

        send_btn = ttk.Button(button_frame, text="Send Goal", command=self.on_send_goal)
        send_btn.pack(side=tk.RIGHT, padx=(0, 0))

        self.latest_status_var = tk.StringVar(value="Waiting for messages...")
        status_box = ttk.LabelFrame(main_frame, text="Latest Status", padding=10)
        status_box.pack(fill=tk.X, pady=(0, 10))

        latest_status_label = ttk.Label(
            status_box,
            textvariable=self.latest_status_var,
            wraplength=700,
            justify="left"
        )
        latest_status_label.pack(anchor="w")

        # Pack the bottom frame first with side=BOTTOM so it's always visible
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        quit_btn = ttk.Button(bottom_frame, text="Quit", command=self.on_quit)
        quit_btn.pack(side=tk.RIGHT)

        log_box = ttk.LabelFrame(main_frame, text="Event Log", padding=10)
        log_box.pack(fill=tk.BOTH, expand=True)

        self.log_text = scrolledtext.ScrolledText(log_box, wrap=tk.WORD, height=18, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def on_send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            theta = float(self.theta_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter valid numeric values for x, y, and theta.")
            return

        self.ros_node.publish_goal(x, y, theta)

    def on_cancel_goal(self):
        self.ros_node.publish_cancel()

    def clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def append_log(self, prefix: str, text: str):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{prefix}] {text}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def process_gui_queue(self):
        while not self.gui_queue.empty():
            msg_type, text = self.gui_queue.get()

            if msg_type == "status":
                self.latest_status_var.set(text)
                self.append_log("STATUS", text)
            elif msg_type == "result":
                self.append_log("RESULT", text)
            elif msg_type == "local":
                self.append_log("LOCAL", text)

        self.root.after(100, self.process_gui_queue)     # check for new messages every 100ms

    def on_quit(self):
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)

    gui_queue = queue.Queue()
    ros_node = NavTkNode(gui_queue)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = NavTkApp(root, ros_node, gui_queue)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()