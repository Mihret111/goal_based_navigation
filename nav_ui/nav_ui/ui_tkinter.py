import queue
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty, String
from nav_interfaces.msg import NavDiagnostics

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
        self.diagnostics_sub = self.create_subscription(
            NavDiagnostics,
            '/nav/diagnostics',
            self.diagnostics_callback,
            10
        )

        self.get_logger().info('Tkinter UI node started.')

    def diagnostics_callback(self, msg: NavDiagnostics):
        # dictionary for diagnostics data
        diagnostics_data = {
        "current_x": msg.current_x,
        "current_y": msg.current_y,
        "current_theta": msg.current_theta,
        "target_x": msg.target_x,
        "target_y": msg.target_y,
        "target_theta": msg.target_theta,
        "distance_error": msg.distance_error,
        "heading_error": msg.heading_error,
        "linear_cmd": msg.linear_cmd,
        "angular_cmd": msg.angular_cmd,
        "phase": msg.phase,
        "controller_mode": msg.controller_mode,
        "goal_active": msg.goal_active,
        }
        self.gui_queue.put(("diagnostics", diagnostics_data))

    def status_callback(self, msg: String):
        # converts the status message to string
        self.gui_queue.put(("status", msg.data))

    def result_callback(self, msg: String):
        # converts the result message to string
        self.gui_queue.put(("result", msg.data))

    def publish_goal(self, x: float, y: float, theta: float):
        # publishes the goal pose to the target pose topic
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.target_pub.publish(msg)
        self.gui_queue.put(("local", f"Published goal: X={x:.3f}, Y={y:.3f}, Theta={theta:.3f}"))

    def publish_cancel(self):
        # publishes the cancel request to the cancel topic
        msg = Empty()
        self.cancel_pub.publish(msg)
        self.gui_queue.put(("local", "Published cancel request."))

# class for building the GUI application
class NavTkApp:

    # initializes it
    def __init__(self, root: tk.Tk, ros_node: NavTkNode, gui_queue: queue.Queue):
        self.root = root
        self.ros_node = ros_node
        self.gui_queue = gui_queue

        self.root.title("Goal Based Navigation")

        self.collapsed_geometry = "760x530"    # without the event log
        self.expanded_geometry = "760x680"     # with the event log (optionally clicked by user)

        self.root.geometry(self.collapsed_geometry)
        self.log_visible = False  # control the visibility of event log

        self.build_layout()
        self.root.after(100, self.process_gui_queue)

        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)  # handles the window close event


    # builds the layout of the GUI
    def build_layout(self):
        # bg_color = '#eaf2f8'  # A very light blue
        bg_color = '#e4e9f0'  # A light grayish blue
        
        fg_color = '#11224d'  # A deep blue-black
        
        self.root.configure(bg=bg_color)

        self.current_pose_var = tk.StringVar(value="X=--, Y=--, Theta=--")
        self.target_pose_var = tk.StringVar(value="X=--, Y=--, Theta=--")
        self.errors_var = tk.StringVar(value="Distance=--, Heading=--")
        self.commands_var = tk.StringVar(value="V=--, Omega=--")
        self.phase_var = tk.StringVar(value="phase=--")
        self.mode_var = tk.StringVar(value="mode=--")
        self.goal_active_var = tk.StringVar(value="goal_active=--")
        
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

        ttk.Label(input_frame, text="X (m)").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.x_entry = ttk.Entry(input_frame, width=12)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(input_frame, text="Y (m)").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.y_entry = ttk.Entry(input_frame, width=12)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(input_frame, text="Theta (rad)").grid(row=0, column=4, padx=5, pady=5, sticky="w")
        self.theta_entry = ttk.Entry(input_frame, width=12)
        self.theta_entry.grid(row=0, column=5, padx=5, pady=5)

        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(0, 10))

        clear_btn = ttk.Button(button_frame, text="Clear Log", command=self.clear_log)
        clear_btn.pack(side=tk.RIGHT, padx=(10, 0))

        self.toggle_log_btn = ttk.Button(
            button_frame,
            text="Show Event Log",
            command=self.toggle_log
        )
        self.toggle_log_btn.pack(side=tk.RIGHT, padx=(10, 0))

        cancel_btn = ttk.Button(button_frame, text="Cancel Goal", command=self.on_cancel_goal)
        cancel_btn.pack(side=tk.RIGHT, padx=(10, 0))

        send_btn = ttk.Button(button_frame, text="Send Goal", command=self.on_send_goal)
        send_btn.pack(side=tk.RIGHT, padx=(0, 0))

        self.latest_status_var = tk.StringVar(value="Idle")  # set the initial status
        status_box = ttk.LabelFrame(main_frame, text="Latest Status", padding=10)
        status_box.pack(fill=tk.X, pady=(0, 10))

        latest_status_label = ttk.Label(
            status_box,
            textvariable=self.latest_status_var,
            wraplength=700,
            justify="left"
        )
        latest_status_label.pack(anchor="w")

        diag_box = ttk.LabelFrame(main_frame, text="Diagnostic Panel", padding=10)
        diag_box.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(diag_box, text="Current Pose:").grid(row=0, column=0, sticky="w", padx=5, pady=4)
        ttk.Label(diag_box, textvariable=self.current_pose_var).grid(row=0, column=1, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Target Pose:").grid(row=1, column=0, sticky="w", padx=5, pady=4)
        ttk.Label(diag_box, textvariable=self.target_pose_var).grid(row=1, column=1, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Errors:").grid(row=2, column=0, sticky="w", padx=5, pady=4)
        ttk.Label(diag_box, textvariable=self.errors_var).grid(row=2, column=1, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Commands:").grid(row=3, column=0, sticky="w", padx=5, pady=4)
        ttk.Label(diag_box, textvariable=self.commands_var).grid(row=3, column=1, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Phase:").grid(row=0, column=2, sticky="w", padx=15, pady=4)
        ttk.Label(diag_box, textvariable=self.phase_var).grid(row=0, column=3, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Controller Mode:").grid(row=1, column=2, sticky="w", padx=15, pady=4)
        ttk.Label(diag_box, textvariable=self.mode_var).grid(row=1, column=3, sticky="w", padx=5, pady=4)

        ttk.Label(diag_box, text="Goal Active:").grid(row=2, column=2, sticky="w", padx=15, pady=4)
        ttk.Label(diag_box, textvariable=self.goal_active_var).grid(row=2, column=3, sticky="w", padx=5, pady=4)

        # Pack the bottom frame first with side=BOTTOM so it's always visible
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        
        quit_btn = ttk.Button(bottom_frame, text="Quit", command=self.on_quit)
        quit_btn.pack(side=tk.RIGHT)

        self.log_box = ttk.LabelFrame(main_frame, text="Event Log", padding=10)

        self.log_text = scrolledtext.ScrolledText(
            self.log_box,
            wrap=tk.WORD,
            height=8,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # called when the send goal button is clicked
    def on_send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            theta = float(self.theta_entry.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter valid numeric values for X, Y, and Theta.")
            return

        self.ros_node.publish_goal(x, y, theta)
    
    # called when the cancel goal button is clicked
    def on_cancel_goal(self):
        self.ros_node.publish_cancel()
    
    # called when the clear log button is clicked
    def clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # appends the log to the log text
    def append_log(self, prefix: str, text: str):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{prefix}] {text}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # toggles the visibility of the log box
    def toggle_log(self):
        if self.log_visible:
            self.log_box.pack_forget()
            self.toggle_log_btn.config(text="Show Event Log")
            self.root.geometry(self.collapsed_geometry)
            self.log_visible = False
        else:
            self.log_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
            self.root.update_idletasks()
            self.toggle_log_btn.config(text="Hide Event Log")
            self.root.geometry(self.expanded_geometry)
            self.log_visible = True

    # processes the GUI queue
    def process_gui_queue(self):
        while not self.gui_queue.empty():
            msg_type, text = self.gui_queue.get()

            if msg_type == "status":
                self.handle_status_message(text)
                self.append_log("STATUS", text)
            elif msg_type == "result":
                self.handle_result_message(text)
                self.append_log("RESULT", text)
            elif msg_type == "local":
                self.append_log("LOCAL", text)
            elif msg_type == "diagnostics":
                self.update_diagnostics_display(text)

        self.root.after(100, self.process_gui_queue)     # check for new messages every 100ms

    # updates the diagnostics display
    def update_diagnostics_display(self, data: dict):
        self.current_pose_var.set(
            f"X={data['current_x']:.3f}, Y={data['current_y']:.3f}, Theta={data['current_theta']:.3f}"
        )
        self.target_pose_var.set(
            f"X={data['target_x']:.3f}, Y={data['target_y']:.3f}, Theta={data['target_theta']:.3f}"
        )
        self.errors_var.set(
            f"Distance={data['distance_error']:.3f}, Heading={data['heading_error']:.3f}"
        )
        self.commands_var.set(
            f"V={data['linear_cmd']:.3f}, Omega={data['angular_cmd']:.3f}"
        )
        self.phase_var.set(data["phase"])
        mode_str = data["controller_mode"]
        self.mode_var.set(mode_str[0].upper() + mode_str[1:])
        self.goal_active_var.set(str(data["goal_active"]))

        # if the goal is active, the status is set to "Navigating towards goal"
        # else, the status is set to "Idle"
        if data["goal_active"]:
            self.latest_status_var.set("Navigating towards goal")
        else:
            current_status = self.latest_status_var.get().strip().lower()
            if current_status == "navigating towards goal":
                self.latest_status_var.set("Idle")

    # handle the status messages
    def handle_status_message(self, text: str):
        lower_text = text.lower()

        if "cancel" in lower_text:
            self.latest_status_var.set("Goal canceled")
        elif "finished" in lower_text:
            self.latest_status_var.set("Action finished")
        elif "succeed" in lower_text:
            self.latest_status_var.set("Action finished successfully")
        elif "abort" in lower_text:
            self.latest_status_var.set("Action aborted")
        elif "server not available" in lower_text:
            self.latest_status_var.set("Action server not available")

    def handle_result_message(self, text: str):
        lower_text = text.lower()

        if "canceled" in lower_text:
            self.latest_status_var.set("Goal canceled")
        elif "succeeded" in lower_text or "success=true" in lower_text:
            self.latest_status_var.set("Action finished successfully")
        elif "aborted" in lower_text or "success=false" in lower_text:
            self.latest_status_var.set("Action aborted")
        else:
            self.latest_status_var.set("Action finished")
        # called when the quit button is clicked
    def on_quit(self):
        self.root.quit()
        self.root.destroy()

# main function
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
        # send a cancel command to the action server
        ros_node.cancel_pub.publish(Empty())  # cancel any active goal, just in case
        try:
            ros_node.destroy_node()    # destroy the node
        except Exception as e:
            print(f"Error destroying node: {e}")
        
        if rclpy.ok():
            rclpy.shutdown()    # shutdown the node
            try:
                root.destroy()
            except:
                pass


if __name__ == '__main__':
    main()