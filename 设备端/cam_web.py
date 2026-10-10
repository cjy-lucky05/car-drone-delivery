#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机画面【网页查看】节点 —— 不需要图形界面，用电脑浏览器看

【为什么用它】
  小车上 image_view / rqt 弹不出窗口（X 转发、OpenCV 图形后端等问题）时，
  这个节点把相机画面转成 MJPEG 流挂在网页上，用【电脑浏览器】看即可 ✓

【怎么跑】（小车上）
    source ~/catkin_ws/devel/setup.bash
    python3 cam_web.py                 # 默认端口 8088，话题 /arm_camera/image_raw
    python3 cam_web.py 8090 /arm_camera/image_raw    # 自定义端口/话题

【怎么看】
    电脑浏览器打开：  http://192.168.43.151:8088
    （IP 换成小车的 IP；同一网络下直接开就行）

【依赖】
    cv_bridge + cv2（ROS 桌面版自带）。先自检：
        python3 -c "import cv_bridge, cv2; print('ok')"
    如果这句报错 → 告诉我，我给你换成不用 cv_bridge 的版本。
"""
import sys
import time
import threading
import http.server
import socketserver

import rospy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
except ImportError as e:
    print("❌ 缺依赖：%s" % e)
    print("   自检：python3 -c \"import cv_bridge, cv2; print('ok')\"")
    sys.exit(1)

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8088
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/arm_camera/image_raw"

_lock = threading.Lock()
_latest = {"jpg": None, "t": 0.0, "n": 0}
_bridge = CvBridge()

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>相机画面</title>
<style>
 body{background:#0f172a;color:#e2e8f0;font-family:system-ui,sans-serif;
      margin:0;padding:18px;text-align:center}
 h1{font-size:16px;font-weight:600;color:#94a3b8;margin:0 0 12px}
 img{max-width:100%;border:2px solid #334155;border-radius:10px;background:#000}
 .m{color:#64748b;font-size:13px;margin-top:10px}
 code{background:#1e293b;padding:2px 6px;border-radius:4px;color:#38bdf8}
</style></head><body>
<h1>相机画面 · TOPIC_PLACEHOLDER</h1>
<img src="/stream">
<div class="m">看不到画面？确认相机在发图：<code>rostopic hz TOPIC_PLACEHOLDER</code></div>
</body></html>"""


def on_image(msg):
    try:
        frame = _bridge.imgmsg_to_cv2(msg, "bgr8")
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with _lock:
                _latest["jpg"] = buf.tobytes()
                _latest["t"] = time.time()
                _latest["n"] += 1
    except Exception:
        pass


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                                  # 别刷屏

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg, n = _latest["jpg"], _latest["n"]
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode() +
                                         b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.06)          # ~16fps 推流，够看
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        body = PAGE.replace("TOPIC_PLACEHOLDER", TOPIC).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    rospy.init_node("cam_web", anonymous=True)
    rospy.Subscriber(TOPIC, Image, on_image, queue_size=1, buff_size=2 ** 24)

    srv = socketserver.ThreadingTCPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    print("=" * 74)
    print(" 相机画面网页查看   话题 %s" % TOPIC)
    print(" ★ 电脑浏览器打开： http://<小车IP>:%d" % PORT)
    print("   例如： http://192.168.43.151:%d" % PORT)
    print(" 停止：Ctrl+C")
    print("=" * 74)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        age = time.time() - _latest["t"] if _latest["t"] else None
        if age is None:
            print("[cam_web] 还没收到图像 → 检查：话题名对不对 / 相机在发图吗"
                  "（rostopic hz %s）" % TOPIC)
        elif age > 3:
            print("[cam_web] 画面停了 %.0f 秒（相机卡了？）" % age)
        r.sleep()


if __name__ == "__main__":
    main()
