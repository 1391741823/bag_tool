import os
import re
import cv2
import rospy
import rosbag
import random
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# ======= 参数区 =======
image_folder = "/media/ao/jiansheng/datasets/LIVO2_dataset/setic_image_png" # 输入文件夹位置
output_bag = "setic_png.bag"    # 输出文件名
topic_name = "/setic/image_raw" # topic名称
image_exts = [".png", ".jpg"]
max_jitter_ns = 2_000_000      # ±2ms 抖动
# =======================

bridge = CvBridge()
bag = rosbag.Bag(output_bag, 'w')

def extract_timestamp(filename):
    """改进版时间戳提取：兼容纯数字或带后缀的文件名"""
    name = os.path.splitext(filename)[0]
    match = re.search(r"^(\d+)", name)
    return int(match.group(1)) if match else None

# 获取并验证图像文件
image_files = []
for f in os.listdir(image_folder):
    if os.path.splitext(f)[1].lower() not in image_exts:
        continue
    ts = extract_timestamp(f)
    if ts is None:
        print(f"[警告] 跳过无法解析时间戳的文件: {f}")
        continue
    image_files.append((ts, f))

if not image_files:
    print("[错误] 未找到有效图像文件，退出。")
    exit(1)

# 按时间戳排序
image_files.sort(key=lambda x: x[0])

print(f"共找到 {len(image_files)} 张有效图像，开始处理...")

min_timestamp = None
max_timestamp = None

# ...existing code...

for i, (file_ts, filename) in enumerate(image_files):
    try:
        # === 时间戳生成 ===
        # 文件时间戳单位为秒，转换为纳秒后加抖动
        jitter_ns = random.randint(-max_jitter_ns, max_jitter_ns)
        current_ts_ns = file_ts * 10**9 + jitter_ns  # 确保单位为纳秒
        
        # 处理时间戳溢出
        secs = current_ts_ns // 10**9
        nsecs = current_ts_ns % 10**9
        ros_time = rospy.Time(secs, nsecs)
        
        # 更新最小/最大时间戳
        if min_timestamp is None or ros_time.to_nsec() < min_timestamp.to_nsec():
            min_timestamp = ros_time
        if max_timestamp is None or ros_time.to_nsec() > max_timestamp.to_nsec():
            max_timestamp = ros_time
        
        # === 图像加载与校验 ===
        img_path = os.path.join(image_folder, filename)
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            print(f"[错误] 无法读取图像: {filename}")
            continue
        h, w = img.shape[:2]
        if h == 0 or w == 0:
            print(f"[错误] 图像尺寸无效: {filename} ({w}x{h})")
            continue
        
        # 通道数检查与修复
        if len(img.shape) != 3 or img.shape[2] != 3:
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                print(f"[修复] 灰度图转换: {filename}")
            else:
                print(f"[错误] 不支持的通道数: {filename} ({img.shape})")
                continue
        
        # === 转换为ROS消息 ===
        try:
            msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except CvBridgeError as e:
            print(f"[CV桥接错误] {filename}: {e}")
            continue
        
        msg.header = Header()
        msg.header.stamp = ros_time
        msg.header.frame_id = "cam0"
        
        # === 写入Bag ===
        bag.write(topic_name, msg, ros_time)
        
        if i % 50 == 0 or i == len(image_files) - 1:
            print(f"[{i+1}/{len(image_files)}] 写入: {filename} @ {ros_time.to_sec():.3f}s")
            
    except Exception as e:
        print(f"[处理失败] {filename}: {str(e)}")

bag.close()

# 计算实际duration
if min_timestamp and max_timestamp:
    duration = max_timestamp - min_timestamp
    print(f"\n✅ Rosbag 已保存至: {output_bag}")
    print(f"   Start Time: {min_timestamp.to_sec():.3f}s")
    print(f"   End Time:   {max_timestamp.to_sec():.3f}s")
    print(f"   Duration:   {duration.to_sec():.3f}s")
else:
    print("\n⚠️ 警告：未能计算有效duration")